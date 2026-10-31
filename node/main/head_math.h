/**
 * @file head_math.h
 * @brief Matematyka fuzji kursu i kompasu slonecznego.
 *
 * Wydzielone celowo do naglowka bez zaleznosci od ESP-IDF: to czyste
 * funkcje nad liczbami, wiec kompiluja sie zwyklym gcc i daja sie
 * przetestowac na hoscie. Ten sam zabieg na parserze ramek wylapal realny
 * blad, ktorego na plytce szukalbym godzinami.
 */
#pragma once

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define HEAD_RAD2DEG (180.0f / (float)M_PI)
#define HEAD_DEG2RAD ((float)M_PI / 180.0f)

/** Normalizuje kat do zakresu 0..360. */
static inline float head_wrap360(float deg)
{
    deg = fmodf(deg, 360.0f);
    return deg < 0.0f ? deg + 360.0f : deg;
}

/** Najkrotsza roznica katow, w zakresie -180..180. Do korekty w filtrze. */
static inline float head_angle_diff(float a, float b)
{
    float d = fmodf(a - b + 540.0f, 360.0f) - 180.0f;
    return d;
}

/** Pochylenie i przechyl z akcelerometru, w stopniach. */
typedef struct {
    float pitch_deg;
    float roll_deg;
    float magnitude_g;   /**< wypadkowa; odstepstwo od 1 g znaczy ruch */
} head_attitude_t;

/**
 * @brief Wyznacza pochylenie i przechyl z wektora przyspieszenia.
 *
 * Uklad osi: Z w gore (maszt pionowo = Z okolo +1 g), X do przodu,
 * Y w lewo. Przy innym montazu trzeba przestawic znaki albo zamienic osie.
 */
static inline head_attitude_t head_attitude_from_accel(const float a[3])
{
    head_attitude_t out;
    out.magnitude_g = sqrtf(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
    /* atan2 z dwoma argumentami jest odporne na a[2] == 0, w przeciwienstwie
     * do atan(a[0]/a[2]) - przy maszcie polozonym poziomo to realny przypadek. */
    out.pitch_deg = atan2f(-a[0], sqrtf(a[1]*a[1] + a[2]*a[2])) * HEAD_RAD2DEG;
    out.roll_deg  = atan2f( a[1], a[2]) * HEAD_RAD2DEG;
    return out;
}

/**
 * @brief Kurs magnetyczny z korekta przechylu.
 *
 * Bez korekty kurs z magnetometru jest poprawny TYLKO przy pionowym maszcie.
 * Na pochylym trawniku skladowa pionowa pola ziemskiego przecieka do osi
 * poziomych i blad siega kilkunastu stopni przy 15 stopniach przechylu -
 * czyli tyle, ile cala dokladnosc, o ktora walczymy.
 *
 * Rzutujemy wiec wektor pola na plaszczyzne pozioma, obracajac go
 * o zmierzone pochylenie i przechyl.
 *
 * @param m  pole magnetyczne w uT, osie jak w akcelerometrze
 * @param at pochylenie i przechyl z head_attitude_from_accel()
 * @return kurs w stopniach, 0..360, liczony od osi X w strone Y
 */
static inline float head_heading_tilt_compensated(const float m[3],
                                                  head_attitude_t at)
{
    float p = at.pitch_deg * HEAD_DEG2RAD;
    float r = at.roll_deg  * HEAD_DEG2RAD;
    float sp = sinf(p), cp = cosf(p);
    float sr = sinf(r), cr = cosf(r);

    float mx = m[0]*cp + m[1]*sr*sp + m[2]*cr*sp;
    float my = m[1]*cr - m[2]*sr;

    return head_wrap360(atan2f(my, mx) * HEAD_RAD2DEG);
}

/** Czy pole magnetyczne wyglada na ziemskie (okolo 25-65 uT na swiecie). */
static inline bool head_mag_plausible(const float m[3])
{
    float f = sqrtf(m[0]*m[0] + m[1]*m[1] + m[2]*m[2]);
    /* Szerzej niz sam zakres ziemski, bo blachy w robocie przesuwaja
     * wartosc - ale wystarczajaco waska, by wylapac magnes albo przewod
     * pod duzym pradem w poblizu. */
    return f > 15.0f && f < 90.0f;
}

/* ================= KOMPAS SLONECZNY ================= */

#define HEAD_SUN_CHANNELS 4

typedef struct {
    float azimuth_deg;   /**< kierunek na slonce wzgledem glowicy */
    float contrast;      /**< 0..1, o ile kanaly sie roznia */
    bool  valid;
} head_sun_t;

/**
 * @brief Azymut slonca z czterech czujnikow ustawionych w krzyz.
 *
 * Kanaly w kolejnosci N, E, S, W - czyli 0, 90, 180, 270 stopni wzgledem
 * glowicy. Liczymy wazona sume wektorowa: kazdy czujnik ciagnie w swoim
 * kierunku z sila proporcjonalna do natezenia.
 *
 * ODEJMUJEMY NAJMNIEJSZY KANAL przed wazeniem. Bez tego swiatlo rozproszone,
 * ktore pada na wszystkie cztery rowno, rozmywa wynik w strone zera -
 * a to wlasnie rozrzut miedzy kanalami nosi informacje o kierunku.
 *
 * @param lux            natezenie na kanalach N, E, S, W
 * @param min_lux        ponizej tego uznajemy, ze jest za ciemno
 * @param min_contrast   ponizej tego uznajemy pelne zachmurzenie
 */
static inline head_sun_t head_sun_azimuth(const float lux[HEAD_SUN_CHANNELS],
                                          float min_lux, float min_contrast)
{
    head_sun_t out = { .azimuth_deg = 0.0f, .contrast = 0.0f, .valid = false };

    float lo = lux[0], hi = lux[0], sum = 0.0f;
    for (int i = 0; i < HEAD_SUN_CHANNELS; i++) {
        if (lux[i] < lo) lo = lux[i];
        if (lux[i] > hi) hi = lux[i];
        sum += lux[i];
    }
    if (hi < min_lux) {
        return out;            /* noc albo gleby cien */
    }

    /* Kontrast jako udzial rozrzutu w najjasniejszym kanale. Przy pelnym
     * zachmurzeniu wszystkie cztery sa niemal rowne i kierunku nie ma. */
    out.contrast = (hi - lo) / hi;
    if (out.contrast < min_contrast) {
        return out;
    }

    /* Kierunki kanalow: N=0, E=90, S=180, W=270 stopni. */
    static const float dir_x[HEAD_SUN_CHANNELS] = {  1.0f,  0.0f, -1.0f,  0.0f };
    static const float dir_y[HEAD_SUN_CHANNELS] = {  0.0f,  1.0f,  0.0f, -1.0f };

    float vx = 0.0f, vy = 0.0f;
    for (int i = 0; i < HEAD_SUN_CHANNELS; i++) {
        float w = lux[i] - lo;
        vx += w * dir_x[i];
        vy += w * dir_y[i];
    }
    if (fabsf(vx) < 1e-6f && fabsf(vy) < 1e-6f) {
        return out;
    }

    out.azimuth_deg = head_wrap360(atan2f(vy, vx) * HEAD_RAD2DEG);
    out.valid = true;
    (void)sum;
    return out;
}

/**
 * @brief Filtr komplementarny kursu.
 *
 * Zyroskop daje szybkie zmiany bez opoznienia, ale dryfuje. Magnetometr
 * jest stabilny bezwzglednie, ale szumi i reaguje na zelazo. Laczymy:
 * calkujemy zyroskop i delikatnie ciagniemy wynik w strone magnetometru.
 *
 * @param heading    biezacy kurs
 * @param rate_dps   predkosc katowa wokol pionu
 * @param dt_s       czas od poprzedniej probki
 * @param mag_heading kurs z magnetometru; ignorowany gdy use_mag = false
 * @param tau_s      stala czasowa korekty; wieksza = wiecej zaufania zyro
 */
static inline float head_fuse_heading(float heading, float rate_dps, float dt_s,
                                      float mag_heading, bool use_mag, float tau_s)
{
    heading = head_wrap360(heading + rate_dps * dt_s);
    if (!use_mag || tau_s <= 0.0f) {
        return heading;
    }
    /* Waga korekty rosnie z dt, wiec filtr zachowuje sie tak samo
     * niezaleznie od tego, czy tick przyszedl punktualnie. */
    float alpha = dt_s / (tau_s + dt_s);
    return head_wrap360(heading + alpha * head_angle_diff(mag_heading, heading));
}
