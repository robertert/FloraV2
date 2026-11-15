/**
 * @file flora_magcal.h
 * @brief Kalibracja magnetometru: hard iron 3D + soft iron w poziomie.
 *
 * Zrodla bledow (wszystkie przyklejone do plytki glowicy):
 *   hard iron  - stale przesuniecie: WLASNE zero AK09916 (nota ICM-20948,
 *                tabela 3: do +-2000 LSB = +-300 uT na os; nasz egzemplarz
 *                ~40 uT w osi Z) plus namagnesowane czesci obok,
 *   soft iron  - zelazo "zaginajace" pole ziemskie, splaszcza okrag
 *                w elipse -> macierz 2x2.
 *
 * ================= MODEL =================
 * m - pole w osiach akcelerometru (sterownik juz przemapowal AK09916):
 *
 *     m'  = m - o                      o = (ox, oy, oz), os plytki
 *     L   = poziomowanie(m', przechyl)  surowy przechyl PLYTKI wzgledem
 *                                      grawitacji, bez offsetu level_cal
 *     c   = W * (Lx, Ly)                soft iron w poziomie
 *     kurs = atan2(cy, cx)              kompasowy, CW
 *
 * Poziomowanie po odjeciu CALEGO o, takze oz - inaczej przy przechyle
 * oz przecieka do osi poziomych: 40 uT * sin(10 st) = 7 uT, czyli ~20 st
 * bledu kursu na skarpie. Stary model (2D na surowych x, y) przy plytce
 * przechylonej w czasie kalibracji zapamietywal ten przeciek jako hard iron
 * i odejmowal go potem drugi raz.
 *
 * ================= SKAD oz =================
 * Robot obraca sie tylko wokol pionu, wiec oz i pionowa skladowa pola
 * Ziemi sa z ruchu nierozroznialne - widac tylko ich sume. Rozdziela je
 * znana inklinacja miejsca pracy (Krakow ~66 st): pionowa ziemska to
 * pozioma * tan(inklinacji), a reszta to oz. Pozioma = R, promien okregu
 * po korekcji. Przy 66 st tan rosnie szybko (1/cos^2 ~ 6 na radian):
 * blad inklinacji o 1 st to ~2 uT bledu oz i ~2 st kursu na 10 st
 * przechylu (host/test.c). Inklinacje wpisac z kalkulatora NOAA/WMM dla
 * miejsca pracy, z dokladnoscia lepsza niz 0,5 st - jak deklinacje.
 *
 * Dopasowanie iteracyjne: poziomowanie z biezacym o -> elipsa (srodek,
 * W, R) -> poprawka o -> ... Kilka obrotow wystarcza, bo sprzezenie oz
 * z srodkiem w poziomie jest rzedu sin(przechylu).
 *
 * Elipsa: stozkowa A x^2 + B xy + C y^2 + D x + E y = 1 metoda najmniejszych
 * kwadratow (uklad 5x5), na danych znormalizowanych i w double.
 *
 * Czysty C, bez ESP-IDF - sprawdzane w host/test.c.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 120 s przy 10 Hz. Bufor statyczny ~24 kB (pole 3D + przechyl na probke). */
#define FLORA_MAGCAL_MAX_SAMPLES 1200

/** Wynik kalibracji, ida tez po kablu (pole status w MAG_CAL_RESULT). */
typedef enum {
    FLORA_MAGCAL_OK          = 0,
    FLORA_MAGCAL_TOO_FEW     = 1,  /**< za malo probek */
    FLORA_MAGCAL_COVERAGE    = 2,  /**< nie obrocono pelnego kola */
    FLORA_MAGCAL_FIT         = 3,  /**< punkty nie ukladaja sie w elipse */
    FLORA_MAGCAL_RESIDUAL    = 4,  /**< za duzy rozrzut wokol elipsy */
    FLORA_MAGCAL_AXIS_RATIO  = 5,  /**< elipsa zbyt splaszczona - zelazo tuz obok czujnika */
} flora_magcal_status_t;

/** Parametry korekty, trzymane w NVS glowicy. */
typedef struct {
    float ox, oy, oz;    /**< hard iron w osiach plytki [uT] */
    float w[2][2];       /**< soft iron w poziomie, symetryczna */
    float h_ref_ut;      /**< promien okregu po korekcji = pole poziome */
    float dip_ref_deg;   /**< inklinacja po korekcji (ujemna: pole w dol, Z w gore) */
} flora_magcal_t;

/** Raport z dopasowania - do logu, telemetrii i pracy. */
typedef struct {
    flora_magcal_status_t status;
    uint16_t n;              /**< liczba probek */
    uint8_t  sectors;        /**< pokryte sektory po 30 st, z 12 */
    float    axis_ratio;     /**< dluzsza / krotsza os elipsy */
    float    rms_pct;        /**< rozrzut promienia po korekcji, % */
} flora_magcal_report_t;

typedef struct {
    float x[FLORA_MAGCAL_MAX_SAMPLES];
    float y[FLORA_MAGCAL_MAX_SAMPLES];
    float z[FLORA_MAGCAL_MAX_SAMPLES];
    float pitch[FLORA_MAGCAL_MAX_SAMPLES];   /**< surowy przechyl plytki [st] */
    float roll[FLORA_MAGCAL_MAX_SAMPLES];
    int   n;
} flora_magcal_collector_t;

/** Wynik korekty jednego pomiaru. */
typedef struct {
    float heading_deg;   /**< kompasowy, CW, 0..360 */
    float h_ut;          /**< pole poziome po korekcji */
    float dip_deg;       /**< inklinacja po korekcji (ujemna) */
    float total_ut;      /**< |m - o| */
} flora_magcal_out_t;

/** Korekta tozsamosciowa (brak kalibracji). */
void flora_magcal_identity(flora_magcal_t *cal);

void flora_magcal_begin(flora_magcal_collector_t *c);

/**
 * @param m            pole w osiach akcelerometru [uT], surowe
 * @param pitch_deg    surowy przechyl plytki w chwili pomiaru (head_attitude_t)
 * @param roll_deg
 * @return false gdy bufor pelny
 */
bool flora_magcal_add(flora_magcal_collector_t *c, const float m[3],
                      float pitch_deg, float roll_deg);

/**
 * Dopasowanie. Przy OK wypelnia cal; przy bledzie cal nietkniety.
 * @param dip_site_deg  inklinacja miejsca pracy, dodatnia (Krakow ~66)
 */
flora_magcal_report_t flora_magcal_fit(const flora_magcal_collector_t *c, float dip_site_deg,
                                       flora_magcal_t *cal);

/**
 * Pole do poziomu: obroty o przechyl jak w head_attitude_from_accel()
 * (Z-Y-X, X przod, Y lewo, Z gora).
 */
void flora_magcal_level(const float m[3], float pitch_deg, float roll_deg, float out[3]);

/** Pelna korekta pomiaru: o, poziomowanie, W. */
void flora_magcal_apply(const flora_magcal_t *cal, const float m[3],
                        float pitch_deg, float roll_deg, flora_magcal_out_t *out);

const char *flora_magcal_status_name(flora_magcal_status_t s);

#ifdef __cplusplus
}
#endif
