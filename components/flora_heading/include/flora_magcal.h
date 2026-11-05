/**
 * @file flora_magcal.h
 * @brief Kalibracja magnetometru: hard iron + soft iron w plaszczyznie poziomej.
 *
 * Kazdy kawalek zelaza i kazdy przewod z pradem w poblizu czujnika
 * znieksztalca pole:
 *   hard iron  - stale pole od namagnesowanych czesci (sruby, glosniki,
 *                magnesy silnikow), przesuwa okrag pomiarow -> offset,
 *   soft iron  - zelazo "zaginajace" pole ziemskie, splaszcza okrag
 *                w elipse -> macierz 2x2.
 * Bez korekty blad kursu ZALEZY OD KURSU (siega kilkudziesieciu stopni),
 * wiec zaden staly offset dalej w lancuchu go nie usunie.
 *
 * PROCEDURA: robot obraca sie powoli w miejscu 2-3 razy (albo glowice
 * obraca sie recznie), zbieramy surowe (x, y) z osi czujnika. Punkty leza
 * na elipsie; dopasowanie elipsy daje srodek (hard iron) i macierz W,
 * ktora zamienia elipse w okrag o tym samym polu (soft iron):
 *
 *     m_skorygowane = W * (m - o)
 *
 * DLACZEGO 2D: robot obraca sie tylko wokol pionu, wiec offset osi Z jest
 * z tego ruchu nieobserwowalny. Maszt stoi pionowo, przechyly trawnika
 * to kilka stopni - wplyw nieskorygowanej osi Z na kurs jest wtedy maly.
 * Pelna kalibracja 3D wymagalaby obracania glowicy we wszystkich
 * kierunkach (zdjetej z masztu) - ograniczenie do opisania w pracy.
 *
 * Dopasowanie: stozkowa A x^2 + B xy + C y^2 + D x + E y = 1 metoda
 * najmniejszych kwadratow (uklad 5x5), na danych znormalizowanych
 * i w double - w float uklad bylby zle uwarunkowany.
 *
 * Czysty C, bez ESP-IDF - sprawdzane w host/test.c.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 120 s przy 10 Hz. Bufor statyczny ~10 kB. */
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
    float ox, oy;        /**< hard iron [uT] */
    float w[2][2];       /**< soft iron, symetryczna */
    float h_ref_ut;      /**< promien okregu po korekcji = pole poziome */
    float dip_ref_deg;   /**< inklinacja w chwili kalibracji (do wykrywania zaklocen) */
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
    float  x[FLORA_MAGCAL_MAX_SAMPLES];
    float  y[FLORA_MAGCAL_MAX_SAMPLES];
    int    n;
    double v_sum;    /**< skladowa pionowa, do inklinacji odniesienia */
} flora_magcal_collector_t;

/** Korekta tozsamosciowa (brak kalibracji). */
void flora_magcal_identity(flora_magcal_t *cal);

void flora_magcal_begin(flora_magcal_collector_t *c);

/**
 * @param x, y  surowe pole w osiach czujnika [uT]
 * @param v     skladowa pionowa po korekcji przechylu [uT]
 * @return false gdy bufor pelny
 */
bool flora_magcal_add(flora_magcal_collector_t *c, float x, float y, float v);

/** Dopasowanie. Przy OK wypelnia cal; przy bledzie cal nietkniety. */
flora_magcal_report_t flora_magcal_fit(const flora_magcal_collector_t *c, flora_magcal_t *cal);

/** m_out = W (m - o), tylko osie x, y. */
void flora_magcal_apply(const flora_magcal_t *cal, float x, float y, float *out_x, float *out_y);

const char *flora_magcal_status_name(flora_magcal_status_t s);

#ifdef __cplusplus
}
#endif
