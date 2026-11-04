/**
 * @file flora_sungate.h
 * @brief Bramki kompasu slonecznego: czy pomiar z krzyza VEML to naprawde slonce.
 *
 * Zly kurs ze slonca (lampa w pomieszczeniu, odbicie od szyby, slonce
 * nisko nad horyzontem) jest grozniejszy niz brak kursu. Glownym zrodlem
 * kursu jest magnetometr (przez glowice), slonce tylko go dostraja.
 * Pomiar przechodzi kolejno:
 *
 *   1 swiatlo   najjasniejszy kanal >= min_lux (15 000 lx). Bezposrednie
 *               slonce to 30-100 tys. lx, oswietlenie we wnetrzu ~0,3-1 tys.
 *   2 niebo     znany czas UTC i slonce > min_elev nad horyzontem (NOAA).
 *               W nocy albo bez zegara slonca po prostu nie ma.
 *   3 stalosc   ostatnie 3 pomiary daja ten sam kurs osi mapy (psi + theta)
 *               w granicach stable_deg. Ta wielkosc nie zalezy od obrotu
 *               robota, a odbicia i cienie skacza.
 *   4 zgodnosc  z magnetometrem: |psi_slonce - psi_glowica| <= mag_max_deg,
 *               gdy glowica ma swiezy kurs z MAG_VALID. Magnetometr ma
 *               pierwszenstwo: slonce moze go dostroic o kilka stopni,
 *               ale nie przelamac.
 *
 * Przyjety pomiar dostaje sigma_deg (15 st; glowica ~4 st), wiec waga
 * magnetometru w EKF jest kilkanascie razy wieksza. Bez magnetometru
 * slonce jest jedynym kursem bezwzglednym - bramka 4 odpada, a sigma
 * rosnie do sigma_nomag_deg.
 *
 * Czysty C, bez ESP-IDF - sprawdzane w host/test.c.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FLORA_SUNGATE_STABLE_N 3

/** Wynik oceny; wartosci ida do telemetrii (pole gate w tele/nav_sun). */
typedef enum {
    FLORA_SUNGATE_NONE         = -1,  /**< jeszcze nic nie oceniano */
    FLORA_SUNGATE_OK           = 0,
    FLORA_SUNGATE_DIM          = 1,   /**< za malo swiatla - nie slonce */
    FLORA_SUNGATE_LOW          = 2,   /**< brak czasu UTC albo slonce nisko */
    FLORA_SUNGATE_UNSTABLE     = 3,   /**< kolejne pomiary niezgodne (albo za malo) */
    FLORA_SUNGATE_MAG_CONFLICT = 4,   /**< niezgodny z magnetometrem */
} flora_sungate_result_t;

typedef struct {
    double lat_deg, lon_deg;   /**< miejsce pracy */
    float  declination_deg;    /**< deklinacja magnetyczna, wschodnia dodatnia */
    float  min_lux;
    float  min_elev_deg;
    float  stable_deg;
    float  mag_max_deg;
    float  sigma_deg;
    float  sigma_nomag_deg;
} flora_sungate_cfg_t;

#define FLORA_SUNGATE_DEFAULT_CFG()   \
    {                                 \
        .min_lux         = 15000.0f,  \
        .min_elev_deg    = 10.0f,     \
        .stable_deg      = 10.0f,     \
        .mag_max_deg     = 20.0f,     \
        .sigma_deg       = 15.0f,     \
        .sigma_nomag_deg = 20.0f,     \
    }

typedef struct {
    flora_sungate_cfg_t cfg;
    float b[FLORA_SUNGATE_STABLE_N];  /**< kurs osi mapy z kolejnych pomiarow */
    int   n;
} flora_sungate_t;

typedef struct {
    float   lux[4];              /**< N, E, S, W w luksach */
    float   azimuth_meas_deg;    /**< azymut slonca wzgledem glowicy, CW */
    int64_t unix_s;              /**< czas UTC; <= 0 gdy nieznany */
    float   th_ref_rad;          /**< biezacy kat robota w mapie (z EKF) */
    bool    head_mag_ok;         /**< swiezy kurs glowicy z MAG_VALID */
    float   head_psi_rad;        /**< ten kurs (kompasowy) */
} flora_sungate_in_t;

typedef struct {
    float psi_rad;      /**< kurs magnetyczny glowicy wg slonca (gdy OK) */
    float sigma_rad;    /**< niepewnosc do EKF (gdy OK) */
    float vs_mag_deg;   /**< slonce minus magnetometr; 0 gdy nie liczono */
} flora_sungate_out_t;

void flora_sungate_init(flora_sungate_t *g, const flora_sungate_cfg_t *cfg);

/** Zapomina historie stalosci - np. po recznym ustawieniu pozy. */
void flora_sungate_reset(flora_sungate_t *g);

flora_sungate_result_t flora_sungate_eval(flora_sungate_t *g, const flora_sungate_in_t *in,
                                          flora_sungate_out_t *out);

const char *flora_sungate_name(flora_sungate_result_t r);

#ifdef __cplusplus
}
#endif
