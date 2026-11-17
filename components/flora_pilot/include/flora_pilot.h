/**
 * @file flora_pilot.h
 * @brief Jazda do punktu i obrot do kursu dla napedu rozniczkowego.
 *
 * Czysty C bez zaleznosci od ESP-IDF (host/test.c z symulacja).
 *
 * Wejscie: poza z EKF (x, y w mm, theta w stopniach CCW od osi X mapy).
 * Wyjscie: zadane predkosci kol w mm/s dla petli PID.
 *
 * SPOSOB JAZDY: obrot w miejscu do kierunku na cel, potem jazda prosto
 * z poprawka kursu. Na trawie obrot w miejscu jest drozszy niz luk, ale
 * daje przewidywalny tor (wazne przy przeszkodach i granicy obszaru)
 * i prosta diagnostyke: kazdy etap ma jeden cel.
 *
 *   TURN   blad kursu > turn_in_deg: stoi i obraca, az spadnie ponizej
 *          turn_out_deg (histereza - inaczej na granicy by szarpal)
 *   DRIVE  jedzie z predkoscia malejaca w slow_radius_mm od celu,
 *          poprawka kursu proporcjonalna (k_heading)
 *   FINAL  opcjonalny obrot do zadanego kursu koncowego
 *
 * ZAKONCZENIE BEZ CELU: brak postepu przez no_progress_s (straznik
 * przeszkod trzyma naped, poslizg, pod gorke) albo przekroczony
 * timeout_s. Pilot sam nie wie dlaczego stoi - mowi tylko, ze nie
 * dojechal, a decyzje podejmuje warstwa wyzej.
 */
#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    FLORA_PILOT_IDLE,
    FLORA_PILOT_TURN,
    FLORA_PILOT_DRIVE,
    FLORA_PILOT_FINAL,
    FLORA_PILOT_DONE,
    FLORA_PILOT_FAILED,
} flora_pilot_state_t;

typedef enum {
    FLORA_PILOT_FAIL_NONE,
    FLORA_PILOT_FAIL_NO_PROGRESS,
    FLORA_PILOT_FAIL_TIMEOUT,
    FLORA_PILOT_FAIL_CANCELLED,
} flora_pilot_fail_t;

typedef struct {
    float track_mm;        /**< rozstaw kol */
    float v_max;           /**< mm/s w jezdzie */
    float v_min;           /**< mm/s tuz przed celem (ponizej naped nie rusza) */
    float w_max_dps;       /**< st/s w obrocie */
    float w_min_dps;       /**< st/s, najwolniejszy obrot w miejscu */
    float k_heading;       /**< 1/s: st/s obrotu na stopien bledu kursu */
    float turn_in_deg;     /**< blad kursu, od ktorego obraca w miejscu */
    float turn_out_deg;    /**< blad kursu, przy ktorym rusza */
    float pos_tol_mm;      /**< dojechal */
    float th_tol_deg;      /**< obrocil */
    float slow_radius_mm;
    float no_progress_s;
    float progress_mm;     /**< tyle zblizenia (albo st obrotu x10) liczy sie jako postep */
    float timeout_s;       /**< 0 = bez limitu */
} flora_pilot_cfg_t;

#define FLORA_PILOT_DEFAULT_CFG()   \
    {                               \
        .track_mm       = 390.0f,   \
        .v_max          = 120.0f,   \
        .v_min          = 40.0f,    \
        .w_max_dps      = 25.0f,    \
        .w_min_dps      = 8.0f,     \
        .k_heading      = 1.5f,     \
        .turn_in_deg    = 25.0f,    \
        .turn_out_deg   = 6.0f,     \
        .pos_tol_mm     = 60.0f,    \
        .th_tol_deg     = 4.0f,     \
        .slow_radius_mm = 300.0f,   \
        .no_progress_s  = 8.0f,     \
        .progress_mm    = 30.0f,    \
        .timeout_s      = 180.0f,   \
    }

typedef struct {
    flora_pilot_state_t state;
    flora_pilot_fail_t  fail;
    float v_l, v_r;        /**< zadane predkosci kol, mm/s */
    float dist_mm;         /**< do celu; 0 przy samym obrocie */
    float err_deg;         /**< blad kursu wzgledem biezacego celu etapu */
} flora_pilot_out_t;

typedef struct {
    flora_pilot_cfg_t cfg;
    flora_pilot_state_t state;
    flora_pilot_fail_t  fail;
    bool  has_point;       /**< cel ma punkt (goto), nie tylko kurs */
    bool  has_final;       /**< cel ma kurs koncowy */
    float gx, gy, gth;
    float t;               /**< czas od startu zadania, s */
    float t_progress;      /**< czas ostatniego postepu */
    float best;            /**< najlepsza dotad miara (odleglosc albo blad kursu) */
    flora_pilot_out_t last;
} flora_pilot_t;

void flora_pilot_init(flora_pilot_t *p, const flora_pilot_cfg_t *cfg);
void flora_pilot_set_cfg(flora_pilot_t *p, const flora_pilot_cfg_t *cfg);

/** Jazda do punktu; kurs koncowy dowolny. */
void flora_pilot_goto(flora_pilot_t *p, float x_mm, float y_mm);

/** Jazda do punktu i obrot do kursu koncowego. */
void flora_pilot_goto_pose(flora_pilot_t *p, float x_mm, float y_mm, float th_deg);

/** Sam obrot w miejscu do kursu (stopnie CCW od osi X mapy). */
void flora_pilot_turn(flora_pilot_t *p, float th_deg);

/** Przerywa zadanie: stan FAILED / CANCELLED. Bez zadania nic nie robi. */
void flora_pilot_cancel(flora_pilot_t *p);

/** true w stanach TURN, DRIVE, FINAL. */
bool flora_pilot_busy(const flora_pilot_t *p);

/**
 * Jeden krok. Poza stanami pracy zwraca zerowe predkosci.
 * @param dt_s czas od poprzedniego kroku
 */
flora_pilot_out_t flora_pilot_update(flora_pilot_t *p, float x_mm, float y_mm,
                                     float th_deg, float dt_s);

#ifdef __cplusplus
}
#endif
