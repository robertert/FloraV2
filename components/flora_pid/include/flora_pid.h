/**
 * @file flora_pid.h
 * @brief Regulator PID z feedforwardem, do petli predkosci kol.
 *
 * Czysty C bez zaleznosci od ESP-IDF, zeby dalo sie go sprawdzic na
 * komputerze (host/test.c) na symulowanym silniku.
 *
 * Wyjscie:
 *   u = ff + P + I + D
 *   ff = kff * zadana + ks * znak(zadana)
 *   P  = kp * e,  e = zadana - pomiar
 *   I  = suma(ki * e * dt), przycieta do +-i_max
 *   D  = -kd * d(pomiar)/dt, przefiltrowane
 *
 * Dlaczego tak:
 *  - Feedforward robi wiekszosc roboty: silnik DC w stanie ustalonym ma
 *    predkosc mniej wiecej proporcjonalna do napiecia, wiec kff*v trafia
 *    blisko celu, a PID poprawia tylko reszte. ks pokrywa tarcie
 *    statyczne, ponizej ktorego kolo w ogole nie rusza.
 *  - D liczone z pomiaru, nie z uchybu: skok zadanej nie daje "kopniecia"
 *    w wyjsciu.
 *  - Calka trzymana juz pomnozona przez ki: zmiana ki w trakcie jazdy
 *    (strojenie przez MQTT) nie powoduje skoku wyjscia.
 *  - Anti-windup warunkowy: gdy wyjscie jest nasycone, a uchyb pcha
 *    dalej w strone nasycenia, calka stoi. Bez tego po utyku albo
 *    podjezdzie calka nabiera zapasu i robot potem przestrzeliwuje.
 */
#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp;        /**< % mocy na mm/s uchybu */
    float ki;        /**< % mocy na (mm/s * s) */
    float kd;        /**< % mocy na mm/s^2 */
    float kff;       /**< % mocy na mm/s zadanej */
    float ks;        /**< % mocy na pokonanie tarcia statycznego */
    float out_min;   /**< dolny limit wyjscia, np. -80 */
    float out_max;   /**< gorny limit wyjscia, np. 80 */
    float i_max;     /**< limit czlonu calkujacego (w % mocy) */
    /** Waga nowej probki w filtrze D, 0..1. 1 = bez filtra. */
    float d_alpha;
    /** Ponizej tej |zadanej| ks nie jest dodawane (mm/s). */
    float ks_deadband;
} flora_pid_cfg_t;

#define FLORA_PID_DEFAULT_CFG()   \
    {                             \
        .kp          = 0.0f,      \
        .ki          = 0.0f,      \
        .kd          = 0.0f,      \
        .kff         = 0.0f,      \
        .ks          = 0.0f,      \
        .out_min     = -80.0f,    \
        .out_max     =  80.0f,    \
        .i_max       =  50.0f,    \
        .d_alpha     =  0.3f,     \
        .ks_deadband =  1.0f,     \
    }

/** Rozbicie ostatniego wyjscia na skladniki - do telemetrii i strojenia. */
typedef struct {
    float ff;
    float p;
    float i;
    float d;
    float out;
} flora_pid_terms_t;

typedef struct {
    flora_pid_cfg_t   cfg;
    float             integ;       /**< czlon I, juz w % mocy */
    float             prev_meas;
    float             d_filt;
    bool              primed;      /**< jest poprzedni pomiar do liczenia D */
    flora_pid_terms_t last;
} flora_pid_t;

void flora_pid_init(flora_pid_t *pid, const flora_pid_cfg_t *cfg);

/** Podmienia nastawy bez zerowania stanu - do strojenia w trakcie jazdy. */
void flora_pid_set_cfg(flora_pid_t *pid, const flora_pid_cfg_t *cfg);

/** Zeruje calke i historie D. Wolaj przy zatrzymaniu i zmianie trybu. */
void flora_pid_reset(flora_pid_t *pid);

/**
 * @brief Jeden krok regulatora.
 *
 * @param setpoint zadana (mm/s)
 * @param measured zmierzona (mm/s)
 * @param dt_s     czas od poprzedniego kroku w sekundach; <= 0 pomija I i D
 * @return wyjscie przyciete do out_min..out_max
 */
float flora_pid_update(flora_pid_t *pid, float setpoint, float measured, float dt_s);

#ifdef __cplusplus
}
#endif
