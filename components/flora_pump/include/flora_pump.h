/**
 * @file flora_pump.h
 * @brief Pompka perystaltyczna na module D4184 (klucz low-side).
 *
 * Komponent dozuje wode czasowo albo objetosciowo i pilnuje, zeby pompka
 * nie mogla chodzic w nieskonczonosc ani na sucho.
 *
 * ZALOZENIE BEZPIECZENSTWA: kazde dozowanie jest ucinane przez sprzetowy
 * timer (esp_timer), a nie przez petle w zadaniu aplikacji. Jesli zadanie
 * wywolujace sie zawiesi, pompka i tak zostanie wylaczona. To jest glowny
 * powod istnienia tego komponentu - samo szarpanie GPIO to jedna linijka.
 *
 * Uwaga sprzetowa: GPIO pompki musi miec rezystor 10k do masy, zeby przy
 * resecie i w czasie bootowania pompka nie zalaczyla sie sama. Stan pinu
 * przed uruchomieniem aplikacji nie jest kontrolowany przez ten kod.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Powod zakonczenia dozowania. */
typedef enum {
    FLORA_PUMP_DONE_COMPLETED,  /**< wybrany czas uplynal normalnie */
    FLORA_PUMP_DONE_STOPPED,    /**< przerwane przez flora_pump_stop() */
    FLORA_PUMP_DONE_DRY,        /**< czujnik zglosil brak wody w trakcie */
    FLORA_PUMP_DONE_CLAMPED,    /**< uciete twardym limitem max_run_ms */
} flora_pump_done_reason_t;

/** Wolane po zakonczeniu dozowania, z kontekstu zadania timera. */
typedef void (*flora_pump_done_cb_t)(flora_pump_done_reason_t reason,
                                     uint32_t ran_ms, void *ctx);

/** Zwraca true, gdy w zbiorniku jest woda. Zastepuje kontrole po GPIO. */
typedef bool (*flora_pump_water_cb_t)(void *ctx);

typedef struct {
    gpio_num_t pump_gpio;
    bool       active_high;     /**< D4184 zalacza stanem wysokim */

    /* --- Sterowanie wydajnoscia (opcjonalne) ---
     * Bez PWM pompka chodzi na pelnej mocy, a dawke okresla czas. Dla
     * perystaltyki to zwykle wlasciwy wybor: przeplyw jest proporcjonalny
     * do obrotow, a przy malym wypelnieniu pompka potrafi stanac. */
    bool             use_pwm;
    uint32_t         pwm_freq_hz;     /**< 1 kHz to bezpieczny domyslny wybor dla MOSFET-a z obciazeniem silnikowym */
    ledc_timer_bit_t pwm_resolution;
    ledc_timer_t     timer;           /**< MUSI byc inny niz timer napedu */
    ledc_channel_t   channel;         /**< MUSI byc inny niz kanaly napedu */

    /* --- Limity bezpieczenstwa --- */
    uint32_t max_run_ms;        /**< twardy sufit jednego dozowania; 0 = brak (odradzane) */
    uint32_t min_interval_ms;   /**< wymagana przerwa od konca poprzedniego dozowania */
    uint32_t daily_limit_ms;    /**< sumaryczny limit w oknie 24 h; 0 = brak */

    /* --- Ochrona przed praca na sucho --- */
    gpio_num_t level_gpio;          /**< czujnik poziomu; -1 wylacza kontrole po GPIO */
    bool       level_water_present; /**< stan pinu oznaczajacy "woda jest" */
    flora_pump_water_cb_t water_cb; /**< ma pierwszenstwo przed level_gpio */
    void      *water_ctx;
    uint32_t   water_check_ms;      /**< co ile sprawdzac poziom w trakcie dozowania */

    /* --- Kalibracja objetosci --- */
    float ml_per_second;        /**< wydajnosc przy pelnej mocy; 0 blokuje dozowanie objetosciowe */

    flora_pump_done_cb_t done_cb;
    void                *done_ctx;
} flora_pump_cfg_t;

/**
 * Wartosci domyslne nastawione zachowawczo: 30 s na dawke, minuta przerwy,
 * 10 minut na dobe. Przy zbiorniku 2-3 l to bezpieczny punkt wyjscia,
 * ktory warto dopiero poluzowac po kalibracji.
 */
#define FLORA_PUMP_DEFAULT_CFG()                 \
    {                                            \
        .active_high        = true,              \
        .use_pwm            = false,             \
        .pwm_freq_hz        = 1000,              \
        .pwm_resolution     = LEDC_TIMER_10_BIT, \
        .timer              = LEDC_TIMER_1,      \
        .channel            = LEDC_CHANNEL_2,    \
        .max_run_ms         = 30000,             \
        .min_interval_ms    = 60000,             \
        .daily_limit_ms     = 600000,            \
        .level_gpio         = GPIO_NUM_NC,       \
        .level_water_present = true,             \
        .water_check_ms     = 500,               \
        .ml_per_second      = 0.0f,              \
    }

typedef struct {
    uint32_t total_run_ms;     /**< od startu ukladu */
    uint32_t window_run_ms;    /**< w biezacym oknie 24 h */
    uint32_t dose_count;
    uint32_t aborted_dry;      /**< przerwane z powodu braku wody */
    uint32_t refused_interval; /**< odmowy: za wczesnie po poprzednim */
    uint32_t refused_limit;    /**< odmowy: wyczerpany limit dobowy */
    uint32_t refused_dry;      /**< odmowy: brak wody juz przed startem */
    int64_t  last_end_us;
} flora_pump_stats_t;

esp_err_t flora_pump_init(const flora_pump_cfg_t *cfg);
esp_err_t flora_pump_deinit(void);

/**
 * @brief Uruchamia dozowanie na zadany czas. Nie blokuje.
 *
 * Czas dluzszy niz max_run_ms zostaje przyciety, a w logu pojawia sie
 * ostrzezenie; callback dostanie wtedy FLORA_PUMP_DONE_CLAMPED i faktyczny
 * czas pracy. Przyciecie jest swiadome: lepiej podlac za malo niz pozwolic
 * pompce chodzic dluzej niz zaklada limit bezpieczenstwa.
 *
 * @return ESP_ERR_INVALID_STATE gdy juz pracuje albo za wczesnie po poprzednim
 *         dozowaniu, ESP_ERR_NOT_FOUND gdy brak wody, ESP_ERR_NO_MEM gdy
 *         wyczerpany limit dobowy.
 */
esp_err_t flora_pump_run_ms(uint32_t ms);

/** Jak wyzej, ale czeka do konca dozowania. */
esp_err_t flora_pump_run_ms_blocking(uint32_t ms, uint32_t timeout_ms);

/** Dozowanie objetosciowe. Wymaga ml_per_second w konfiguracji. */
esp_err_t flora_pump_dose_ml(float ml);

/** Natychmiast zatrzymuje pompke. Bezpieczne do wolania, gdy nie pracuje. */
esp_err_t flora_pump_stop(void);

bool flora_pump_is_running(void);

/** Ile ms zostalo do konca biezacego dozowania; 0 gdy pompka stoi. */
uint32_t flora_pump_remaining_ms(void);

/** Sprawdza czujnik poziomu teraz. Bez skonfigurowanego czujnika zwraca true. */
bool flora_pump_water_available(void);

void flora_pump_get_stats(flora_pump_stats_t *out);

/** Zeruje liczniki i okno dobowe. Nie dotyka trwajacego dozowania. */
void flora_pump_reset_stats(void);

#ifdef __cplusplus
}
#endif
