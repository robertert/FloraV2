#include <inttypes.h>
#include <math.h>
#include <string.h>

#include "flora_pump.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "flora_pump";

#define WINDOW_US  (24LL * 60 * 60 * 1000 * 1000)  /* doba w mikrosekundach */

static struct {
    flora_pump_cfg_t   cfg;
    uint32_t           duty_max;

    SemaphoreHandle_t  lock;
    SemaphoreHandle_t  done_sig;   /**< podnoszony na koniec dozowania */

    esp_timer_handle_t stop_timer;   /**< jednorazowy: twarde zatrzymanie */
    esp_timer_handle_t water_timer;  /**< okresowy: kontrola poziomu w trakcie */

    bool     running;
    int64_t  run_start_us;
    uint32_t run_planned_ms;
    bool     run_clamped;

    int64_t  window_start_us;
    flora_pump_stats_t stats;

    bool initialized;
} s_p;

/* ---------- warstwa wyjsciowa ---------- */

static void output_set(bool on)
{
    if (s_p.cfg.use_pwm) {
        uint32_t duty = on ? s_p.duty_max : 0;
        if (!s_p.cfg.active_high) {
            duty = s_p.duty_max - duty;
        }
        ledc_set_duty(LEDC_LOW_SPEED_MODE, s_p.cfg.channel, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, s_p.cfg.channel);
    } else {
        gpio_set_level(s_p.cfg.pump_gpio, on == s_p.cfg.active_high ? 1 : 0);
    }
}

/* ---------- czujnik poziomu ---------- */

static bool water_ok(void)
{
    if (s_p.cfg.water_cb) {
        return s_p.cfg.water_cb(s_p.cfg.water_ctx);
    }
    if (s_p.cfg.level_gpio == GPIO_NUM_NC) {
        return true;   /* brak czujnika: nie blokujemy, ale i nie chronimy */
    }
    return gpio_get_level(s_p.cfg.level_gpio) == (s_p.cfg.level_water_present ? 1 : 0);
}

bool flora_pump_water_available(void)
{
    return s_p.initialized ? water_ok() : false;
}

/* ---------- koniec dozowania ----------
 * Wolane z zadania timera (koniec czasu albo brak wody) lub z zadania
 * aplikacji (flora_pump_stop). Zaklada, ze blokada NIE jest trzymana.
 */
static void pump_end(flora_pump_done_reason_t reason)
{
    flora_pump_done_cb_t cb = NULL;
    void *ctx = NULL;
    uint32_t ran_ms = 0;
    bool fire = false;

    xSemaphoreTake(s_p.lock, portMAX_DELAY);
    if (s_p.running) {
        output_set(false);
        esp_timer_stop(s_p.stop_timer);
        esp_timer_stop(s_p.water_timer);

        int64_t now = esp_timer_get_time();
        ran_ms = (uint32_t)((now - s_p.run_start_us) / 1000);

        s_p.stats.total_run_ms  += ran_ms;
        s_p.stats.window_run_ms += ran_ms;
        s_p.stats.dose_count++;
        s_p.stats.last_end_us = now;
        if (reason == FLORA_PUMP_DONE_DRY) {
            s_p.stats.aborted_dry++;
        }

        /* Przyciecie do max_run_ms zglaszamy dopiero tutaj, zeby aplikacja
         * poznala rzeczywisty czas pracy, a nie ten, o ktory prosila. */
        if (reason == FLORA_PUMP_DONE_COMPLETED && s_p.run_clamped) {
            reason = FLORA_PUMP_DONE_CLAMPED;
        }

        s_p.running = false;
        cb   = s_p.cfg.done_cb;
        ctx  = s_p.cfg.done_ctx;
        fire = true;

        ESP_LOGI(TAG, "dozowanie zakonczone: %" PRIu32 " ms, powod %d", ran_ms, (int)reason);
    }
    xSemaphoreGive(s_p.lock);

    if (fire) {
        xSemaphoreGive(s_p.done_sig);
        /* Callback poza blokada: moze wolac API komponentu bez zakleszczenia. */
        if (cb) {
            cb(reason, ran_ms, ctx);
        }
    }
}

static void stop_timer_cb(void *arg)
{
    pump_end(FLORA_PUMP_DONE_COMPLETED);
}

static void water_timer_cb(void *arg)
{
    if (!water_ok()) {
        ESP_LOGW(TAG, "brak wody w trakcie dozowania - zatrzymuje pompke");
        pump_end(FLORA_PUMP_DONE_DRY);
    }
}

/* ---------- uruchomienie ---------- */

esp_err_t flora_pump_run_ms(uint32_t ms)
{
    ESP_RETURN_ON_FALSE(s_p.initialized, ESP_ERR_INVALID_STATE, TAG, "nie zainicjalizowano");
    ESP_RETURN_ON_FALSE(ms > 0, ESP_ERR_INVALID_ARG, TAG, "czas musi byc wiekszy od zera");

    xSemaphoreTake(s_p.lock, portMAX_DELAY);
    esp_err_t ret = ESP_OK;
    int64_t now = esp_timer_get_time();

    if (s_p.running) {
        ESP_LOGW(TAG, "pompka juz pracuje");
        ret = ESP_ERR_INVALID_STATE;
        goto out;
    }

    /* Okno dobowe jest ruchome i liczone od startu ukladu, nie od polnocy -
     * bez zegara RTC nie ma sensownego pojecia "doby kalendarzowej". */
    if (now - s_p.window_start_us > WINDOW_US) {
        s_p.window_start_us = now;
        s_p.stats.window_run_ms = 0;
        ESP_LOGI(TAG, "nowe okno dobowe, limit odnowiony");
    }

    if (s_p.cfg.min_interval_ms > 0 && s_p.stats.last_end_us > 0) {
        int64_t since_ms = (now - s_p.stats.last_end_us) / 1000;
        if (since_ms < (int64_t)s_p.cfg.min_interval_ms) {
            ESP_LOGW(TAG, "za wczesnie: minelo %" PRId64 " ms z wymaganych %" PRIu32,
                     since_ms, s_p.cfg.min_interval_ms);
            s_p.stats.refused_interval++;
            ret = ESP_ERR_INVALID_STATE;
            goto out;
        }
    }

    if (s_p.cfg.daily_limit_ms > 0 &&
        s_p.stats.window_run_ms + ms > s_p.cfg.daily_limit_ms) {
        ESP_LOGW(TAG, "limit dobowy wyczerpany: %" PRIu32 " z %" PRIu32 " ms",
                 s_p.stats.window_run_ms, s_p.cfg.daily_limit_ms);
        s_p.stats.refused_limit++;
        ret = ESP_ERR_NO_MEM;
        goto out;
    }

    if (!water_ok()) {
        ESP_LOGW(TAG, "czujnik zglasza brak wody - nie uruchamiam");
        s_p.stats.refused_dry++;
        ret = ESP_ERR_NOT_FOUND;
        goto out;
    }

    s_p.run_clamped = false;
    if (s_p.cfg.max_run_ms > 0 && ms > s_p.cfg.max_run_ms) {
        ESP_LOGW(TAG, "zadano %" PRIu32 " ms, przycinam do limitu %" PRIu32 " ms",
                 ms, s_p.cfg.max_run_ms);
        ms = s_p.cfg.max_run_ms;
        s_p.run_clamped = true;
    }

    /* Timer odcinajacy uzbrajamy PRZED zalaczeniem pompki. Gdyby start
     * timera sie nie udal, pompka w ogole nie ruszy - zamiast ruszyc
     * i zostac bez nadzoru. */
    ret = esp_timer_start_once(s_p.stop_timer, (uint64_t)ms * 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nie udalo sie uzbroic timera odcinajacego: %s", esp_err_to_name(ret));
        goto out;
    }

    bool guard = (s_p.cfg.water_cb || s_p.cfg.level_gpio != GPIO_NUM_NC)
                 && s_p.cfg.water_check_ms > 0;
    if (guard) {
        esp_timer_start_periodic(s_p.water_timer, (uint64_t)s_p.cfg.water_check_ms * 1000);
    }

    xSemaphoreTake(s_p.done_sig, 0);   /* kasujemy sygnal po poprzednim przebiegu */
    s_p.run_start_us   = esp_timer_get_time();
    s_p.run_planned_ms = ms;
    s_p.running        = true;
    output_set(true);

    ESP_LOGI(TAG, "dozowanie %" PRIu32 " ms%s", ms, guard ? " (kontrola poziomu aktywna)" : "");

out:
    xSemaphoreGive(s_p.lock);
    return ret;
}

esp_err_t flora_pump_run_ms_blocking(uint32_t ms, uint32_t timeout_ms)
{
    ESP_RETURN_ON_ERROR(flora_pump_run_ms(ms), TAG, "start dozowania");
    if (xSemaphoreTake(s_p.done_sig, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        ESP_LOGE(TAG, "dozowanie nie zglosilo konca w %" PRIu32 " ms - zatrzymuje awaryjnie",
                 timeout_ms);
        flora_pump_stop();
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

esp_err_t flora_pump_dose_ml(float ml)
{
    ESP_RETURN_ON_FALSE(s_p.initialized, ESP_ERR_INVALID_STATE, TAG, "nie zainicjalizowano");
    ESP_RETURN_ON_FALSE(s_p.cfg.ml_per_second > 0.0f, ESP_ERR_INVALID_STATE, TAG,
                        "brak kalibracji ml_per_second - dozowanie objetosciowe niedostepne");
    ESP_RETURN_ON_FALSE(ml > 0.0f, ESP_ERR_INVALID_ARG, TAG, "objetosc musi byc dodatnia");

    uint32_t ms = (uint32_t)lroundf((ml / s_p.cfg.ml_per_second) * 1000.0f);
    ESP_LOGI(TAG, "dawka %.1f ml -> %" PRIu32 " ms przy %.2f ml/s",
             ml, ms, s_p.cfg.ml_per_second);
    return flora_pump_run_ms(ms);
}

esp_err_t flora_pump_stop(void)
{
    ESP_RETURN_ON_FALSE(s_p.initialized, ESP_ERR_INVALID_STATE, TAG, "nie zainicjalizowano");
    pump_end(FLORA_PUMP_DONE_STOPPED);
    return ESP_OK;
}

bool flora_pump_is_running(void)
{
    return s_p.initialized && s_p.running;
}

uint32_t flora_pump_remaining_ms(void)
{
    if (!s_p.initialized || !s_p.running) {
        return 0;
    }
    int64_t elapsed_ms = (esp_timer_get_time() - s_p.run_start_us) / 1000;
    int64_t left = (int64_t)s_p.run_planned_ms - elapsed_ms;
    return left > 0 ? (uint32_t)left : 0;
}

void flora_pump_get_stats(flora_pump_stats_t *out)
{
    if (!out || !s_p.initialized) {
        return;
    }
    xSemaphoreTake(s_p.lock, portMAX_DELAY);
    *out = s_p.stats;
    xSemaphoreGive(s_p.lock);
}

void flora_pump_reset_stats(void)
{
    if (!s_p.initialized) {
        return;
    }
    xSemaphoreTake(s_p.lock, portMAX_DELAY);
    memset(&s_p.stats, 0, sizeof(s_p.stats));
    s_p.window_start_us = esp_timer_get_time();
    xSemaphoreGive(s_p.lock);
}

/* ---------- inicjalizacja ---------- */

static esp_err_t output_init(const flora_pump_cfg_t *cfg)
{
    if (cfg->use_pwm) {
        uint32_t steps = 1UL << (uint32_t)cfg->pwm_resolution;
        ESP_RETURN_ON_FALSE((uint64_t)cfg->pwm_freq_hz * steps <= 80000000ULL,
                            ESP_ERR_INVALID_ARG, TAG,
                            "PWM %" PRIu32 " Hz przy %" PRIu32 " krokach nie miesci sie w 80 MHz APB",
                            cfg->pwm_freq_hz, steps);
        s_p.duty_max = steps - 1;

        ledc_timer_config_t t = {
            .speed_mode      = LEDC_LOW_SPEED_MODE,
            .duty_resolution = cfg->pwm_resolution,
            .timer_num       = cfg->timer,
            .freq_hz         = cfg->pwm_freq_hz,
            .clk_cfg         = LEDC_AUTO_CLK,
        };
        ESP_RETURN_ON_ERROR(ledc_timer_config(&t), TAG, "ledc_timer_config");

        ledc_channel_config_t c = {
            .gpio_num   = cfg->pump_gpio,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel    = cfg->channel,
            .timer_sel  = cfg->timer,
            .duty       = cfg->active_high ? 0 : s_p.duty_max,  /* start: pompka wylaczona */
            .hpoint     = 0,
        };
        ESP_RETURN_ON_ERROR(ledc_channel_config(&c), TAG, "ledc_channel_config");
        return ESP_OK;
    }

    gpio_config_t io = {
        .pin_bit_mask = 1ULL << cfg->pump_gpio,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,  /* spojne z rezystorem 10k na plytce */
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io), TAG, "gpio_config(pompka)");
    return gpio_set_level(cfg->pump_gpio, cfg->active_high ? 0 : 1);
}

esp_err_t flora_pump_init(const flora_pump_cfg_t *cfg)
{
    ESP_RETURN_ON_FALSE(cfg, ESP_ERR_INVALID_ARG, TAG, "cfg == NULL");
    ESP_RETURN_ON_FALSE(!s_p.initialized, ESP_ERR_INVALID_STATE, TAG, "juz zainicjalizowano");

    if (cfg->max_run_ms == 0) {
        ESP_LOGW(TAG, "max_run_ms = 0: pompka nie ma twardego limitu czasu pracy. "
                      "Przy zawieszeniu aplikacji nic jej nie wylaczy.");
    }

    memset(&s_p, 0, sizeof(s_p));
    s_p.cfg = *cfg;

    esp_err_t ret = ESP_OK;
    s_p.lock = xSemaphoreCreateMutex();
    ESP_RETURN_ON_FALSE(s_p.lock, ESP_ERR_NO_MEM, TAG, "brak pamieci na mutex");
    s_p.done_sig = xSemaphoreCreateBinary();
    ESP_GOTO_ON_FALSE(s_p.done_sig, ESP_ERR_NO_MEM, fail, TAG, "brak pamieci na semafor");

    ESP_GOTO_ON_ERROR(output_init(cfg), fail, TAG, "konfiguracja wyjscia");

    if (cfg->level_gpio != GPIO_NUM_NC) {
        gpio_config_t lvl = {
            .pin_bit_mask = 1ULL << cfg->level_gpio,
            .mode         = GPIO_MODE_INPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,   /* XKC-Y25 ma wlasne podciagniecie */
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        ESP_GOTO_ON_ERROR(gpio_config(&lvl), fail, TAG, "konfiguracja czujnika poziomu");
    }

    const esp_timer_create_args_t stop_args = {
        .callback = stop_timer_cb,
        .name     = "pump_stop",
    };
    ESP_GOTO_ON_ERROR(esp_timer_create(&stop_args, &s_p.stop_timer), fail, TAG, "timer odcinajacy");

    const esp_timer_create_args_t water_args = {
        .callback = water_timer_cb,
        .name     = "pump_water",
    };
    ESP_GOTO_ON_ERROR(esp_timer_create(&water_args, &s_p.water_timer), fail, TAG, "timer poziomu");

    s_p.window_start_us = esp_timer_get_time();
    s_p.initialized = true;

    ESP_LOGI(TAG, "pompka na GPIO%d (%s, %s), limity: %" PRIu32 " ms/dawke, "
                  "%" PRIu32 " ms przerwy, %" PRIu32 " ms/dobe",
             cfg->pump_gpio,
             cfg->active_high ? "aktywny wysoki" : "aktywny niski",
             cfg->use_pwm ? "PWM" : "wlacz/wylacz",
             cfg->max_run_ms, cfg->min_interval_ms, cfg->daily_limit_ms);
    if (cfg->level_gpio != GPIO_NUM_NC) {
        ESP_LOGI(TAG, "czujnik poziomu na GPIO%d, woda = stan %s, kontrola co %" PRIu32 " ms",
                 cfg->level_gpio, cfg->level_water_present ? "wysoki" : "niski",
                 cfg->water_check_ms);
    } else if (!cfg->water_cb) {
        ESP_LOGW(TAG, "brak czujnika poziomu - pompka moze pracowac na sucho");
    }
    return ESP_OK;

fail:
    if (s_p.stop_timer)  { esp_timer_delete(s_p.stop_timer);  s_p.stop_timer = NULL; }
    if (s_p.water_timer) { esp_timer_delete(s_p.water_timer); s_p.water_timer = NULL; }
    if (s_p.done_sig)    { vSemaphoreDelete(s_p.done_sig);    s_p.done_sig = NULL; }
    if (s_p.lock)        { vSemaphoreDelete(s_p.lock);        s_p.lock = NULL; }
    return ret;
}

esp_err_t flora_pump_deinit(void)
{
    ESP_RETURN_ON_FALSE(s_p.initialized, ESP_ERR_INVALID_STATE, TAG, "nie zainicjalizowano");

    pump_end(FLORA_PUMP_DONE_STOPPED);
    output_set(false);   /* jeszcze raz, na wypadek gdyby nie pracowala */

    esp_timer_delete(s_p.stop_timer);
    esp_timer_delete(s_p.water_timer);
    vSemaphoreDelete(s_p.done_sig);
    vSemaphoreDelete(s_p.lock);

    s_p.initialized = false;
    return ESP_OK;
}
