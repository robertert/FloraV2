#include <inttypes.h>
#include <stdlib.h>
#include <string.h>

#include "flora_pwrsw.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "flora_pwrsw";

struct flora_pwrsw_s {
    flora_pwrsw_cfg_t   cfg;
    flora_pwrsw_stats_t stats;
    int64_t             off_since_us;   /**< 0 gdy nigdy nie bylo zalaczone */
    int64_t             on_since_us;
};

static void drive(flora_pwrsw_handle_t h, bool on)
{
    gpio_set_level(h->cfg.gpio, (on == h->cfg.active_high) ? 1 : 0);
}

esp_err_t flora_pwrsw_init(const flora_pwrsw_cfg_t *cfg, flora_pwrsw_handle_t *out)
{
    ESP_RETURN_ON_FALSE(cfg && out, ESP_ERR_INVALID_ARG, TAG, "zly argument");

    flora_pwrsw_handle_t h = calloc(1, sizeof(struct flora_pwrsw_s));
    ESP_RETURN_ON_FALSE(h, ESP_ERR_NO_MEM, TAG, "brak pamieci");
    h->cfg = *cfg;

    gpio_config_t io = {
        .pin_bit_mask = 1ULL << cfg->gpio,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        /* Spojne z rezystorem 10k na plytce: przy resecie i w czasie
         * bootowania sekcja ma byc wylaczona, a nie zalaczac sie sama. */
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    esp_err_t ret = gpio_config(&io);
    if (ret != ESP_OK) {
        free(h);
        ESP_RETURN_ON_ERROR(ret, TAG, "gpio_config(GPIO%d)", cfg->gpio);
    }

    drive(h, false);
    h->stats.is_on          = false;
    h->stats.last_change_us = esp_timer_get_time();
    /* Zero znaczy "nigdy nie bylo zalaczone" - pierwsze zalaczenie nie musi
     * czekac na przerwe, ktorej nie bylo po czym odliczac. */
    h->off_since_us = 0;

    ESP_LOGI(TAG, "klucz na GPIO%d (%s), przerwa %" PRIu32 " ms, rozruch %" PRIu32 " ms, "
                  "stan poczatkowy: wylaczone",
             cfg->gpio, cfg->active_high ? "aktywny wysoki" : "aktywny niski",
             cfg->min_off_ms, cfg->boot_ms);
    *out = h;
    return ESP_OK;
}

esp_err_t flora_pwrsw_deinit(flora_pwrsw_handle_t h)
{
    ESP_RETURN_ON_FALSE(h, ESP_ERR_INVALID_ARG, TAG, "h == NULL");
    flora_pwrsw_off(h);
    free(h);
    return ESP_OK;
}

uint32_t flora_pwrsw_remaining_off_ms(flora_pwrsw_handle_t h)
{
    if (!h || h->stats.is_on || h->off_since_us == 0) {
        return 0;
    }
    int64_t elapsed_ms = (esp_timer_get_time() - h->off_since_us) / 1000;
    int64_t left = (int64_t)h->cfg.min_off_ms - elapsed_ms;
    return left > 0 ? (uint32_t)left : 0;
}

esp_err_t flora_pwrsw_on(flora_pwrsw_handle_t h)
{
    ESP_RETURN_ON_FALSE(h, ESP_ERR_INVALID_ARG, TAG, "h == NULL");
    if (h->stats.is_on) {
        return ESP_OK;
    }

    uint32_t left = flora_pwrsw_remaining_off_ms(h);
    ESP_RETURN_ON_FALSE(left == 0, ESP_ERR_INVALID_STATE, TAG,
                        "jeszcze %" PRIu32 " ms wymuszonej przerwy", left);

    drive(h, true);
    h->on_since_us          = esp_timer_get_time();
    h->stats.last_change_us = h->on_since_us;
    h->stats.is_on          = true;
    h->stats.on_count++;

    ESP_LOGI(TAG, "zalaczone (%" PRIu32 ". raz)", h->stats.on_count);
    return ESP_OK;
}

esp_err_t flora_pwrsw_on_wait(flora_pwrsw_handle_t h)
{
    ESP_RETURN_ON_FALSE(h, ESP_ERR_INVALID_ARG, TAG, "h == NULL");
    uint32_t left = flora_pwrsw_remaining_off_ms(h);
    if (left > 0) {
        ESP_LOGI(TAG, "czekam %" PRIu32 " ms na koniec wymuszonej przerwy", left);
        vTaskDelay(pdMS_TO_TICKS(left + 10));
    }
    return flora_pwrsw_on(h);
}

esp_err_t flora_pwrsw_off(flora_pwrsw_handle_t h)
{
    ESP_RETURN_ON_FALSE(h, ESP_ERR_INVALID_ARG, TAG, "h == NULL");
    if (!h->stats.is_on) {
        return ESP_OK;
    }

    drive(h, false);
    int64_t now = esp_timer_get_time();
    h->stats.total_on_ms   += (uint64_t)((now - h->on_since_us) / 1000);
    h->stats.last_change_us = now;
    h->stats.is_on          = false;
    h->off_since_us         = now;

    ESP_LOGI(TAG, "wylaczone (lacznie %" PRIu64 " ms pracy)", h->stats.total_on_ms);
    return ESP_OK;
}

bool flora_pwrsw_is_on(flora_pwrsw_handle_t h)
{
    return h && h->stats.is_on;
}

esp_err_t flora_pwrsw_wait_boot(flora_pwrsw_handle_t h)
{
    ESP_RETURN_ON_FALSE(h, ESP_ERR_INVALID_ARG, TAG, "h == NULL");
    ESP_RETURN_ON_FALSE(h->stats.is_on, ESP_ERR_INVALID_STATE, TAG, "sekcja wylaczona");

    int64_t elapsed_ms = (esp_timer_get_time() - h->on_since_us) / 1000;
    int64_t left = (int64_t)h->cfg.boot_ms - elapsed_ms;
    if (left > 0) {
        vTaskDelay(pdMS_TO_TICKS(left));
    }
    return ESP_OK;
}

void flora_pwrsw_get_stats(flora_pwrsw_handle_t h, flora_pwrsw_stats_t *out)
{
    if (!h || !out) {
        return;
    }
    *out = h->stats;
    /* Biezacy, jeszcze nie zamkniety okres tez sie liczy - inaczej odczyt
     * w trakcie pracy pokazywalby stan sprzed ostatniego zalaczenia. */
    if (h->stats.is_on) {
        out->total_on_ms += (uint64_t)((esp_timer_get_time() - h->on_since_us) / 1000);
    }
}
