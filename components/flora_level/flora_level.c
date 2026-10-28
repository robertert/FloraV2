#include <inttypes.h>
#include <stdlib.h>
#include <string.h>

#include "flora_level.h"

#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "flora_level";

struct flora_level_s {
    flora_level_cfg_t  cfg;
    esp_timer_handle_t timer;

    bool     stable;        /**< stan po uspokojeniu */
    bool     candidate;     /**< stan, ktory probuje sie ustabilizowac */
    uint8_t  agree_count;   /**< ile zgodnych odczytow z rzedu */
    uint32_t changes;
    bool     primed;
};

static bool read_raw(const struct flora_level_s *h)
{
    return gpio_get_level(h->cfg.gpio) == (h->cfg.water_present_level ? 1 : 0);
}

static void sample_cb(void *arg)
{
    flora_level_handle_t h = (flora_level_handle_t)arg;
    bool now = read_raw(h);

    if (now != h->candidate) {
        /* Odczyt rozni sie od kandydata - zaczynamy liczyc od nowa. */
        h->candidate   = now;
        h->agree_count = 1;
        return;
    }
    if (h->agree_count < h->cfg.stable_samples) {
        h->agree_count++;
    }
    if (h->agree_count < h->cfg.stable_samples || h->stable == h->candidate) {
        return;
    }

    h->stable = h->candidate;
    h->changes++;
    ESP_LOGI(TAG, "poziom: %s", h->stable ? "woda jest" : "brak wody");
    if (h->cfg.on_change) {
        h->cfg.on_change(h->stable, h->cfg.ctx);
    }
}

esp_err_t flora_level_init(const flora_level_cfg_t *cfg, flora_level_handle_t *out)
{
    ESP_RETURN_ON_FALSE(cfg && out, ESP_ERR_INVALID_ARG, TAG, "zly argument");
    ESP_RETURN_ON_FALSE(cfg->sample_ms > 0 && cfg->stable_samples > 0,
                        ESP_ERR_INVALID_ARG, TAG, "sample_ms i stable_samples musza byc > 0");

    flora_level_handle_t h = calloc(1, sizeof(struct flora_level_s));
    ESP_RETURN_ON_FALSE(h, ESP_ERR_NO_MEM, TAG, "brak pamieci");
    h->cfg = *cfg;

    gpio_config_t io = {
        .pin_bit_mask = 1ULL << cfg->gpio,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = cfg->enable_pullup ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    esp_err_t ret = gpio_config(&io);
    if (ret != ESP_OK) {
        free(h);
        ESP_RETURN_ON_ERROR(ret, TAG, "gpio_config(GPIO%d)", cfg->gpio);
    }

    /* Stan poczatkowy bierzemy z pierwszego odczytu, zeby nie zglaszac
     * fikcyjnej zmiany zaraz po starcie. */
    h->stable      = read_raw(h);
    h->candidate   = h->stable;
    h->agree_count = cfg->stable_samples;
    h->primed      = true;

    const esp_timer_create_args_t args = { .callback = sample_cb, .arg = h, .name = "level" };
    ret = esp_timer_create(&args, &h->timer);
    if (ret != ESP_OK) {
        free(h);
        ESP_RETURN_ON_ERROR(ret, TAG, "esp_timer_create");
    }
    ret = esp_timer_start_periodic(h->timer, (uint64_t)cfg->sample_ms * 1000);
    if (ret != ESP_OK) {
        esp_timer_delete(h->timer);
        free(h);
        ESP_RETURN_ON_ERROR(ret, TAG, "esp_timer_start_periodic");
    }

    ESP_LOGI(TAG, "GPIO%d, woda = stan %s, uspokajanie %" PRIu32 " ms (%u x %" PRIu32 " ms), "
                  "stan poczatkowy: %s",
             cfg->gpio, cfg->water_present_level ? "wysoki" : "niski",
             cfg->sample_ms * cfg->stable_samples, cfg->stable_samples, cfg->sample_ms,
             h->stable ? "woda jest" : "brak wody");
    *out = h;
    return ESP_OK;
}

esp_err_t flora_level_deinit(flora_level_handle_t h)
{
    ESP_RETURN_ON_FALSE(h, ESP_ERR_INVALID_ARG, TAG, "h == NULL");
    esp_timer_stop(h->timer);
    esp_timer_delete(h->timer);
    free(h);
    return ESP_OK;
}

bool flora_level_water_present(flora_level_handle_t h)
{
    return h ? h->stable : false;
}

bool flora_level_raw(flora_level_handle_t h)
{
    return h ? read_raw(h) : false;
}

uint32_t flora_level_change_count(flora_level_handle_t h)
{
    return h ? h->changes : 0;
}

bool flora_level_pump_cb(void *handle_as_ctx)
{
    return flora_level_water_present((flora_level_handle_t)handle_as_ctx);
}
