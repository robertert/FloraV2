#include <inttypes.h>
#include <string.h>

#include "flora_temp.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"

#include "onewire_bus.h"
#include "ds18b20.h"

static const char *TAG = "flora_temp";

static struct {
    onewire_bus_handle_t    bus;
    ds18b20_device_handle_t dev[FLORA_TEMP_MAX_SENSORS];
    uint64_t                addr[FLORA_TEMP_MAX_SENSORS];
    int                     count;
    uint32_t                conv_ms;
    bool                    initialized;
} s_t;

/** Mapuje nasza rozdzielczosc na typ sterownika i czas konwersji. */
static void resolution_map(flora_temp_resolution_t in,
                           ds18b20_resolution_t *out, uint32_t *conv_ms)
{
    switch (in) {
        case FLORA_TEMP_RES_9BIT:  *out = DS18B20_RESOLUTION_9B;  *conv_ms = 110;  break;
        case FLORA_TEMP_RES_10BIT: *out = DS18B20_RESOLUTION_10B; *conv_ms = 210;  break;
        case FLORA_TEMP_RES_11BIT: *out = DS18B20_RESOLUTION_11B; *conv_ms = 410;  break;
        default:                   *out = DS18B20_RESOLUTION_12B; *conv_ms = 800;  break;
    }
    /* Czasy z zapasem ~10% ponad nominal z noty katalogowej - uklad przy
     * cieplejszym zasilaniu konwertuje odrobine wolniej. */
}

esp_err_t flora_temp_init(const flora_temp_cfg_t *cfg)
{
    ESP_RETURN_ON_FALSE(cfg, ESP_ERR_INVALID_ARG, TAG, "cfg == NULL");
    ESP_RETURN_ON_FALSE(!s_t.initialized, ESP_ERR_INVALID_STATE, TAG, "juz zainicjalizowano");

    memset(&s_t, 0, sizeof(s_t));

    ds18b20_resolution_t res;
    resolution_map(cfg->resolution, &res, &s_t.conv_ms);

    onewire_bus_config_t bus_cfg = {
        .bus_gpio_num     = cfg->gpio,
        .flags.en_pull_up = cfg->enable_internal_pullup,
    };
    onewire_bus_rmt_config_t rmt_cfg = {
        /* 1 bajt komendy ROM + 8 bajtow adresu + 1 bajt komendy urzadzenia */
        .max_rx_bytes = 10,
    };

    esp_err_t ret = ESP_OK;
    ESP_RETURN_ON_ERROR(onewire_new_bus_rmt(&bus_cfg, &rmt_cfg, &s_t.bus),
                        TAG, "magistrala 1-Wire na GPIO%d", cfg->gpio);

    onewire_device_iter_handle_t iter = NULL;
    ESP_GOTO_ON_ERROR(onewire_new_device_iter(s_t.bus, &iter), fail, TAG, "iterator urzadzen");

    onewire_device_t found;
    int others = 0;
    esp_err_t search;
    do {
        search = onewire_device_iter_get_next(iter, &found);
        if (search == ESP_ERR_NOT_FOUND) {
            break;
        }
        if (search != ESP_OK) {
            /* Kazdy inny blad konczy skan. Bez tego pusta magistrala (linia
             * bez podciagniecia, brak czujnika) dawala nieskonczona petle
             * nieudanych resetow - kanal RMT jest po pierwszym juz wylaczony,
             * wiec ponawianie nie ma szans - i watchdog restartowal uklad. */
            ESP_LOGW(TAG, "skan 1-Wire przerwany: %s", esp_err_to_name(search));
            break;
        }
        if (s_t.count >= FLORA_TEMP_MAX_SENSORS) {
            ESP_LOGW(TAG, "wiecej niz %d czujnikow na magistrali, reszte pomijam",
                     FLORA_TEMP_MAX_SENSORS);
            break;
        }
        ds18b20_config_t dev_cfg = {};
        if (ds18b20_new_device_from_enumeration(&found, &dev_cfg, &s_t.dev[s_t.count]) == ESP_OK) {
            s_t.addr[s_t.count] = found.address;
            ESP_LOGI(TAG, "czujnik [%d]: %016llX", s_t.count, (unsigned long long)found.address);
            s_t.count++;
        } else {
            /* Na magistrali moga wisiec inne uklady 1-Wire - to nie blad. */
            others++;
            ESP_LOGD(TAG, "urzadzenie 1-Wire, ale nie DS18B20: %016llX",
                     (unsigned long long)found.address);
        }
    } while (search != ESP_ERR_NOT_FOUND);
    onewire_del_device_iter(iter);

    ESP_GOTO_ON_FALSE(s_t.count > 0, ESP_ERR_NOT_FOUND, fail, TAG,
                      "brak DS18B20 na GPIO%d (znaleziono %d innych urzadzen) - "
                      "sprawdz rezystor 4,7k do 3,3V, zasilanie i przewod danych",
                      cfg->gpio, others);

    for (int i = 0; i < s_t.count; i++) {
        ESP_GOTO_ON_ERROR(ds18b20_set_resolution(s_t.dev[i], res), fail, TAG,
                          "ustawienie rozdzielczosci czujnika %d", i);
    }

    s_t.initialized = true;
    ESP_LOGI(TAG, "gotowe: %d czujnik(ow) na GPIO%d, konwersja %" PRIu32 " ms",
             s_t.count, cfg->gpio, s_t.conv_ms);
    return ESP_OK;

fail:
    for (int i = 0; i < s_t.count; i++) {
        ds18b20_del_device(s_t.dev[i]);
    }
    onewire_bus_del(s_t.bus);
    memset(&s_t, 0, sizeof(s_t));
    return ret;
}

esp_err_t flora_temp_deinit(void)
{
    ESP_RETURN_ON_FALSE(s_t.initialized, ESP_ERR_INVALID_STATE, TAG, "nie zainicjalizowano");
    for (int i = 0; i < s_t.count; i++) {
        ds18b20_del_device(s_t.dev[i]);
    }
    onewire_bus_del(s_t.bus);
    memset(&s_t, 0, sizeof(s_t));
    return ESP_OK;
}

int flora_temp_count(void)
{
    return s_t.initialized ? s_t.count : 0;
}

uint64_t flora_temp_address(int index)
{
    if (!s_t.initialized || index < 0 || index >= s_t.count) {
        return 0;
    }
    return s_t.addr[index];
}

int flora_temp_index_of(uint64_t address)
{
    if (!s_t.initialized) {
        return -1;
    }
    for (int i = 0; i < s_t.count; i++) {
        if (s_t.addr[i] == address) {
            return i;
        }
    }
    return -1;
}

uint32_t flora_temp_conversion_ms(void)
{
    return s_t.initialized ? s_t.conv_ms : 0;
}

esp_err_t flora_temp_start_conversion(void)
{
    ESP_RETURN_ON_FALSE(s_t.initialized, ESP_ERR_INVALID_STATE, TAG, "nie zainicjalizowano");
    /* Jedna komenda dla calej magistrali: wszystkie czujniki konwertuja
     * rownolegle, wiec czas nie rosnie z ich liczba. */
    return ds18b20_trigger_temperature_conversion_for_all(s_t.bus);
}

esp_err_t flora_temp_fetch(int index, float *celsius)
{
    ESP_RETURN_ON_FALSE(s_t.initialized, ESP_ERR_INVALID_STATE, TAG, "nie zainicjalizowano");
    ESP_RETURN_ON_FALSE(celsius, ESP_ERR_INVALID_ARG, TAG, "celsius == NULL");
    ESP_RETURN_ON_FALSE(index >= 0 && index < s_t.count, ESP_ERR_INVALID_ARG, TAG,
                        "indeks %d poza zakresem (jest %d czujnikow)", index, s_t.count);

    *celsius = FLORA_TEMP_INVALID;
    ESP_RETURN_ON_ERROR(ds18b20_get_temperature(s_t.dev[index], celsius),
                        TAG, "odczyt czujnika %d", index);
    return ESP_OK;
}

esp_err_t flora_temp_read(int index, float *celsius)
{
    ESP_RETURN_ON_ERROR(flora_temp_start_conversion(), TAG, "zlecenie konwersji");
    vTaskDelay(pdMS_TO_TICKS(s_t.conv_ms));
    return flora_temp_fetch(index, celsius);
}

esp_err_t flora_temp_read_all(float *out, int max, int *count)
{
    ESP_RETURN_ON_FALSE(s_t.initialized, ESP_ERR_INVALID_STATE, TAG, "nie zainicjalizowano");
    ESP_RETURN_ON_FALSE(out && max > 0, ESP_ERR_INVALID_ARG, TAG, "zly bufor");

    ESP_RETURN_ON_ERROR(flora_temp_start_conversion(), TAG, "zlecenie konwersji");
    vTaskDelay(pdMS_TO_TICKS(s_t.conv_ms));

    int n = s_t.count < max ? s_t.count : max;
    int ok = 0;
    for (int i = 0; i < n; i++) {
        /* Blad jednego czujnika nie przerywa odczytu pozostalych - w polu
         * jeden luzny przewod nie powinien zabierac calej telemetrii. */
        if (flora_temp_fetch(i, &out[i]) != ESP_OK) {
            out[i] = FLORA_TEMP_INVALID;
        } else {
            ok++;
        }
    }
    if (count) {
        *count = n;
    }
    return ok > 0 ? ESP_OK : ESP_FAIL;
}
