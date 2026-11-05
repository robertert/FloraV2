/**
 * @file hub_sensorcal.c
 * @brief Kalibracje pompki i INA226, procedury w hub_sensorcal.h.
 */
#include <math.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "hub_config.h"
#include "hub_sensorcal.h"
#include "hub_telemetry.h"
#include "flora_board.h"
#include "flora_ina226.h"
#include "flora_pump.h"
#include "flora_store.h"

static const char *TAG = "kal_czujnikow";

#define INA_MIN_A       0.3f    /**< ponizej rozdzielczosc i szum zjadaja dokladnosc */
#define INA_MAX_CHANGE  0.15f   /**< +-15%: wiecej to raczej zly pomiar niz tolerancja */
#define INA_SAMPLES     20

static float s_shunt_active = FLORA_SHUNT_MILLIOHM;

static float nvs_float(const char *key, float def)
{
    float v = def;
    flora_store_t h;
    if (flora_store_open(FLORA_NVS_NS_CALIB, &h) == ESP_OK) {
        flora_store_get_float(h, key, &v, def);
        flora_store_close(h);
    }
    return v;
}

static esp_err_t nvs_set_float(const char *key, float v)
{
    flora_store_t h;
    esp_err_t err = flora_store_open(FLORA_NVS_NS_CALIB, &h);
    if (err == ESP_OK) {
        err = flora_store_set_float(h, key, v);
        flora_store_close(h);
    }
    return err;
}

static bool parse_float(const char *arg, float *out)
{
    char *end;
    float v = strtof(arg, &end);
    if (end == arg || *end || !isfinite(v)) {
        return false;
    }
    *out = v;
    return true;
}

float hub_sensorcal_pump_ml_s(void)
{
    return nvs_float(FLORA_NVS_PUMP_ML_S, 0.0f);
}

float hub_sensorcal_ina_shunt_boot(void)
{
    s_shunt_active = nvs_float(FLORA_NVS_INA_SHUNT, FLORA_SHUNT_MILLIOHM);
    if (s_shunt_active != FLORA_SHUNT_MILLIOHM) {
        ESP_LOGI(TAG, "bocznik INA226 z kalibracji: %.3f mOhm", s_shunt_active);
    }
    return s_shunt_active;
}

float hub_sensorcal_ina_shunt_active(void)
{
    return s_shunt_active;
}

esp_err_t hub_sensorcal_pump(const char *arg)
{
    float ml;
    if (!parse_float(arg, &ml) || ml <= 0.0f || ml > 5000.0f) {
        return ESP_ERR_INVALID_ARG;
    }
    uint32_t ms = hub_telemetry_pump_last_ms();
    if (ms < 2000) {
        /* Przy krotkiej dawce rozruch i spuszczenie powietrza z rurki
         * wazyl by za duzo - wynik bylby zanizony. */
        ESP_LOGW(TAG, "ostatnia dawka %lu ms - pompuj co najmniej 2 s (np. cmd/pump 10000)",
                 (unsigned long)ms);
        return ESP_ERR_INVALID_STATE;
    }
    float ml_s = ml / (ms / 1000.0f);
    esp_err_t err = nvs_set_float(FLORA_NVS_PUMP_ML_S, ml_s);
    if (err == ESP_OK) {
        flora_pump_set_ml_per_second(ml_s);
        ESP_LOGI(TAG, "pompka: %.0f ml w %lu ms = %.2f ml/s, zapisane", ml, (unsigned long)ms, ml_s);
        hub_config_publish();
    }
    return err;
}

esp_err_t hub_sensorcal_ina(const char *arg)
{
    float truth;
    if (!parse_float(arg, &truth) || truth < INA_MIN_A || truth > 20.0f) {
        return ESP_ERR_INVALID_ARG;
    }
    flora_ina226_handle_t ina = hub_telemetry_ina();
    if (!ina) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Srednia z ~1 s - prad silnikow i pompki faluje z PWM. */
    double sum = 0;
    int n = 0;
    for (int i = 0; i < INA_SAMPLES; i++) {
        float a;
        if (flora_ina226_read_current(ina, &a) == ESP_OK) {
            sum += a;
            n++;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (n < INA_SAMPLES / 2) {
        return ESP_FAIL;
    }
    float meas = (float)fabs(sum / n);
    if (meas < INA_MIN_A * 0.5f) {
        ESP_LOGW(TAG, "INA226 widzi %.3f A - czy prad naprawde plynie przez bocznik?", meas);
        return ESP_ERR_INVALID_STATE;
    }

    /* I = U / R: za duzy odczyt znaczy, ze prawdziwy opor jest wiekszy. */
    float ratio = meas / truth;
    if (fabsf(ratio - 1.0f) > INA_MAX_CHANGE) {
        ESP_LOGW(TAG, "INA226 %.3f A vs multimetr %.3f A - roznica %.0f%%, to raczej blad "
                      "pomiaru (zakres multimetru? prad bokiem?) niz tolerancja bocznika",
                 meas, truth, 100.0f * (ratio - 1.0f));
        return ESP_ERR_INVALID_ARG;
    }
    float shunt = s_shunt_active * ratio;
    esp_err_t err = nvs_set_float(FLORA_NVS_INA_SHUNT, shunt);
    if (err == ESP_OK) {
        ESP_LOGW(TAG, "INA226: odczyt %.3f A, prawda %.3f A -> bocznik %.3f mOhm "
                      "(byl %.3f). Dziala po restarcie (cmd/reboot).",
                 meas, truth, shunt, s_shunt_active);
        hub_config_publish();
    }
    return err;
}
