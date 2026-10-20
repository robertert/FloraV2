#include <inttypes.h>
#include <stdlib.h>
#include <string.h>

#include "flora_veml7700.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "flora_veml";

#define REG_ALS_CONF   0x00
#define REG_ALS        0x04
#define REG_WHITE      0x05
#define REG_POWER_SAVE 0x03

/* Pola rejestru konfiguracji. */
#define CONF_SHUTDOWN  (1u << 0)

struct flora_veml7700_s {
    i2c_master_dev_handle_t dev;
    flora_veml_gain_t       gain;
    flora_veml_it_t         it;
    bool                    correction;
    flora_veml_prepare_cb_t prepare;
    void                   *prepare_ctx;
};

/* Kodowanie pol w rejestrze ALS_CONF - wartosci wprost z noty katalogowej. */
static const uint16_t GAIN_BITS[] = { 0x0, 0x1, 0x2, 0x3 };
static const uint16_t IT_BITS[]   = { 0xC, 0x8, 0x0, 0x1, 0x2, 0x3 };

/* Mnozniki do wyliczenia rozdzielczosci. */
static const float GAIN_FACTOR[] = { 1.0f, 2.0f, 0.125f, 0.25f };
static const uint32_t IT_MS[]    = { 25, 50, 100, 200, 400, 800 };

/** Najwieksza czulosc: wzmocnienie x2 i 800 ms daje 0,0036 lx na zliczenie. */
#define RES_AT_MAX_SENSITIVITY 0.0036f

static float resolution_of(flora_veml_gain_t g, flora_veml_it_t it)
{
    return RES_AT_MAX_SENSITIVITY
           * (2.0f / GAIN_FACTOR[g])
           * (800.0f / (float)IT_MS[it]);
}

static esp_err_t prepare(flora_veml7700_handle_t h)
{
    return h->prepare ? h->prepare(h->prepare_ctx) : ESP_OK;
}

static esp_err_t reg_write(flora_veml7700_handle_t h, uint8_t reg, uint16_t value)
{
    ESP_RETURN_ON_ERROR(prepare(h), TAG, "przygotowanie magistrali");
    /* VEML7700 jest little-endian: najpierw mlodszy bajt. */
    uint8_t buf[3] = { reg, (uint8_t)(value & 0xFF), (uint8_t)(value >> 8) };
    return i2c_master_transmit(h->dev, buf, sizeof(buf), 100);
}

static esp_err_t reg_read(flora_veml7700_handle_t h, uint8_t reg, uint16_t *out)
{
    ESP_RETURN_ON_ERROR(prepare(h), TAG, "przygotowanie magistrali");
    uint8_t rx[2] = { 0 };
    ESP_RETURN_ON_ERROR(i2c_master_transmit_receive(h->dev, &reg, 1, rx, sizeof(rx), 100),
                        TAG, "odczyt rejestru 0x%02X", reg);
    *out = (uint16_t)rx[0] | ((uint16_t)rx[1] << 8);
    return ESP_OK;
}

static esp_err_t write_config(flora_veml7700_handle_t h)
{
    uint16_t conf = (GAIN_BITS[h->gain] << 11) | (IT_BITS[h->it] << 6);
    return reg_write(h, REG_ALS_CONF, conf);   /* bit SHUTDOWN zostaje zerem */
}

esp_err_t flora_veml7700_init(i2c_master_bus_handle_t bus,
                              const flora_veml7700_cfg_t *cfg,
                              flora_veml7700_handle_t *out)
{
    ESP_RETURN_ON_FALSE(bus && cfg && out, ESP_ERR_INVALID_ARG, TAG, "zly argument");

    flora_veml7700_handle_t h = calloc(1, sizeof(struct flora_veml7700_s));
    ESP_RETURN_ON_FALSE(h, ESP_ERR_NO_MEM, TAG, "brak pamieci");

    h->gain        = cfg->gain;
    h->it          = cfg->integration_time;
    h->correction  = cfg->apply_correction;
    h->prepare     = cfg->prepare;
    h->prepare_ctx = cfg->prepare_ctx;

    esp_err_t ret = ESP_OK;
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = cfg->address,
        .scl_speed_hz    = cfg->speed_hz,
    };
    ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(bus, &dev_cfg, &h->dev),
                      fail, TAG, "dodanie urzadzenia 0x%02X", cfg->address);

    ESP_GOTO_ON_ERROR(write_config(h), fail, TAG, "zapis konfiguracji");

    /* VEML7700 nie ma rejestru identyfikacyjnego. Sprawdzamy wiec, czy
     * zapisana konfiguracja wraca taka sama - to odroznia dzialajacy
     * uklad od przypadkowego potwierdzenia na magistrali. */
    uint16_t readback = 0;
    ESP_GOTO_ON_ERROR(reg_read(h, REG_ALS_CONF, &readback), fail, TAG, "odczyt konfiguracji");
    uint16_t expected = (GAIN_BITS[h->gain] << 11) | (IT_BITS[h->it] << 6);
    ESP_GOTO_ON_FALSE((readback & 0x1FC0) == (expected & 0x1FC0), ESP_ERR_NOT_FOUND,
                      fail, TAG,
                      "pod adresem 0x%02X konfiguracja nie wraca poprawnie "
                      "(zapisano 0x%04X, odczytano 0x%04X) - to chyba nie VEML7700",
                      cfg->address, expected, readback);

    /* Pierwszy pomiar potrzebuje pelnego okresu calkowania. */
    vTaskDelay(pdMS_TO_TICKS(IT_MS[h->it] + 10));

    ESP_LOGI(TAG, "VEML7700 @0x%02X: wzmocnienie %.3fx, calkowanie %" PRIu32 " ms, "
                  "%.4f lx na zliczenie%s",
             cfg->address, GAIN_FACTOR[h->gain], IT_MS[h->it],
             resolution_of(h->gain, h->it),
             h->correction ? ", korekta nieliniowosci wlaczona" : "");
    *out = h;
    return ESP_OK;

fail:
    if (h->dev) {
        i2c_master_bus_rm_device(h->dev);
    }
    free(h);
    return ret;
}

esp_err_t flora_veml7700_deinit(flora_veml7700_handle_t h)
{
    ESP_RETURN_ON_FALSE(h, ESP_ERR_INVALID_ARG, TAG, "h == NULL");
    reg_write(h, REG_ALS_CONF, CONF_SHUTDOWN);
    i2c_master_bus_rm_device(h->dev);
    free(h);
    return ESP_OK;
}

/**
 * Korekta nieliniowosci z noty aplikacyjnej Vishay. Czujnik zanizą odczyt
 * przy duzych natezeniach i bez tego blad rosnie wraz z jasnoscia - przy
 * slonecznym niebie siega kilkudziesieciu procent.
 */
static float correct(float lux)
{
    return 6.0135e-13f * lux * lux * lux * lux
         - 9.3924e-9f  * lux * lux * lux
         + 8.1488e-5f  * lux * lux
         + 1.0023f     * lux;
}

static esp_err_t read_channel(flora_veml7700_handle_t h, uint8_t reg, float *lux)
{
    uint16_t raw = 0;
    ESP_RETURN_ON_ERROR(reg_read(h, reg, &raw), TAG, "odczyt kanalu");

    float value = (float)raw * resolution_of(h->gain, h->it);
    if (h->correction) {
        value = correct(value);
    }
    *lux = value;
    return ESP_OK;
}

esp_err_t flora_veml7700_read_lux(flora_veml7700_handle_t h, float *lux)
{
    ESP_RETURN_ON_FALSE(h && lux, ESP_ERR_INVALID_ARG, TAG, "zly argument");
    return read_channel(h, REG_ALS, lux);
}

esp_err_t flora_veml7700_read_white(flora_veml7700_handle_t h, float *lux)
{
    ESP_RETURN_ON_FALSE(h && lux, ESP_ERR_INVALID_ARG, TAG, "zly argument");
    return read_channel(h, REG_WHITE, lux);
}

esp_err_t flora_veml7700_read_raw(flora_veml7700_handle_t h, uint16_t *raw)
{
    ESP_RETURN_ON_FALSE(h && raw, ESP_ERR_INVALID_ARG, TAG, "zly argument");
    return reg_read(h, REG_ALS, raw);
}

esp_err_t flora_veml7700_set_sensitivity(flora_veml7700_handle_t h,
                                         flora_veml_gain_t gain, flora_veml_it_t it)
{
    ESP_RETURN_ON_FALSE(h, ESP_ERR_INVALID_ARG, TAG, "h == NULL");
    h->gain = gain;
    h->it   = it;
    ESP_RETURN_ON_ERROR(write_config(h), TAG, "zapis konfiguracji");
    vTaskDelay(pdMS_TO_TICKS(IT_MS[it] + 10));
    return ESP_OK;
}

float flora_veml7700_resolution(flora_veml7700_handle_t h)
{
    return h ? resolution_of(h->gain, h->it) : 0.0f;
}

uint32_t flora_veml7700_settle_ms(flora_veml7700_handle_t h)
{
    return h ? IT_MS[h->it] + 10 : 0;
}
