#include <stdlib.h>
#include <string.h>

#include "flora_ina226.h"
#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "flora_ina226";

#define REG_CONFIG       0x00
#define REG_SHUNT_V      0x01
#define REG_BUS_V        0x02
#define REG_POWER        0x03
#define REG_CURRENT      0x04
#define REG_CALIBRATION  0x05
#define REG_MANUF_ID     0xFE
#define REG_DIE_ID       0xFF

#define MANUF_ID_TI      0x5449
#define DIE_ID_INA226    0x2260

/* Stale z noty katalogowej. */
#define BUS_LSB_V        0.00125f    /* 1,25 mV na bit */
#define SHUNT_LSB_MV     0.0025f     /* 2,5 uV na bit */
#define POWER_LSB_RATIO  25.0f       /* power_lsb = 25 * current_lsb */
#define CAL_CONSTANT     0.00512f

struct flora_ina226_s {
    i2c_master_dev_handle_t dev;
    float current_lsb;   /**< A na bit */
    float power_lsb;     /**< W na bit */
};

static esp_err_t reg_write(i2c_master_dev_handle_t dev, uint8_t reg, uint16_t value)
{
    /* INA226 jest big-endian: najpierw starszy bajt. */
    uint8_t buf[3] = { reg, (uint8_t)(value >> 8), (uint8_t)(value & 0xFF) };
    return i2c_master_transmit(dev, buf, sizeof(buf), 100);
}

static esp_err_t reg_read(i2c_master_dev_handle_t dev, uint8_t reg, uint16_t *out)
{
    uint8_t rx[2] = { 0 };
    ESP_RETURN_ON_ERROR(i2c_master_transmit_receive(dev, &reg, 1, rx, sizeof(rx), 100),
                        TAG, "odczyt rejestru 0x%02X", reg);
    *out = ((uint16_t)rx[0] << 8) | rx[1];
    return ESP_OK;
}

/** Mapuje liczbe probek na 3-bitowe pole AVG rejestru konfiguracji. */
static esp_err_t avg_bits(uint16_t samples, uint16_t *out)
{
    switch (samples) {
        case 1:    *out = 0; return ESP_OK;
        case 4:    *out = 1; return ESP_OK;
        case 16:   *out = 2; return ESP_OK;
        case 64:   *out = 3; return ESP_OK;
        case 128:  *out = 4; return ESP_OK;
        case 256:  *out = 5; return ESP_OK;
        case 512:  *out = 6; return ESP_OK;
        case 1024: *out = 7; return ESP_OK;
        default:   return ESP_ERR_INVALID_ARG;
    }
}

esp_err_t flora_ina226_init(i2c_master_bus_handle_t bus,
                            const flora_ina226_cfg_t *cfg,
                            flora_ina226_handle_t *out)
{
    ESP_RETURN_ON_FALSE(bus && cfg && out, ESP_ERR_INVALID_ARG, TAG, "zly argument");
    ESP_RETURN_ON_FALSE(cfg->shunt_milliohm > 0.0f, ESP_ERR_INVALID_ARG, TAG,
                        "shunt_milliohm musi byc dodatnie");
    ESP_RETURN_ON_FALSE(cfg->max_expected_current_a > 0.0f, ESP_ERR_INVALID_ARG, TAG,
                        "max_expected_current_a musi byc dodatnie");

    uint16_t avg = 0;
    ESP_RETURN_ON_FALSE(avg_bits(cfg->averaging_samples, &avg) == ESP_OK,
                        ESP_ERR_INVALID_ARG, TAG,
                        "averaging_samples=%u: dozwolone 1,4,16,64,128,256,512,1024",
                        cfg->averaging_samples);

    float r_ohm = cfg->shunt_milliohm / 1000.0f;
    float hw_ceiling_a = (FLORA_INA226_MAX_SHUNT_MV / 1000.0f) / r_ohm;
    if (cfg->max_expected_current_a > hw_ceiling_a) {
        ESP_LOGW(TAG, "zadano zakres %.2f A, ale bocznik %.1f mOhm daje sufit %.2f A "
                      "(+-81,92 mV) - powyzej odczyt sie nasyci",
                 cfg->max_expected_current_a, cfg->shunt_milliohm, hw_ceiling_a);
    }

    flora_ina226_handle_t h = calloc(1, sizeof(struct flora_ina226_s));
    ESP_RETURN_ON_FALSE(h, ESP_ERR_NO_MEM, TAG, "brak pamieci");

    esp_err_t ret = ESP_OK;
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = cfg->i2c_address,
        .scl_speed_hz    = cfg->i2c_speed_hz,
    };
    ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(bus, &dev_cfg, &h->dev),
                      fail, TAG, "dodanie urzadzenia 0x%02X", cfg->i2c_address);

    /* Zanim cokolwiek skonfigurujemy: czy to na pewno INA226 pod tym adresem. */
    uint16_t manuf = 0, die = 0;
    ESP_GOTO_ON_ERROR(reg_read(h->dev, REG_MANUF_ID, &manuf), fail, TAG, "manuf id");
    ESP_GOTO_ON_FALSE(manuf == MANUF_ID_TI, ESP_ERR_NOT_FOUND, fail, TAG,
                      "pod adresem 0x%02X nie ma INA226 (manuf id 0x%04X, oczekiwano 0x%04X)",
                      cfg->i2c_address, manuf, MANUF_ID_TI);
    if (reg_read(h->dev, REG_DIE_ID, &die) == ESP_OK && die != DIE_ID_INA226) {
        ESP_LOGW(TAG, "nietypowy die id 0x%04X (INA226 ma 0x%04X) - uklad odpowiada, "
                      "ale moze byc innym wariantem", die, DIE_ID_INA226);
    }

    /* Rozdzielczosc pradu: pelna skala licznika 15-bitowego ze znakiem. */
    h->current_lsb = cfg->max_expected_current_a / 32768.0f;
    h->power_lsb   = POWER_LSB_RATIO * h->current_lsb;

    uint16_t cal = (uint16_t)(CAL_CONSTANT / (h->current_lsb * r_ohm));
    ESP_GOTO_ON_FALSE(cal != 0, ESP_ERR_INVALID_ARG, fail, TAG,
                      "wyliczona kalibracja wyszla 0 - sprawdz shunt i zakres pradu");
    ESP_GOTO_ON_ERROR(reg_write(h->dev, REG_CALIBRATION, cal), fail, TAG, "zapis kalibracji");

    /* Bity 14:12 maja stala wartosc 0b100. Czasy konwersji 1,1 ms dla obu
     * torow, tryb ciagly shunt+bus. */
    uint16_t config = 0x4000
                    | (avg << 9)
                    | (0x4 << 6)   /* VBUSCT = 1,1 ms */
                    | (0x4 << 3)   /* VSHCT  = 1,1 ms */
                    | 0x7;         /* MODE: shunt i bus, ciagly */
    ESP_GOTO_ON_ERROR(reg_write(h->dev, REG_CONFIG, config), fail, TAG, "zapis konfiguracji");

    ESP_LOGI(TAG, "INA226 @0x%02X gotowy: bocznik %.1f mOhm, zakres %.2f A, "
                  "%.3f mA/bit, cal=%u, usrednianie %u probek",
             cfg->i2c_address, cfg->shunt_milliohm, cfg->max_expected_current_a,
             h->current_lsb * 1000.0f, cal, cfg->averaging_samples);

    *out = h;
    return ESP_OK;

fail:
    if (h->dev) {
        i2c_master_bus_rm_device(h->dev);
    }
    free(h);
    return ret;
}

esp_err_t flora_ina226_deinit(flora_ina226_handle_t h)
{
    ESP_RETURN_ON_FALSE(h, ESP_ERR_INVALID_ARG, TAG, "h == NULL");
    /* Tryb power-down, zeby uklad nie konwertowal w kolko po zamknieciu. */
    reg_write(h->dev, REG_CONFIG, 0x4000);
    i2c_master_bus_rm_device(h->dev);
    free(h);
    return ESP_OK;
}

esp_err_t flora_ina226_read(flora_ina226_handle_t h, flora_ina226_reading_t *out)
{
    ESP_RETURN_ON_FALSE(h && out, ESP_ERR_INVALID_ARG, TAG, "zly argument");

    uint16_t raw_bus = 0, raw_shunt = 0, raw_current = 0, raw_power = 0;
    ESP_RETURN_ON_ERROR(reg_read(h->dev, REG_BUS_V,   &raw_bus),     TAG, "bus");
    ESP_RETURN_ON_ERROR(reg_read(h->dev, REG_SHUNT_V, &raw_shunt),   TAG, "shunt");
    ESP_RETURN_ON_ERROR(reg_read(h->dev, REG_CURRENT, &raw_current), TAG, "current");
    ESP_RETURN_ON_ERROR(reg_read(h->dev, REG_POWER,   &raw_power),   TAG, "power");

    /* Napiecie szyny jest bez znaku, bocznik i prad ze znakiem (U2). */
    out->bus_voltage_v    = (float)raw_bus * BUS_LSB_V;
    out->shunt_voltage_mv = (float)(int16_t)raw_shunt * SHUNT_LSB_MV;
    out->current_a        = (float)(int16_t)raw_current * h->current_lsb;
    out->power_w          = (float)raw_power * h->power_lsb;
    return ESP_OK;
}

esp_err_t flora_ina226_read_bus_voltage(flora_ina226_handle_t h, float *volts)
{
    ESP_RETURN_ON_FALSE(h && volts, ESP_ERR_INVALID_ARG, TAG, "zly argument");
    uint16_t raw = 0;
    ESP_RETURN_ON_ERROR(reg_read(h->dev, REG_BUS_V, &raw), TAG, "bus");
    *volts = (float)raw * BUS_LSB_V;
    return ESP_OK;
}

esp_err_t flora_ina226_read_current(flora_ina226_handle_t h, float *amps)
{
    ESP_RETURN_ON_FALSE(h && amps, ESP_ERR_INVALID_ARG, TAG, "zly argument");
    uint16_t raw = 0;
    ESP_RETURN_ON_ERROR(reg_read(h->dev, REG_CURRENT, &raw), TAG, "current");
    *amps = (float)(int16_t)raw * h->current_lsb;
    return ESP_OK;
}

float flora_ina226_current_lsb(flora_ina226_handle_t h)
{
    return h ? h->current_lsb : 0.0f;
}
