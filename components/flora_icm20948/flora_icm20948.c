#include <stdlib.h>
#include <string.h>

#include "flora_icm20948.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "flora_icm";

/* Wybor banku jest widoczny pod tym samym adresem we WSZYSTKICH bankach -
 * dzieki temu da sie przelaczyc bank nie wiedzac, w ktorym sie jest. */
#define REG_BANK_SEL     0x7F

/* --- bank 0 --- */
#define B0_WHO_AM_I      0x00
#define B0_USER_CTRL     0x03
#define B0_PWR_MGMT_1    0x06
#define B0_PWR_MGMT_2    0x07
#define B0_INT_PIN_CFG   0x0F
#define B0_ACCEL_XOUT_H  0x2D
#define B0_TEMP_OUT_H    0x39

/* --- bank 2 --- */
#define B2_GYRO_SMPLRT   0x00
#define B2_GYRO_CONFIG_1 0x01
#define B2_ACCEL_CONFIG  0x14

#define WHO_AM_I_VALUE   0xEA

#define USER_CTRL_I2C_MST_EN  (1u << 5)
#define INT_PIN_CFG_BYPASS_EN (1u << 1)
#define PWR1_DEVICE_RESET     0x80
#define PWR1_CLKSEL_AUTO      0x01

/* --- AK09916, magnetometr --- */
#define AK_WIA2          0x01
#define AK_ST1           0x10
#define AK_HXL           0x11
#define AK_ST2           0x18
#define AK_CNTL2         0x31
#define AK_CNTL3         0x32

#define AK_WIA2_VALUE    0x09
#define AK_MODE_CONT_100HZ 0x08
#define AK_ST1_DRDY      0x01
#define AK_ST2_HOFL      0x08

/** Mikrotesle na bit - stala dla AK09916. */
#define AK_SCALE_UT      0.15f

struct flora_icm20948_s {
    i2c_master_dev_handle_t dev;
    i2c_master_dev_handle_t mag;
    uint8_t who_am_i;
    uint8_t current_bank;
    bool    bank_known;
    bool    has_mag;

    float accel_scale;   /**< g na bit */
    float gyro_scale;    /**< dps na bit */
    float gyro_bias[3];
};

static esp_err_t raw_write(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t v)
{
    uint8_t buf[2] = { reg, v };
    return i2c_master_transmit(dev, buf, sizeof(buf), 100);
}

static esp_err_t raw_read(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_master_transmit_receive(dev, &reg, 1, buf, len, 100);
}

static esp_err_t set_bank(flora_icm20948_handle_t h, uint8_t bank)
{
    if (h->bank_known && h->current_bank == bank) {
        return ESP_OK;
    }
    ESP_RETURN_ON_ERROR(raw_write(h->dev, REG_BANK_SEL, (uint8_t)(bank << 4)),
                        TAG, "wybor banku %u", bank);
    h->current_bank = bank;
    h->bank_known   = true;
    return ESP_OK;
}

static esp_err_t bank_write(flora_icm20948_handle_t h, uint8_t bank, uint8_t reg, uint8_t v)
{
    ESP_RETURN_ON_ERROR(set_bank(h, bank), TAG, "bank");
    return raw_write(h->dev, reg, v);
}

static esp_err_t bank_read(flora_icm20948_handle_t h, uint8_t bank, uint8_t reg,
                           uint8_t *buf, size_t len)
{
    ESP_RETURN_ON_ERROR(set_bank(h, bank), TAG, "bank");
    return raw_read(h->dev, reg, buf, len);
}

/* Czulosci z noty katalogowej. */
static const float ACCEL_LSB_PER_G[]   = { 16384.0f, 8192.0f, 4096.0f, 2048.0f };
static const float GYRO_LSB_PER_DPS[]  = { 131.0f, 65.5f, 32.8f, 16.4f };

static esp_err_t mag_setup(flora_icm20948_handle_t h, i2c_master_bus_handle_t bus,
                           const flora_icm20948_cfg_t *cfg)
{
    /* Tryb przelotowy: wylaczamy wewnetrzny kontroler I2C i zwieramy
     * magistrale pomocnicza do glownej. Dopiero wtedy AK09916 odpowiada. */
    ESP_RETURN_ON_ERROR(bank_write(h, 0, B0_USER_CTRL, 0x00), TAG, "wylaczenie kontrolera I2C");
    ESP_RETURN_ON_ERROR(bank_write(h, 0, B0_INT_PIN_CFG, INT_PIN_CFG_BYPASS_EN),
                        TAG, "wlaczenie trybu przelotowego");
    vTaskDelay(pdMS_TO_TICKS(10));

    i2c_device_config_t mag_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = cfg->mag_address,
        .scl_speed_hz    = cfg->speed_hz,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus, &mag_cfg, &h->mag),
                        TAG, "dodanie magnetometru 0x%02X", cfg->mag_address);

    uint8_t wia = 0;
    ESP_RETURN_ON_ERROR(raw_read(h->mag, AK_WIA2, &wia, 1), TAG, "odczyt WIA2");
    ESP_RETURN_ON_FALSE(wia == AK_WIA2_VALUE, ESP_ERR_NOT_FOUND, TAG,
                        "AK09916 pod 0x%02X zglasza 0x%02X zamiast 0x%02X",
                        cfg->mag_address, wia, AK_WIA2_VALUE);

    ESP_RETURN_ON_ERROR(raw_write(h->mag, AK_CNTL3, 0x01), TAG, "reset magnetometru");
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_RETURN_ON_ERROR(raw_write(h->mag, AK_CNTL2, AK_MODE_CONT_100HZ),
                        TAG, "tryb ciagly");
    vTaskDelay(pdMS_TO_TICKS(10));
    return ESP_OK;
}

esp_err_t flora_icm20948_init(i2c_master_bus_handle_t bus,
                              const flora_icm20948_cfg_t *cfg,
                              flora_icm20948_handle_t *out)
{
    ESP_RETURN_ON_FALSE(bus && cfg && out, ESP_ERR_INVALID_ARG, TAG, "zly argument");

    flora_icm20948_handle_t h = calloc(1, sizeof(struct flora_icm20948_s));
    ESP_RETURN_ON_FALSE(h, ESP_ERR_NO_MEM, TAG, "brak pamieci");

    h->accel_scale = 1.0f / ACCEL_LSB_PER_G[cfg->accel_fs];
    h->gyro_scale  = 1.0f / GYRO_LSB_PER_DPS[cfg->gyro_fs];

    esp_err_t ret = ESP_OK;
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = cfg->address,
        .scl_speed_hz    = cfg->speed_hz,
    };
    ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(bus, &dev_cfg, &h->dev),
                      fail, TAG, "dodanie urzadzenia 0x%02X", cfg->address);

    ESP_GOTO_ON_ERROR(bank_read(h, 0, B0_WHO_AM_I, &h->who_am_i, 1), fail, TAG, "WHO_AM_I");
    ESP_GOTO_ON_FALSE(h->who_am_i == WHO_AM_I_VALUE, ESP_ERR_NOT_FOUND, fail, TAG,
                      "pod adresem 0x%02X WHO_AM_I = 0x%02X, oczekiwano 0x%02X. "
                      "Drugi mozliwy adres to 0x69 (pin AD0 w stanie wysokim)",
                      cfg->address, h->who_am_i, WHO_AM_I_VALUE);

    ESP_GOTO_ON_ERROR(bank_write(h, 0, B0_PWR_MGMT_1, PWR1_DEVICE_RESET), fail, TAG, "reset");
    vTaskDelay(pdMS_TO_TICKS(50));
    h->bank_known = false;   /* reset przywraca bank 0, ale nie zgadujemy */

    ESP_GOTO_ON_ERROR(bank_write(h, 0, B0_PWR_MGMT_1, PWR1_CLKSEL_AUTO),
                      fail, TAG, "wybudzenie");
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_GOTO_ON_ERROR(bank_write(h, 0, B0_PWR_MGMT_2, 0x00), fail, TAG,
                      "zalaczenie akcelerometru i zyroskopu");

    /* Zakresy pomiarowe siedza w banku 2, bit 0 kazdego rejestru wlacza
     * filtr dolnoprzepustowy - zostawiamy go wlaczonym. */
    ESP_GOTO_ON_ERROR(bank_write(h, 2, B2_GYRO_CONFIG_1,
                                 (uint8_t)((cfg->gyro_fs << 1) | 0x01)),
                      fail, TAG, "zakres zyroskopu");
    ESP_GOTO_ON_ERROR(bank_write(h, 2, B2_ACCEL_CONFIG,
                                 (uint8_t)((cfg->accel_fs << 1) | 0x01)),
                      fail, TAG, "zakres akcelerometru");
    ESP_GOTO_ON_ERROR(set_bank(h, 0), fail, TAG, "powrot do banku 0");

    if (cfg->enable_magnetometer) {
        if (mag_setup(h, bus, cfg) == ESP_OK) {
            h->has_mag = true;
        } else {
            /* Brak magnetometru nie blokuje akcelerometru i zyroskopu. */
            ESP_LOGW(TAG, "magnetometr niedostepny - akcelerometr i zyroskop beda dzialac");
            if (h->mag) {
                i2c_master_bus_rm_device(h->mag);
                h->mag = NULL;
            }
        }
    }

    ESP_LOGI(TAG, "ICM-20948 @0x%02X gotowy: akcelerometr +-%dg, zyroskop +-%ddps%s",
             cfg->address,
             2 << cfg->accel_fs,
             250 << cfg->gyro_fs,
             h->has_mag ? ", magnetometr w trybie przelotowym" : "");
    *out = h;
    return ESP_OK;

fail:
    if (h->mag) i2c_master_bus_rm_device(h->mag);
    if (h->dev) i2c_master_bus_rm_device(h->dev);
    free(h);
    return ret;
}

esp_err_t flora_icm20948_deinit(flora_icm20948_handle_t h)
{
    ESP_RETURN_ON_FALSE(h, ESP_ERR_INVALID_ARG, TAG, "h == NULL");
    if (h->mag) {
        raw_write(h->mag, AK_CNTL2, 0x00);   /* magnetometr w uspienie */
        i2c_master_bus_rm_device(h->mag);
    }
    bank_write(h, 0, B0_PWR_MGMT_1, 0x40);   /* uspienie ukladu */
    i2c_master_bus_rm_device(h->dev);
    free(h);
    return ESP_OK;
}

esp_err_t flora_icm20948_read(flora_icm20948_handle_t h, flora_icm20948_data_t *out)
{
    ESP_RETURN_ON_FALSE(h && out, ESP_ERR_INVALID_ARG, TAG, "zly argument");

    /* Akcelerometr, zyroskop i temperatura leza obok siebie - jeden
     * odczyt 14 bajtow daje spojna migawke wszystkich osi. */
    uint8_t d[14];
    ESP_RETURN_ON_ERROR(bank_read(h, 0, B0_ACCEL_XOUT_H, d, sizeof(d)), TAG, "odczyt danych");

    for (int i = 0; i < 3; i++) {
        int16_t a = (int16_t)((d[i * 2] << 8) | d[i * 2 + 1]);
        int16_t g = (int16_t)((d[6 + i * 2] << 8) | d[6 + i * 2 + 1]);
        out->accel_g[i]  = (float)a * h->accel_scale;
        out->gyro_dps[i] = (float)g * h->gyro_scale - h->gyro_bias[i];
    }

    int16_t t = (int16_t)((d[12] << 8) | d[13]);
    out->temperature_c = ((float)t / 333.87f) + 21.0f;
    return ESP_OK;
}

esp_err_t flora_icm20948_read_mag(flora_icm20948_handle_t h, flora_icm20948_mag_t *out)
{
    ESP_RETURN_ON_FALSE(h && out, ESP_ERR_INVALID_ARG, TAG, "zly argument");
    ESP_RETURN_ON_FALSE(h->has_mag, ESP_ERR_NOT_SUPPORTED, TAG, "magnetometr wylaczony");

    uint8_t st1 = 0;
    ESP_RETURN_ON_ERROR(raw_read(h->mag, AK_ST1, &st1, 1), TAG, "odczyt ST1");
    if (!(st1 & AK_ST1_DRDY)) {
        return ESP_ERR_NOT_FINISHED;
    }

    /* Czytamy sesc bajtow danych RAZEM z rejestrem ST2 - bez jego odczytu
     * AK09916 nie zwalnia bufora i kolejny pomiar nie zostanie zapisany. */
    uint8_t d[8];
    ESP_RETURN_ON_ERROR(raw_read(h->mag, AK_HXL, d, sizeof(d)), TAG, "odczyt pola");

    /* Uwaga: magnetometr oddaje dane little-endian, odwrotnie niz
     * akcelerometr i zyroskop w tym samym ukladzie. */
    for (int i = 0; i < 3; i++) {
        int16_t v = (int16_t)(d[i * 2] | (d[i * 2 + 1] << 8));
        out->field_ut[i] = (float)v * AK_SCALE_UT;
    }
    out->overflow = (d[7] & AK_ST2_HOFL) != 0;
    return ESP_OK;
}

esp_err_t flora_icm20948_calibrate_gyro(flora_icm20948_handle_t h, int samples)
{
    ESP_RETURN_ON_FALSE(h, ESP_ERR_INVALID_ARG, TAG, "h == NULL");
    ESP_RETURN_ON_FALSE(samples > 0, ESP_ERR_INVALID_ARG, TAG, "samples musi byc > 0");

    memset(h->gyro_bias, 0, sizeof(h->gyro_bias));

    double sum[3] = { 0.0, 0.0, 0.0 };
    for (int i = 0; i < samples; i++) {
        flora_icm20948_data_t d;
        ESP_RETURN_ON_ERROR(flora_icm20948_read(h, &d), TAG, "odczyt przy kalibracji");
        for (int ax = 0; ax < 3; ax++) {
            sum[ax] += d.gyro_dps[ax];
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    for (int ax = 0; ax < 3; ax++) {
        h->gyro_bias[ax] = (float)(sum[ax] / samples);
    }
    ESP_LOGI(TAG, "offset zyroskopu: X %.3f, Y %.3f, Z %.3f dps",
             h->gyro_bias[0], h->gyro_bias[1], h->gyro_bias[2]);
    return ESP_OK;
}

void flora_icm20948_get_gyro_bias(flora_icm20948_handle_t h, float bias_dps[3])
{
    if (h && bias_dps) {
        memcpy(bias_dps, h->gyro_bias, sizeof(h->gyro_bias));
    }
}

bool flora_icm20948_has_magnetometer(flora_icm20948_handle_t h)
{
    return h && h->has_mag;
}

uint8_t flora_icm20948_who_am_i(flora_icm20948_handle_t h)
{
    return h ? h->who_am_i : 0;
}
