#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "flora_bme280.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "flora_bme280";

#define REG_CALIB_00  0x88   /* 26 bajtow: wspolczynniki T i P oraz H1 */
#define REG_CHIP_ID   0xD0
#define REG_RESET     0xE0
#define REG_CALIB_26  0xE1   /* 7 bajtow: wspolczynniki H2..H6 */
#define REG_CTRL_HUM  0xF2
#define REG_STATUS    0xF3
#define REG_CTRL_MEAS 0xF4
#define REG_CONFIG    0xF5
#define REG_DATA      0xF7   /* 8 bajtow: cisnienie, temperatura, wilgotnosc */

#define CHIP_ID_BME280 0x60
#define CHIP_ID_BMP280 0x58

#define MODE_SLEEP  0x00
#define MODE_FORCED 0x01

#define STATUS_MEASURING 0x08

struct flora_bme280_s {
    i2c_master_dev_handle_t dev;
    uint8_t  chip_id;
    bool     has_humidity;

    flora_bme280_cfg_t cfg;

    /* Wspolczynniki kalibracyjne, fabrycznie zapisane w kazdym egzemplarzu.
     * Bez nich surowe odczyty nie znacza nic - rozrzut miedzy sztukami jest
     * na tyle duzy, ze wspolnej formuly nie da sie uzyc. */
    uint16_t T1; int16_t T2, T3;
    uint16_t P1; int16_t P2, P3, P4, P5, P6, P7, P8, P9;
    uint8_t  H1; int16_t H2; uint8_t H3; int16_t H4, H5; int8_t H6;

    int32_t t_fine;   /* stan posredni dzielony przez wszystkie trzy kompensacje */
};

static esp_err_t reg_read(flora_bme280_handle_t h, uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_master_transmit_receive(h->dev, &reg, 1, buf, len, 100);
}

static esp_err_t reg_write(flora_bme280_handle_t h, uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = { reg, value };
    return i2c_master_transmit(h->dev, buf, sizeof(buf), 100);
}

static esp_err_t read_calibration(flora_bme280_handle_t h)
{
    uint8_t c[26];
    ESP_RETURN_ON_ERROR(reg_read(h, REG_CALIB_00, c, sizeof(c)), TAG, "kalibracja T/P");

    h->T1 = (uint16_t)(c[0]  | (c[1]  << 8));
    h->T2 = (int16_t) (c[2]  | (c[3]  << 8));
    h->T3 = (int16_t) (c[4]  | (c[5]  << 8));
    h->P1 = (uint16_t)(c[6]  | (c[7]  << 8));
    h->P2 = (int16_t) (c[8]  | (c[9]  << 8));
    h->P3 = (int16_t) (c[10] | (c[11] << 8));
    h->P4 = (int16_t) (c[12] | (c[13] << 8));
    h->P5 = (int16_t) (c[14] | (c[15] << 8));
    h->P6 = (int16_t) (c[16] | (c[17] << 8));
    h->P7 = (int16_t) (c[18] | (c[19] << 8));
    h->P8 = (int16_t) (c[20] | (c[21] << 8));
    h->P9 = (int16_t) (c[22] | (c[23] << 8));
    h->H1 = c[25];

    if (!h->has_humidity) {
        return ESP_OK;
    }

    uint8_t d[7];
    ESP_RETURN_ON_ERROR(reg_read(h, REG_CALIB_26, d, sizeof(d)), TAG, "kalibracja H");
    h->H2 = (int16_t)(d[0] | (d[1] << 8));
    h->H3 = d[2];
    /* H4 i H5 sa 12-bitowe i dziela ze soba bajt d[4] - polowka na kazdy. */
    h->H4 = (int16_t)((d[3] << 4) | (d[4] & 0x0F));
    h->H5 = (int16_t)((d[5] << 4) | (d[4] >> 4));
    h->H6 = (int8_t)d[6];
    return ESP_OK;
}

/* Formuly kompensacji wprost z noty katalogowej Boscha, w wersji
 * calkowitoliczbowej. Sa dokladne i nie wciagaja emulacji double. */

static int32_t compensate_temperature(flora_bme280_handle_t h, int32_t adc_T)
{
    int32_t var1 = ((((adc_T >> 3) - ((int32_t)h->T1 << 1))) * ((int32_t)h->T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - ((int32_t)h->T1)) *
                      ((adc_T >> 4) - ((int32_t)h->T1))) >> 12) * ((int32_t)h->T3)) >> 14;
    h->t_fine = var1 + var2;
    return (h->t_fine * 5 + 128) >> 8;      /* setne czesci stopnia */
}

static uint32_t compensate_pressure(flora_bme280_handle_t h, int32_t adc_P)
{
    int64_t var1 = ((int64_t)h->t_fine) - 128000;
    int64_t var2 = var1 * var1 * (int64_t)h->P6;
    var2 = var2 + ((var1 * (int64_t)h->P5) << 17);
    var2 = var2 + (((int64_t)h->P4) << 35);
    var1 = ((var1 * var1 * (int64_t)h->P3) >> 8) + ((var1 * (int64_t)h->P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)h->P1) >> 33;
    if (var1 == 0) {
        return 0;   /* zabezpieczenie przed dzieleniem przez zero */
    }
    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)h->P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)h->P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)h->P7) << 4);
    return (uint32_t)p;                      /* paskale razy 256 */
}

static uint32_t compensate_humidity(flora_bme280_handle_t h, int32_t adc_H)
{
    int32_t v = h->t_fine - ((int32_t)76800);
    v = (((((adc_H << 14) - (((int32_t)h->H4) << 20) - (((int32_t)h->H5) * v)) +
           ((int32_t)16384)) >> 15) *
         (((((((v * ((int32_t)h->H6)) >> 10) *
              (((v * ((int32_t)h->H3)) >> 11) + ((int32_t)32768))) >> 10) +
            ((int32_t)2097152)) * ((int32_t)h->H2) + 8192) >> 14));
    v = v - (((((v >> 15) * (v >> 15)) >> 7) * ((int32_t)h->H1) >> 4));
    if (v < 0)         v = 0;
    if (v > 419430400) v = 419430400;
    return (uint32_t)(v >> 12);              /* procenty razy 1024 */
}

esp_err_t flora_bme280_init(i2c_master_bus_handle_t bus,
                            const flora_bme280_cfg_t *cfg,
                            flora_bme280_handle_t *out)
{
    ESP_RETURN_ON_FALSE(bus && cfg && out, ESP_ERR_INVALID_ARG, TAG, "zly argument");

    flora_bme280_handle_t h = calloc(1, sizeof(struct flora_bme280_s));
    ESP_RETURN_ON_FALSE(h, ESP_ERR_NO_MEM, TAG, "brak pamieci");
    h->cfg = *cfg;

    esp_err_t ret = ESP_OK;
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = cfg->address,
        .scl_speed_hz    = cfg->speed_hz,
    };
    ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(bus, &dev_cfg, &h->dev),
                      fail, TAG, "dodanie urzadzenia 0x%02X", cfg->address);

    ESP_GOTO_ON_ERROR(reg_read(h, REG_CHIP_ID, &h->chip_id, 1), fail, TAG, "odczyt chip_id");
    ESP_GOTO_ON_FALSE(h->chip_id == CHIP_ID_BME280 || h->chip_id == CHIP_ID_BMP280,
                      ESP_ERR_NOT_FOUND, fail, TAG,
                      "pod adresem 0x%02X chip_id = 0x%02X, oczekiwano 0x60 (BME280) "
                      "albo 0x58 (BMP280). Sprawdz adres - drugi wariant to 0x77",
                      cfg->address, h->chip_id);
    h->has_humidity = (h->chip_id == CHIP_ID_BME280);

    ESP_GOTO_ON_ERROR(reg_write(h, REG_RESET, 0xB6), fail, TAG, "reset");
    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_GOTO_ON_ERROR(read_calibration(h), fail, TAG, "wspolczynniki kalibracyjne");

    /* ctrl_hum musi byc zapisany PRZED ctrl_meas - inaczej uklad go zignoruje.
     * To jedna z tych rzeczy, ktore w nocie sa w przypisie, a objawiaja sie
     * stalym odczytem wilgotnosci. */
    if (h->has_humidity) {
        ESP_GOTO_ON_ERROR(reg_write(h, REG_CTRL_HUM, cfg->os_humidity),
                          fail, TAG, "ctrl_hum");
    }
    ESP_GOTO_ON_ERROR(reg_write(h, REG_CONFIG, (uint8_t)(cfg->filter << 2)),
                      fail, TAG, "config");

    ESP_LOGI(TAG, "%s @0x%02X gotowy (chip_id 0x%02X)",
             h->has_humidity ? "BME280" : "BMP280", cfg->address, h->chip_id);
    *out = h;
    return ESP_OK;

fail:
    if (h->dev) {
        i2c_master_bus_rm_device(h->dev);
    }
    free(h);
    return ret;
}

esp_err_t flora_bme280_deinit(flora_bme280_handle_t h)
{
    ESP_RETURN_ON_FALSE(h, ESP_ERR_INVALID_ARG, TAG, "h == NULL");
    reg_write(h, REG_CTRL_MEAS, MODE_SLEEP);
    i2c_master_bus_rm_device(h->dev);
    free(h);
    return ESP_OK;
}

esp_err_t flora_bme280_read(flora_bme280_handle_t h, flora_bme280_data_t *out)
{
    ESP_RETURN_ON_FALSE(h && out, ESP_ERR_INVALID_ARG, TAG, "zly argument");

    /* Tryb wymuszony: jedna konwersja, potem uklad sam wraca do uspienia. */
    uint8_t ctrl = (uint8_t)((h->cfg.os_temperature << 5) |
                             (h->cfg.os_pressure << 2) | MODE_FORCED);
    ESP_RETURN_ON_ERROR(reg_write(h, REG_CTRL_MEAS, ctrl), TAG, "zlecenie pomiaru");

    /* Czekamy na koniec konwersji, odpytujac rejestr stanu. Czas zalezy od
     * nadprobkowania, wiec odpytywanie jest pewniejsze niz staly odstep. */
    int64_t deadline = esp_timer_get_time() + 200000;   /* 200 ms z zapasem */
    uint8_t status = STATUS_MEASURING;
    while (esp_timer_get_time() < deadline) {
        ESP_RETURN_ON_ERROR(reg_read(h, REG_STATUS, &status, 1), TAG, "odczyt stanu");
        if (!(status & STATUS_MEASURING)) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    ESP_RETURN_ON_FALSE(!(status & STATUS_MEASURING), ESP_ERR_TIMEOUT, TAG,
                        "konwersja nie zakonczyla sie w 200 ms");

    uint8_t d[8];
    ESP_RETURN_ON_ERROR(reg_read(h, REG_DATA, d, sizeof(d)), TAG, "odczyt danych");

    int32_t adc_P = (int32_t)(((uint32_t)d[0] << 12) | ((uint32_t)d[1] << 4) | (d[2] >> 4));
    int32_t adc_T = (int32_t)(((uint32_t)d[3] << 12) | ((uint32_t)d[4] << 4) | (d[5] >> 4));
    int32_t adc_H = (int32_t)(((uint32_t)d[6] << 8)  |  (uint32_t)d[7]);

    /* Kolejnosc ma znaczenie: kompensacja temperatury wylicza t_fine,
     * z ktorego korzystaja pozostale dwie. */
    out->temperature_c = (float)compensate_temperature(h, adc_T) / 100.0f;
    out->pressure_pa   = (float)compensate_pressure(h, adc_P) / 256.0f;
    out->humidity_pct  = h->has_humidity
                         ? (float)compensate_humidity(h, adc_H) / 1024.0f
                         : NAN;
    return ESP_OK;
}

bool flora_bme280_has_humidity(flora_bme280_handle_t h)
{
    return h && h->has_humidity;
}

uint8_t flora_bme280_chip_id(flora_bme280_handle_t h)
{
    return h ? h->chip_id : 0;
}

float flora_bme280_sea_level_pa(float pressure_pa, float temperature_c, float altitude_m)
{
    /* Wzor barometryczny. Temperatura wchodzi w kelwinach. */
    float t_k = temperature_c + 273.15f;
    return pressure_pa * powf(1.0f - (0.0065f * altitude_m) / (t_k + 0.0065f * altitude_m), -5.257f);
}
