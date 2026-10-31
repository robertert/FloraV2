/**
 * @file bringup_head.c
 * @brief Test rozruchowy czujnikow glowicy.
 *
 * Wydzielone z main.c, gdy powstala docelowa petla sterowania. Kod zostaje,
 * bo diagnostyka przydaje sie przy kazdej zmianie okablowania pod kopula -
 * skan kanalow muxa pokazuje od razu, czy VEML-e siedza tam, gdzie mialy.
 *
 * Uruchomienie: bringup_head_run() z app_main, przed startem zadan.
 */
#include <inttypes.h>
#include <math.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/i2c_master.h"

#include "bringup_head.h"
#include "flora_head_board.h"
#include "flora_i2cmux.h"
#include "flora_veml7700.h"
#include "flora_bme280.h"
#include "flora_icm20948.h"

static const char *TAG = "rozruch";

static i2c_master_bus_handle_t s_bus;
static flora_i2cmux_handle_t   s_mux;
static flora_i2cmux_port_t    *s_sun_port;
static flora_veml7700_handle_t *s_sun;
static flora_bme280_handle_t   s_bme;
static flora_icm20948_handle_t s_imu;
static int                     s_sun_found;
static const char             *s_sun_name[4] = { "N", "E", "S", "W" };

static int bus_scan(void)
{
    ESP_LOGI(TAG, "skan magistrali glownej (SDA=%d, SCL=%d):",
             FLORA_HEAD_I2C_SDA, FLORA_HEAD_I2C_SCL);
    int n = 0;
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        if (i2c_master_probe(s_bus, addr, 50) != ESP_OK) {
            continue;
        }
        const char *who = "";
        switch (addr) {
            case FLORA_HEAD_ADDR_MUX:     who = "  <- TCA9548A";     break;
            case FLORA_HEAD_ADDR_BME280:  who = "  <- BME280";       break;
            case 0x77:                    who = "  <- BME280 (adres alternatywny)"; break;
            case FLORA_HEAD_ADDR_ICM:     who = "  <- ICM-20948";    break;
            case 0x69:                    who = "  <- ICM-20948 (AD0 w stanie wysokim)"; break;
            case FLORA_HEAD_ADDR_AK09916: who = "  <- magnetometr";  break;
            default: break;
        }
        ESP_LOGI(TAG, "  0x%02X%s", addr, who);
        n++;
    }
    if (n == 0) {
        ESP_LOGE(TAG, "  CISZA. Sprawdz w tej kolejnosci:");
        ESP_LOGE(TAG, "    numery pinow w flora_head_board.h (sa wstepne!),");
        ESP_LOGE(TAG, "    zasilanie czujnikow, rezystory podciagajace, wspolna mase");
    }
    return n;
}

/* ================= KOMPAS SLONECZNY ================= */

static void test_sun_compass(void)
{
    ESP_LOGI(TAG, "=== KOMPAS SLONECZNY ===");

    esp_err_t err = flora_i2cmux_init(s_bus, FLORA_HEAD_ADDR_MUX,
                                      FLORA_HEAD_I2C_FREQ_HZ, &s_mux);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "  multiplekser niedostepny: %s", esp_err_to_name(err));
        return;
    }

    /* Skan kanalow pokazuje od razu, czy czujniki siedza tam, gdzie mialy. */
    flora_i2cmux_scan(s_mux);

    for (int i = 0; i < FLORA_HEAD_SUN_COUNT; i++) {
        s_sun_port[i].mux     = s_mux;
        s_sun_port[i].channel = (uint8_t)i;

        flora_veml7700_cfg_t cfg = FLORA_VEML7700_DEFAULT_CFG();
        cfg.address     = FLORA_HEAD_ADDR_VEML;
        cfg.speed_hz    = FLORA_HEAD_I2C_FREQ_HZ;
        cfg.prepare     = flora_i2cmux_port_select;
        cfg.prepare_ctx = &s_sun_port[i];

        if (flora_veml7700_init(s_bus, &cfg, &s_sun[i]) == ESP_OK) {
            s_sun_found++;
        } else {
            ESP_LOGE(TAG, "  kanal %d (%s): brak czujnika", i, s_sun_name[i]);
            s_sun[i] = NULL;
        }
    }
    ESP_LOGI(TAG, "  znaleziono %d z %d czujnikow", s_sun_found, FLORA_HEAD_SUN_COUNT);

    if (s_sun_found == 0) {
        return;
    }

    ESP_LOGI(TAG, "  odczyt poczatkowy:");
    for (int i = 0; i < FLORA_HEAD_SUN_COUNT; i++) {
        if (!s_sun[i]) {
            continue;
        }
        float lux = 0.0f;
        uint16_t raw = 0;
        flora_veml7700_read_raw(s_sun[i], &raw);
        if (flora_veml7700_read_lux(s_sun[i], &lux) == ESP_OK) {
            ESP_LOGI(TAG, "    %s: %8.1f lx (surowo %5u)", s_sun_name[i], lux, raw);
            /* Nasycenie oznacza, ze czulosc jest za duza na te warunki. */
            if (raw >= 65000) {
                ESP_LOGW(TAG, "       odczyt przy maksimum - zmniejsz czulosc, "
                              "inaczej kompas nie rozrozni kierunkow");
            }
        }
    }
    ESP_LOGI(TAG, "  zaslon po jednym czujniku i sprawdz w podgladzie,");
    ESP_LOGI(TAG, "  czy kanal zgadza sie z kierunkiem w uchwycie");
}

/* ================= BME280 ================= */

static void test_bme280(void)
{
    ESP_LOGI(TAG, "=== BME280 ===");

    flora_bme280_cfg_t cfg = FLORA_BME280_DEFAULT_CFG();
    cfg.address  = FLORA_HEAD_ADDR_BME280;
    cfg.speed_hz = FLORA_HEAD_I2C_FREQ_HZ;

    esp_err_t err = flora_bme280_init(s_bus, &cfg, &s_bme);
    if (err == ESP_ERR_NOT_FOUND) {
        /* Moduly roznia sie stanem pinu SDO, wiec druga proba pod 0x77. */
        ESP_LOGW(TAG, "  brak pod 0x%02X, probuje 0x77", cfg.address);
        cfg.address = 0x77;
        err = flora_bme280_init(s_bus, &cfg, &s_bme);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "  niedostepny: %s", esp_err_to_name(err));
        s_bme = NULL;
        return;
    }

    if (!flora_bme280_has_humidity(s_bme)) {
        ESP_LOGW(TAG, "  to BMP280 (chip_id 0x%02X), nie BME280 - bez wilgotnosci",
                 flora_bme280_chip_id(s_bme));
    }

    flora_bme280_data_t d;
    if (flora_bme280_read(s_bme, &d) == ESP_OK) {
        ESP_LOGI(TAG, "  %.2f C, %.1f hPa", d.temperature_c, d.pressure_pa / 100.0f);
        if (!isnan(d.humidity_pct)) {
            ESP_LOGI(TAG, "  wilgotnosc %.1f%%", d.humidity_pct);
        }
        /* Krakow lezy okolo 220 m n.p.m. - popraw, gdy testujesz gdzie indziej. */
        ESP_LOGI(TAG, "  po przeliczeniu na poziom morza: %.1f hPa (dla 220 m)",
                 flora_bme280_sea_level_pa(d.pressure_pa, d.temperature_c, 220.0f) / 100.0f);
    }
}

/* ================= IMU ================= */

static void test_imu(void)
{
    ESP_LOGI(TAG, "=== ICM-20948 ===");

    flora_icm20948_cfg_t cfg = FLORA_ICM20948_DEFAULT_CFG();
    cfg.address  = FLORA_HEAD_ADDR_ICM;
    cfg.speed_hz = FLORA_HEAD_I2C_FREQ_HZ;

    esp_err_t err = flora_icm20948_init(s_bus, &cfg, &s_imu);
    if (err == ESP_ERR_NOT_FOUND) {
        ESP_LOGW(TAG, "  brak pod 0x%02X, probuje 0x69", cfg.address);
        cfg.address = 0x69;
        err = flora_icm20948_init(s_bus, &cfg, &s_imu);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "  niedostepny: %s", esp_err_to_name(err));
        s_imu = NULL;
        return;
    }

    flora_icm20948_data_t d;
    if (flora_icm20948_read(s_imu, &d) == ESP_OK) {
        float g = sqrtf(d.accel_g[0] * d.accel_g[0] +
                        d.accel_g[1] * d.accel_g[1] +
                        d.accel_g[2] * d.accel_g[2]);
        ESP_LOGI(TAG, "  przyspieszenie: %+.3f %+.3f %+.3f g, wypadkowa %.3f g",
                 d.accel_g[0], d.accel_g[1], d.accel_g[2], g);
        /* Nieruchomy czujnik musi widziec 1 g. Odchylka to zly zakres,
         * zle skalowanie albo uklad w ruchu. */
        if (g < 0.9f || g > 1.1f) {
            ESP_LOGW(TAG, "  wypadkowa odbiega od 1 g - czujnik w ruchu albo zle skalowanie");
        }
        ESP_LOGI(TAG, "  temperatura ukladu: %.1f C", d.temperature_c);
    }

    ESP_LOGI(TAG, "  kalibracja zyroskopu - NIE RUSZAJ masztem przez ~2 s");
    vTaskDelay(pdMS_TO_TICKS(500));
    flora_icm20948_calibrate_gyro(s_imu, 200);

    if (flora_icm20948_has_magnetometer(s_imu)) {
        flora_icm20948_mag_t m;
        vTaskDelay(pdMS_TO_TICKS(50));
        if (flora_icm20948_read_mag(s_imu, &m) == ESP_OK) {
            float f = sqrtf(m.field_ut[0] * m.field_ut[0] +
                            m.field_ut[1] * m.field_ut[1] +
                            m.field_ut[2] * m.field_ut[2]);
            ESP_LOGI(TAG, "  pole magnetyczne: %+.1f %+.1f %+.1f uT, wypadkowa %.1f uT",
                     m.field_ut[0], m.field_ut[1], m.field_ut[2], f);
            /* Pole ziemskie w Polsce to okolo 50 uT. Znacznie wieksza
             * wartosc oznacza zelazo albo przewod z pradem w poblizu. */
            if (f > 80.0f) {
                ESP_LOGW(TAG, "  znacznie powyzej pola ziemskiego (~50 uT) - "
                              "w poblizu jest zelazo albo przewod pod obciazeniem");
            }
        }
    }
}

void bringup_head_run(const bringup_head_ctx_t *ctx)
{
    s_bus      = ctx->bus;
    s_mux      = ctx->mux;
    s_sun_port = ctx->sun_port;
    s_sun      = ctx->sun;
    s_bme      = ctx->bme;
    s_imu      = ctx->imu;

    if (bus_scan() == 0) {
        ESP_LOGE(TAG, "brak urzadzen na magistrali");
        return;
    }
    test_sun_compass();
    test_bme280();
    test_imu();
}

flora_i2cmux_handle_t   bringup_head_mux(void)      { return s_mux; }
flora_veml7700_handle_t bringup_head_sun(int i)     { return (i >= 0 && i < 4) ? s_sun[i] : NULL; }
flora_bme280_handle_t   bringup_head_bme(void)      { return s_bme; }
flora_icm20948_handle_t bringup_head_imu(void)      { return s_imu; }
