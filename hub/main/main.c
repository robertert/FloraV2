/**
 * Doniczka-robot, jednostka dolna (bryla).
 *
 * Test napedu na zmontowanej platformie. Teraz, gdy kola sa w podwoziu,
 * da sie wreszcie rozstrzygnac dwie rzeczy, ktore wczesniej wisialy:
 *   - znak odometrii (bez zdefiniowanego "przodu" nie bylo tego jak ustalic)
 *   - prawdziwe przelozenie przekladni (przez porownanie z pomiarem linijka)
 *
 * Pozostale testy, juz zaliczone, czekaja w swoich plikach:
 *   bringup_hydro_run()    - pompka z zabezpieczeniami, DS18B20
 *   bringup_sensors_run()  - wilgotnosc gleby, dalmierz ultradzwiekowy
 *
 * UWAGA: robot pojedzie. Potrzeba okolo metra wolnej przestrzeni albo
 * postawienia go na podstawkach.
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"
#include "driver/i2c_master.h"

#include "bringup_drive.h"
#include "bringup_net.h"
#include "bringup_power.h"
#include "flora_board.h"
#include "flora_ina226.h"
#include "flora_store.h"

static const char *TAG = "dol";

static flora_ina226_handle_t   s_ina;
static i2c_master_bus_handle_t s_i2c_bus;

/** INA226 jest opcjonalny - daje tylko napiecie szyny w logu testu. */
static void ina226_setup(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port          = I2C_NUM_0,
        .sda_io_num        = FLORA_PIN_I2C_SDA,
        .scl_io_num        = FLORA_PIN_I2C_SCL,
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    if (i2c_new_master_bus(&bus_cfg, &s_i2c_bus) != ESP_OK) {
        ESP_LOGW(TAG, "magistrala I2C niedostepna");
        return;
    }

    flora_ina226_cfg_t cfg = FLORA_INA226_DEFAULT_CFG();
    cfg.shunt_milliohm = FLORA_SHUNT_MILLIOHM;
    for (uint8_t addr = 0x40; addr <= 0x4F; addr++) {
        if (i2c_master_probe(s_i2c_bus, addr, 50) != ESP_OK) {
            continue;
        }
        cfg.i2c_address = addr;
        if (flora_ina226_init(s_i2c_bus, &cfg, &s_ina) == ESP_OK) {
            return;
        }
    }
    s_ina = NULL;
    ESP_LOGW(TAG, "nie znalazlem INA226 - test pojdzie bez odczytu zasilania");
}

static void bringup_task(void *arg)
{
    /* Biezacy przedmiot rozruchu. Poprzednie, juz zaliczone, czekaja
     * w swoich plikach i wracaja jednym wywolaniem:
     *   bringup_power_run(s_i2c_bus) - weryfikacja INA226
     *   bringup_drive_calibrate(s_ina, 1) - kalibracja przelozenia
     *   bringup_hydro_run(s_ina)   - pompka, DS18B20
     *   bringup_sensors_run(true)  - gleba, sonar
     *   bringup_net_run()          - WiFi, MQTT
     */
    bringup_drive_run(s_ina);

    ESP_LOGI(TAG, "test zakonczony");
    vTaskDelete(NULL);
}

void app_main(void)
{
    ESP_LOGI(TAG, "doniczka-robot, jednostka dolna: test napedu ze strażnikiem");

    /* NVS jako pierwsze: trzymamy w nim kalibracje, a dodatkowo potrzebuja
     * go WiFi (kalibracja PHY) i BLE, gdy dojda. Wywolanie jest idempotentne. */
    ESP_ERROR_CHECK(flora_store_init());
    flora_store_report_space();

    ina226_setup();
    xTaskCreate(bringup_task, "bringup", 4096, NULL, 5, NULL);
}
