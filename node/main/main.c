/**
 * Doniczka-robot, glowica masztu.
 *
 * Docelowa petla sterowania: cykliczne probkowanie czujnikow, fuzja kursu
 * i telemetria do bryly. Dwa zadania robocze plus zadania odbiorcze
 * w flora_link:
 *
 *   head_fusion   60 Hz   IMU, kurs, ATTITUDE co 3. tick = 20 Hz
 *   head_slow      1 Hz   kompas sloneczny, BME280, HEAD_STATUS
 *
 * Czestotliwosci policzone, nie przyjete z wyczucia: przy 100 kHz na I2C
 * daja 9,1% obciazenia magistrali, a blad calkowania zyroskopu przy 60 Hz
 * to 0,5 stopnia na probke przy obrocie 30 st/s.
 *
 * Sterownik I2C w ESP-IDF 6.1 serializuje pojedyncze transakcje wlasnym
 * semaforem, wiec oba zadania moga dzielic magistrale bez dodatkowego
 * mutexu. Jedno zastrzezenie: para "wybierz kanal muxa, potem odczytaj"
 * NIE jest niepodzielna, dlatego wszystkie cztery VEML-e obsluguje
 * wylacznie head_slow.
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"
#include "driver/i2c_master.h"

#include "bringup_head.h"
#include "bringup_link.h"
#include "head_cmd.h"
#include "head_fusion.h"
#include "head_links.h"
#include "head_slow.h"

#include "flora_head_board.h"
#include "flora_i2cmux.h"
#include "flora_store.h"

static const char *TAG = "glowica";

static i2c_master_bus_handle_t s_bus;

/* Opisy portow muxa MUSZA zyc tak dlugo jak czujniki - sterownik VEML
 * trzyma do nich wskaznik, nie kopie. */
static flora_i2cmux_port_t     s_sun_port[FLORA_HEAD_SUN_COUNT];
static flora_veml7700_handle_t s_sun[FLORA_HEAD_SUN_COUNT];

static esp_err_t bus_setup(void)
{
    i2c_master_bus_config_t cfg = {
        .i2c_port          = I2C_NUM_0,
        .sda_io_num        = FLORA_HEAD_I2C_SDA,
        .scl_io_num        = FLORA_HEAD_I2C_SCL,
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    return i2c_new_master_bus(&cfg, &s_bus);
}

static void start_task(void *arg)
{
    /* 1. Lacze na petli zwrotnej - nie wymaga zadnego sprzetu poza plytka,
     *    wiec przechodzi niezaleznie od stanu czujnikow. */
    bringup_link_loopback();

    /* 2. Lacza docelowe: w dol do bryly, w gore do modulu znacznikow. */
    if (head_links_init() != ESP_OK) {
        ESP_LOGE(TAG, "lacza nie wystartowaly - przerywam");
        vTaskDelete(NULL);
    }
    head_cmd_register(head_link_down());

    /* 3. Test rozruchowy czujnikow. Inicjalizuje je i zwraca uchwyty,
     *    wiec petla dostaje dokladnie te, ktore test sprawdzil. */
    bringup_head_ctx_t ctx = {
        .bus      = s_bus,
        .sun_port = s_sun_port,
        .sun      = s_sun,
    };
    bringup_head_run(&ctx);

    /* 4. Zadania robocze. Brak czujnika nie blokuje startu - fuzja czeka
     *    w INIT, a czujniki wolne maja wlasne stany zdrowia i probuja
     *    ponownie co 10 s, wiec czujnik wpiety po starcie tez sie znajdzie. */
    ESP_ERROR_CHECK(head_fusion_start(bringup_head_imu(), head_link_down()));

    head_slow_cfg_t slow = {
        .mux  = bringup_head_mux(),
        .bme  = bringup_head_bme(),
        .down = head_link_down(),
    };
    for (int i = 0; i < FLORA_HEAD_SUN_COUNT; i++) {
        slow.sun[i] = bringup_head_sun(i);
    }
    ESP_ERROR_CHECK(head_slow_start(&slow));

    ESP_LOGI(TAG, "petla sterowania dziala");
    vTaskDelete(NULL);
}

void app_main(void)
{
    ESP_LOGI(TAG, "doniczka-robot, glowica masztu");

    /* NVS jako pierwsze: offset zyroskopu jest w nim zapamietany, zeby
     * glowica nie musiala kalibrowac sie przy kazdym wybudzeniu - bryla
     * odcina jej zasilanie i nie da sie zagwarantowac bezruchu. */
    ESP_ERROR_CHECK(flora_store_init());

    esp_err_t err = bus_setup();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "magistrala I2C nie wystartowala: %s", esp_err_to_name(err));
        return;
    }

    /* Start w osobnym zadaniu, bo rozruch blokuje na kilka sekund,
     * a app_main nie powinno tyle trzymac. */
    xTaskCreate(start_task, "start", 5120, NULL, 6, NULL);
}
