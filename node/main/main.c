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
#include "driver/gpio.h"
#include "esp_rom_sys.h"

#include "bringup_cam.h"
#include "bringup_head.h"
#include "bringup_link.h"
#include "head_cmd.h"
#include "head_fusion.h"
#include "head_links.h"
#include "head_net.h"
#include "head_obstacles.h"
#include "head_slow.h"

#include "flora_head_board.h"
#include "flora_i2cmux.h"
#include "flora_store.h"

static const char *TAG = "glowica";

/* TYMCZASOWE: test czujnikow na sucho bez podpietej kamery. Wroc na 0. */
#define TEST_BEZ_KAMERY 1

static i2c_master_bus_handle_t s_bus;

/* Opisy portow muxa MUSZA zyc tak dlugo jak czujniki - sterownik VEML
 * trzyma do nich wskaznik, nie kopie. */
static flora_i2cmux_port_t     s_sun_port[FLORA_HEAD_SUN_COUNT];
static flora_veml7700_handle_t s_sun[FLORA_HEAD_SUN_COUNT];

/* Stan linii przed startem sterownika I2C. Timeout na kazdym adresie
 * znaczy, ze ktoras linia wisi nisko - tu widac ktora. Niski SDA przy
 * wysokim SCL to zwykle uklad zawieszony w polowie transakcji (ESP sie
 * zresetowal, czujnik nie), wiec probujemy standardowego odblokowania:
 * 9 impulsow zegara i STOP. */
static void bus_line_check(void)
{
    const gpio_num_t sda = FLORA_HEAD_I2C_SDA, scl = FLORA_HEAD_I2C_SCL;
    gpio_config_t in = {
        .pin_bit_mask = (1ULL << sda) | (1ULL << scl),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&in);
    esp_rom_delay_us(100);
    int l_sda = gpio_get_level(sda), l_scl = gpio_get_level(scl);
    ESP_LOGI(TAG, "linie I2C w spoczynku: SDA(%d)=%d SCL(%d)=%d (oczekiwane 1/1)",
             sda, l_sda, scl, l_scl);

    if (l_scl == 0) {
        ESP_LOGE(TAG, "  SCL trzymany nisko - zwarcie do masy albo niezasilony uklad;"
                      " odblokowanie zegarem niemozliwe");
    } else if (l_sda == 0) {
        ESP_LOGW(TAG, "  SDA trzymany nisko - probuje 9 impulsow SCL");
        gpio_set_direction(scl, GPIO_MODE_INPUT_OUTPUT_OD);
        for (int i = 0; i < 9 && gpio_get_level(sda) == 0; i++) {
            gpio_set_level(scl, 0);
            esp_rom_delay_us(10);
            gpio_set_level(scl, 1);
            esp_rom_delay_us(10);
        }
        /* STOP: SDA w gore przy wysokim SCL */
        gpio_set_direction(sda, GPIO_MODE_INPUT_OUTPUT_OD);
        gpio_set_level(sda, 0);
        esp_rom_delay_us(10);
        gpio_set_level(sda, 1);
        esp_rom_delay_us(10);
        gpio_set_direction(sda, GPIO_MODE_INPUT);
        gpio_set_direction(scl, GPIO_MODE_INPUT);
        ESP_LOGW(TAG, "  po odblokowaniu: SDA=%d SCL=%d",
                 gpio_get_level(sda), gpio_get_level(scl));
    }
    gpio_reset_pin(sda);
    gpio_reset_pin(scl);
}

static esp_err_t bus_setup(void)
{
    bus_line_check();

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

    /* 5. Kamera. W pracy w terenie bez sieci: obrazu nikt nie oglada, a
     *    radio na solarnej glowicy to najwiekszy pojedynczy pobor. Podglad
     *    przez przegladarke wlacza sie flaga w flora_head_board.h, gdy
     *    trzeba wycelowac kamere, zebrac klatki albo ustawic jej wysokosc.
     *    Blad nie przerywa startu - reszta glowicy dziala bez obrazu. */
    if (TEST_BEZ_KAMERY) {
        ESP_LOGW(TAG, "TEST_BEZ_KAMERY: kamera i wizja wylaczone");
    } else {
    if (FLORA_HEAD_CAM_NET_DEFAULT) {
        bringup_cam_preview();
    } else {
        bringup_cam_offline();
    }

    /* 6. Wykrywanie przeszkod. Rusza tylko w trybie surowym kamery, bo w
     *    trybie podgladu JPEG nie ma czego przetwarzac. Brak wizji nie
     *    przerywa startu - glowica dziala bez niej. */
    head_obstacles_start(head_link_down());
    }

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
    /* Zaraz po NVS: od tej chwili logi czekaja w kolejce na wyslanie przez
     * MQTT (takze te z rozruchu czujnikow nizej), a wersja po OTA odlicza
     * czas do potwierdzenia. */
    head_net_early();

    esp_err_t err = bus_setup();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "magistrala I2C nie wystartowala: %s", esp_err_to_name(err));
        return;
    }

    /* Siec w tle, niezaleznie od reszty: brak WiFi nie zatrzymuje czujnikow,
     * a zepsute lacze z bryla nie odcina logow. */
    if (head_net_start() != ESP_OK) {
        ESP_LOGW(TAG, "siec glowicy nie wystartowala");
    }

    /* Start w osobnym zadaniu, bo rozruch blokuje na kilka sekund,
     * a app_main nie powinno tyle trzymac. Stos z zapasem na podniesienie
     * WiFi i kamery w kroku 5 - samo esp_wifi_init zjada ponad 3 kB. */
    xTaskCreate(start_task, "start", 6144, NULL, 6, NULL);
}
