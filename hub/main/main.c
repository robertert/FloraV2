/**
 * Doniczka-robot, jednostka dolna (bryla).
 *
 * Test rozruchowy hydrauliki i pomiaru temperatury.
 *
 * Test napedu (silniki, enkodery, diagnostyka zbocz) zostal przeniesiony do
 * bringup_drive.c po zakonczeniu jego rozruchu - wywolaj bringup_drive_run(),
 * jesli bedziesz go potrzebowal ponownie.
 *
 * UWAGA: czujnik poziomu wody NIE jest jeszcze podlaczony, wiec ochrona
 * przed praca na sucho jest WYLACZONA. Przed uruchomieniem upewnij sie,
 * ze w zbiorniku jest woda albo odlacz pompke.
 */
#include <inttypes.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "driver/i2c_master.h"

#include "flora_board.h"
#include "flora_ina226.h"
#include "flora_pump.h"
#include "flora_temp.h"

static const char *TAG = "dol";

/* --- Limity pompki na czas testu ---
 * Znacznie ciasniejsze niz docelowe (30 s / 60 s / 10 min), zeby przy braku
 * czujnika poziomu zadna pomylka nie zdazyla narobic szkody, a sam test nie
 * trwal minutami przez dluga przerwe miedzy dawkami. */
#define TEST_PUMP_MAX_RUN_MS   2000
#define TEST_PUMP_INTERVAL_MS  3000
#define TEST_PUMP_DAILY_MS    30000
#define TEST_PUMP_DOSE_MS      1000

static flora_ina226_handle_t   s_ina;
static i2c_master_bus_handle_t s_i2c_bus;
static bool s_temp_ready;
static bool s_pump_ready;

/* ================= I2C / INA226 ================= */

static esp_err_t i2c_bus_setup(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port          = I2C_NUM_0,
        .sda_io_num        = FLORA_PIN_I2C_SDA,
        .scl_io_num        = FLORA_PIN_I2C_SCL,
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    return i2c_new_master_bus(&bus_cfg, &s_i2c_bus);
}

/** Adres INA226 zalezy od zworek A0/A1, wiec szukamy go zamiast zakladac. */
static void ina226_autodetect(void)
{
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
    ESP_LOGW(TAG, "nie znalazlem INA226 - test pojdzie bez pomiaru zasilania");
}

/** Napiecie szyny. Prad pomijamy: tor pomiarowy bocznika jest uszkodzony. */
static float bus_voltage(void)
{
    float v = 0.0f;
    if (s_ina) {
        flora_ina226_read_bus_voltage(s_ina, &v);
    }
    return v;
}

/* ================= TEMPERATURA ================= */

static void test_temperature(void)
{
    ESP_LOGI(TAG, "=== CZUJNIKI TEMPERATURY ===");

    flora_temp_cfg_t cfg = FLORA_TEMP_DEFAULT_CFG();
    cfg.gpio = FLORA_PIN_DS18B20;

    esp_err_t err = flora_temp_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "  brak czujnikow na GPIO%d (%s)", FLORA_PIN_DS18B20, esp_err_to_name(err));
        ESP_LOGE(TAG, "  sprawdz rezystor 4,7k miedzy linia danych a 3,3V, "
                      "zasilanie czujnika i wspolna mase");
        return;
    }
    s_temp_ready = true;

    int n = flora_temp_count();
    ESP_LOGI(TAG, "  znaleziono %d czujnik(ow), konwersja %" PRIu32 " ms",
             n, flora_temp_conversion_ms());
    for (int i = 0; i < n; i++) {
        ESP_LOGI(TAG, "    [%d] adres %016llX", i,
                 (unsigned long long)flora_temp_address(i));
    }

    /* Tryb prosty: jedno wywolanie, blokuje na czas konwersji. */
    int64_t t0 = esp_timer_get_time();
    float celsius = 0.0f;
    if (flora_temp_read(0, &celsius) == ESP_OK) {
        ESP_LOGI(TAG, "  odczyt blokujacy: %.3f C (zajal %" PRId64 " ms)",
                 celsius, (esp_timer_get_time() - t0) / 1000);
        /* 85,0 C to wartosc, ktora uklad trzyma w rejestrze po resecie. */
        if (celsius > 84.9f && celsius < 85.1f) {
            ESP_LOGW(TAG, "  dokladnie 85 C - to moze byc wartosc poresetowa, "
                          "a nie prawdziwy pomiar");
        }
    } else {
        ESP_LOGE(TAG, "  odczyt blokujacy nie powiodl sie");
    }

    /* Tryb dwuetapowy: tak wyglada uzycie w petli sterowania. */
    t0 = esp_timer_get_time();
    if (flora_temp_start_conversion() == ESP_OK) {
        int64_t trigger_us = esp_timer_get_time() - t0;
        vTaskDelay(pdMS_TO_TICKS(flora_temp_conversion_ms()));
        if (flora_temp_fetch(0, &celsius) == ESP_OK) {
            ESP_LOGI(TAG, "  odczyt dwuetapowy: %.3f C "
                          "(zlecenie zajelo %" PRId64 " us, reszte mozna robic cos innego)",
                     celsius, trigger_us);
        }
    }

    if (n > 1) {
        float all[FLORA_TEMP_MAX_SENSORS];
        int got = 0;
        if (flora_temp_read_all(all, FLORA_TEMP_MAX_SENSORS, &got) == ESP_OK) {
            for (int i = 0; i < got; i++) {
                ESP_LOGI(TAG, "  [%d] %.3f C", i, all[i]);
            }
        }
    }
}

/* ================= POMPKA ================= */

static void pump_done_cb(flora_pump_done_reason_t reason, uint32_t ran_ms, void *ctx)
{
    static const char *names[] = {
        [FLORA_PUMP_DONE_COMPLETED] = "zakonczone normalnie",
        [FLORA_PUMP_DONE_STOPPED]   = "zatrzymane recznie",
        [FLORA_PUMP_DONE_DRY]       = "przerwane: brak wody",
        [FLORA_PUMP_DONE_CLAMPED]   = "uciete limitem max_run_ms",
    };
    ESP_LOGI(TAG, "  -> callback: %s, pracowala %" PRIu32 " ms", names[reason], ran_ms);
}

/** Czeka na koniec dozowania, pokazujac po drodze pozostaly czas. */
static void wait_for_pump(void)
{
    while (flora_pump_is_running()) {
        ESP_LOGI(TAG, "     pracuje, zostalo %" PRIu32 " ms, szyna %.2f V",
                 flora_pump_remaining_ms(), bus_voltage());
        vTaskDelay(pdMS_TO_TICKS(400));
    }
}

static void test_pump(void)
{
    ESP_LOGI(TAG, "=== POMPKA ===");

    flora_pump_cfg_t cfg = FLORA_PUMP_DEFAULT_CFG();
    cfg.pump_gpio = FLORA_PIN_PUMP;
    /* Czujnik poziomu niepodlaczony. GPIO_NUM_NC wylacza kontrole calkowicie -
     * wpiecie tu GPIO10 bylo by gorsze niz jej brak, bo pin wiszacy w
     * powietrzu daje losowe stany i pompka ruszalaby albo blokowala sie
     * przypadkowo. */
    cfg.level_gpio       = GPIO_NUM_NC;
    cfg.max_run_ms       = TEST_PUMP_MAX_RUN_MS;
    cfg.min_interval_ms  = TEST_PUMP_INTERVAL_MS;
    cfg.daily_limit_ms   = TEST_PUMP_DAILY_MS;
    cfg.done_cb          = pump_done_cb;

    esp_err_t err = flora_pump_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "  inicjalizacja pompki nie powiodla sie: %s", esp_err_to_name(err));
        return;
    }
    s_pump_ready = true;

    ESP_LOGW(TAG, "  ----------------------------------------------");
    ESP_LOGW(TAG, "  BRAK CZUJNIKA POZIOMU - zero ochrony przed sucho");
    ESP_LOGW(TAG, "  Upewnij sie, ze jest woda albo odlacz pompke.");
    ESP_LOGW(TAG, "  Start za 5 s, lacznie popracuje okolo 3 s.");
    ESP_LOGW(TAG, "  ----------------------------------------------");
    for (int i = 5; i > 0; i--) {
        ESP_LOGW(TAG, "  %d...", i);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    /* 1. Zwykle dozowanie. */
    ESP_LOGI(TAG, "  [1] dawka %d ms", TEST_PUMP_DOSE_MS);
    if (flora_pump_run_ms(TEST_PUMP_DOSE_MS) == ESP_OK) {
        wait_for_pump();
    } else {
        ESP_LOGE(TAG, "      nie wystartowala");
    }

    /* 2. Natychmiastowa druga proba: ma sie odbic od min_interval_ms. */
    ESP_LOGI(TAG, "  [2] proba zaraz po poprzedniej (spodziewana odmowa)");
    err = flora_pump_run_ms(TEST_PUMP_DOSE_MS);
    if (err == ESP_ERR_INVALID_STATE) {
        ESP_LOGI(TAG, "      OK: odmowiono, przerwa miedzy dawkami dziala");
    } else {
        ESP_LOGE(TAG, "      BLAD: zabezpieczenie nie zadzialalo (%s)", esp_err_to_name(err));
        flora_pump_stop();
    }

    /* 3. Zadanie ponad limit: ma zostac przyciete do max_run_ms. */
    vTaskDelay(pdMS_TO_TICKS(TEST_PUMP_INTERVAL_MS + 200));
    ESP_LOGI(TAG, "  [3] zadanie 10000 ms przy limicie %d ms (spodziewane przyciecie)",
             TEST_PUMP_MAX_RUN_MS);
    int64_t t0 = esp_timer_get_time();
    if (flora_pump_run_ms(10000) == ESP_OK) {
        wait_for_pump();
        int64_t ran = (esp_timer_get_time() - t0) / 1000;
        if (ran < TEST_PUMP_MAX_RUN_MS + 500) {
            ESP_LOGI(TAG, "      OK: trwalo %" PRId64 " ms zamiast 10000", ran);
        } else {
            ESP_LOGE(TAG, "      BLAD: trwalo %" PRId64 " ms, limit nie zadzialal", ran);
        }
    }

    flora_pump_stats_t st;
    flora_pump_get_stats(&st);
    ESP_LOGI(TAG, "  statystyki: %" PRIu32 " dawek, lacznie %" PRIu32 " ms, "
                  "odmowy: %" PRIu32 " (przerwa) %" PRIu32 " (limit) %" PRIu32 " (sucho)",
             st.dose_count, st.total_run_ms,
             st.refused_interval, st.refused_limit, st.refused_dry);
}

/* ================= PRZEBIEG ================= */

static void bringup_task(void *arg)
{
    test_temperature();
    test_pump();

    ESP_LOGI(TAG, "========================================================");
    ESP_LOGI(TAG, " Podglad na zywo. Ctrl+] konczy monitor.");
    ESP_LOGI(TAG, "========================================================");

    while (1) {
        float celsius = FLORA_TEMP_INVALID;
        bool have_temp = s_temp_ready && flora_temp_read(0, &celsius) == ESP_OK;

        if (have_temp) {
            ESP_LOGI(TAG, "temperatura %.2f C | szyna %.2f V | pompka %s",
                     celsius, bus_voltage(),
                     flora_pump_is_running() ? "pracuje" : "stoi");
        } else {
            ESP_LOGI(TAG, "temperatura ---- | szyna %.2f V | pompka %s",
                     bus_voltage(), flora_pump_is_running() ? "pracuje" : "stoi");
        }

        /* flora_temp_read blokuje juz na czas konwersji, wiec dopelniamy
         * tylko reszte sekundy. */
        uint32_t conv = s_temp_ready ? flora_temp_conversion_ms() : 0;
        vTaskDelay(pdMS_TO_TICKS(conv < 1000 ? 1000 - conv : 100));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "doniczka-robot, jednostka dolna: hydraulika i temperatura");

    ESP_ERROR_CHECK(i2c_bus_setup());
    ina226_autodetect();

    xTaskCreate(bringup_task, "bringup", 4096, NULL, 5, NULL);
}
