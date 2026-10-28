/**
 * @file bringup_hydro.c
 * @brief Test rozruchowy hydrauliki: pompka i czujniki temperatury.
 *
 * Wydzielone z main.c po pomyslnym tescie na sprzecie (2026-10-15).
 * Uruchomienie: bringup_hydro_run(ina) z app_main().
 *
 * UWAGA: czujnik poziomu wody niepodlaczony, wiec ochrona przed praca
 * na sucho jest WYLACZONA. Upewnij sie, ze jest woda albo odlacz pompke.
 */
#include <inttypes.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"

#include "bringup_hydro.h"
#include "flora_board.h"
#include "flora_ina226.h"
#include "flora_pump.h"
#include "flora_temp.h"
#include "flora_level.h"

static const char *TAG = "hydro";

static flora_ina226_handle_t s_ina;
static bool s_temp_ready;
static bool s_pump_ready;
static flora_level_handle_t s_level;

/* --- Limity pompki na czas testu ---
 * Znacznie ciasniejsze niz docelowe (30 s / 60 s / 10 min), zeby przy braku
 * czujnika poziomu zadna pomylka nie zdazyla narobic szkody, a sam test nie
 * trwal minutami przez dluga przerwe miedzy dawkami. */
#define TEST_PUMP_MAX_RUN_MS   2000
#define TEST_PUMP_INTERVAL_MS  3000
#define TEST_PUMP_DAILY_MS    30000
#define TEST_PUMP_DOSE_MS      1000

static flora_ina226_handle_t   s_ina;
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

    /* Czujnik poziomu przez flora_level, nie przez wbudowana kontrole GPIO
     * w flora_pump. Powod: w jadacym robocie woda chlupie i surowy odczyt
     * pinu przelaczalby sie kilka razy na sekunde. flora_level probkuje
     * i uspokaja, wiec pompka dostaje stan stabilny. */
    flora_level_cfg_t lvl = FLORA_LEVEL_DEFAULT_CFG();
    lvl.gpio = FLORA_PIN_WATER_LEVEL;
    bool have_level = (flora_level_init(&lvl, &s_level) == ESP_OK);
    if (have_level) {
        ESP_LOGI(TAG, "  czujnik poziomu: surowo %s, po uspokojeniu %s",
                 flora_level_raw(s_level) ? "woda" : "sucho",
                 flora_level_water_present(s_level) ? "woda" : "sucho");
        ESP_LOGW(TAG, "  SPRAWDZ POLARYZACJE: zanurz czujnik i potwierdz, ze surowy");
        ESP_LOGW(TAG, "  odczyt zmienia sie na \"woda\". XKC-Y25 bywa NPN albo PNP,");
        ESP_LOGW(TAG, "  a zla flaga oznacza pompke na sucho przy pustym zbiorniku.");
    } else {
        ESP_LOGW(TAG, "  czujnik poziomu niedostepny - brak ochrony przed sucho");
    }

    flora_pump_cfg_t cfg = FLORA_PUMP_DEFAULT_CFG();
    cfg.pump_gpio = FLORA_PIN_PUMP;
    /* Wbudowana kontrola po GPIO zostaje wylaczona - stan dostarcza
     * flora_level przez callback, juz uspokojony. */
    cfg.level_gpio       = GPIO_NUM_NC;
    if (have_level) {
        cfg.water_cb  = flora_level_pump_cb;
        cfg.water_ctx = s_level;
    }
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
    if (flora_pump_water_available()) {
        ESP_LOGW(TAG, "  Czujnik zglasza wode. Pompka ruszy.");
    } else {
        ESP_LOGW(TAG, "  Czujnik NIE zglasza wody - dozowanie zostanie odmowione.");
    }
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

void bringup_hydro_run(flora_ina226_handle_t ina)
{
    s_ina = ina;
    test_temperature();
    test_pump();

    if (s_temp_ready) {
        float c = 0.0f;
        if (flora_temp_read(0, &c) == ESP_OK) {
            ESP_LOGI(TAG, "temperatura po tescie: %.2f C", c);
        }
    }
    (void)s_pump_ready;
}
