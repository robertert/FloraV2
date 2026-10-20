/**
 * @file bringup_sensors.c
 * @brief Test rozruchowy czujnikow: wilgotnosc gleby i dalmierz ultradzwiekowy.
 *
 * Wydzielone z main.c po udanym tescie i kalibracji gleby (2026-10-19).
 * Uruchomienie: bringup_sensors_run() z app_main().
 *
 * Podlaczony jest jeden dalmierz, echo na GPIO14. Pozostale wejscia
 * (GPIO21, GPIO47) sa w flora_board.h - wystarczy dopisac do tablicy.
 */
#include <inttypes.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"

#include "bringup_sensors.h"
#include "flora_board.h"
#include "flora_soil.h"
#include "flora_sonar.h"
#include "flora_temp.h"

static const char *TAG = "czujniki";

static bool s_soil_ready;
static bool s_sonar_ready;
static bool s_temp_ready;

static void test_soil(void)
{
    ESP_LOGI(TAG, "=== WILGOTNOSC GLEBY ===");

    flora_soil_cfg_t cfg = FLORA_SOIL_DEFAULT_CFG();
    cfg.gpio    = FLORA_PIN_SOIL_ADC;
    cfg.raw_dry = FLORA_SOIL_RAW_DRY;
    cfg.raw_wet = FLORA_SOIL_RAW_WET;

    esp_err_t err = flora_soil_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "  inicjalizacja nie powiodla sie: %s", esp_err_to_name(err));
        return;
    }
    s_soil_ready = true;

    /* Kilka odczytow pod rzad: rozrzut pokazuje, ile szumu jest na wejsciu.
     * Przy czujniku pojemnosciowym w glebie rozrzut powyzej ~50 jednostek
     * zwykle oznacza zly styk albo zaklocenia od silnikow. */
    ESP_LOGI(TAG, "  seria odczytow (sprawdzamy stabilnosc):");
    int min_raw = INT32_MAX, max_raw = 0;
    for (int i = 0; i < 5; i++) {
        int raw = 0;
        if (flora_soil_read_raw(&raw) != ESP_OK) {
            ESP_LOGE(TAG, "    odczyt %d nie powiodl sie", i);
            continue;
        }
        if (raw < min_raw) min_raw = raw;
        if (raw > max_raw) max_raw = raw;

        int mv = -1;
        if (flora_soil_has_voltage_calibration()) {
            flora_soil_read_mv(&mv);
        }
        ESP_LOGI(TAG, "    [%d] surowo %4d%s", i, raw,
                 mv >= 0 ? "" : "  (brak kalibracji mV)");
        if (mv >= 0) {
            ESP_LOGI(TAG, "         %d mV", mv);
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    ESP_LOGI(TAG, "  rozrzut: %d jednostek (min %d, max %d)",
             max_raw - min_raw, min_raw, max_raw);
    if (max_raw - min_raw > 50) {
        ESP_LOGW(TAG, "  duzy rozrzut - sprawdz styki i mase czujnika");
    }

    /* Procenty wymagaja dwupunktowej kalibracji, ktorej jeszcze nie ma. */
    float percent = 0.0f;
    if (flora_soil_read_percent(&percent) == ESP_OK) {
        ESP_LOGI(TAG, "  wilgotnosc: %.1f%%", percent);
        ESP_LOGI(TAG, "  (skala: %d = sucho, %d = zanurzony w wodzie)",
                 FLORA_SOIL_RAW_DRY, FLORA_SOIL_RAW_WET);
    } else {
        ESP_LOGW(TAG, "  procenty niedostepne - sprawdz kalibracje w flora_board.h");
    }
}

/* ================= DALMIERZ ULTRADZWIEKOWY ================= */

static void test_sonar(void)
{
    ESP_LOGI(TAG, "=== DALMIERZ ULTRADZWIEKOWY ===");

    flora_sonar_cfg_t cfg = FLORA_SONAR_DEFAULT_CFG();
    cfg.trig_gpio    = FLORA_PIN_US_TRIG;
    cfg.echo_gpio[0] = FLORA_PIN_US_ECHO_1;   /* GPIO14, jedyny podlaczony */
    cfg.count        = 1;

    /* Jesli mamy DS18B20, bierzemy z niego temperature do korekty predkosci
     * dzwieku - to realna poprawka, miedzy 0 a 30 C roznica siega 5%. */
    if (s_temp_ready) {
        float c = 0.0f;
        if (flora_temp_read(0, &c) == ESP_OK && c > -40.0f && c < 80.0f) {
            cfg.temperature_c = c;
            ESP_LOGI(TAG, "  korekta temperaturowa z DS18B20: %.1f C", c);
        }
    }

    esp_err_t err = flora_sonar_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "  inicjalizacja nie powiodla sie: %s", esp_err_to_name(err));
        return;
    }
    s_sonar_ready = true;

    ESP_LOGI(TAG, "  zakres martwy czujnika to ok. 20-25 cm - blizej nie zmierzy");
    ESP_LOGI(TAG, "  seria pojedynczych pomiarow:");

    int ok = 0, timeout = 0, rejected = 0;
    for (int i = 0; i < 8; i++) {
        float mm = 0.0f;
        esp_err_t r = flora_sonar_measure(0, &mm);
        if (r == ESP_OK) {
            ESP_LOGI(TAG, "    [%d] %.0f mm (%.1f cm)", i, mm, mm / 10.0f);
            ok++;
        } else if (r == ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "    [%d] brak echa", i);
            timeout++;
        } else {
            ESP_LOGW(TAG, "    [%d] odrzucony (poza zakresem wiarygodnosci)", i);
            rejected++;
        }
    }

    ESP_LOGI(TAG, "  wynik: %d udanych, %d bez echa, %d odrzuconych", ok, timeout, rejected);
    if (ok == 0) {
        ESP_LOGE(TAG, "  zaden pomiar sie nie udal. Sprawdz w tej kolejnosci:");
        ESP_LOGE(TAG, "    zasilanie czujnika 5V, dzielnik na ECHO (5V -> 3,3V),");
        ESP_LOGE(TAG, "    TRIG na GPIO%d, ECHO na GPIO%d, wspolna masa",
                 FLORA_PIN_US_TRIG, FLORA_PIN_US_ECHO_1);
        return;
    }

    float median = 0.0f;
    if (flora_sonar_measure_median(0, 5, &median) == ESP_OK) {
        ESP_LOGI(TAG, "  mediana z 5 pomiarow: %.0f mm (%.1f cm)", median, median / 10.0f);
    }
}

/* ================= PRZEBIEG ================= */

void bringup_sensors_run(bool live_preview)
{
    /* DS18B20 tylko po to, zeby skorygowac predkosc dzwieku. */
    flora_temp_cfg_t tcfg = FLORA_TEMP_DEFAULT_CFG();
    tcfg.gpio = FLORA_PIN_DS18B20;
    s_temp_ready = (flora_temp_init(&tcfg) == ESP_OK);

    test_soil();
    test_sonar();

    if (!live_preview) {
        return;
    }

    ESP_LOGI(TAG, "========================================================");
    ESP_LOGI(TAG, " Podglad na zywo. Przysun reke do czujnika i patrz.");
    ESP_LOGI(TAG, "========================================================");

    while (1) {
        char soil_txt[32] = "----";
        if (s_soil_ready) {
            int raw = 0;
            if (flora_soil_read_raw(&raw) == ESP_OK) {
                float p = 0.0f;
                if (flora_soil_read_percent(&p) == ESP_OK) {
                    snprintf(soil_txt, sizeof(soil_txt), "%4d (%.0f%%)", raw, p);
                } else {
                    snprintf(soil_txt, sizeof(soil_txt), "%4d", raw);
                }
            }
        }

        char dist_txt[32] = "brak echa";
        if (s_sonar_ready) {
            float mm = 0.0f;
            if (flora_sonar_measure_median(0, 3, &mm) == ESP_OK) {
                snprintf(dist_txt, sizeof(dist_txt), "%.1f cm", mm / 10.0f);
            }
        }

        ESP_LOGI(TAG, "gleba %-14s | odleglosc %s", soil_txt, dist_txt);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
