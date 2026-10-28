/**
 * @file bringup_power.c
 * @brief Weryfikacja toru pomiarowego INA226.
 *
 * Powod istnienia: poprzedni egzemplarz czytal napiecie szyny poprawnie
 * (12,66 V), ale prad stal na 2,2 mA niezaleznie od obciazenia - przy
 * kreacych sie silnikach multimetr pokazywal na boczniku 4,5 mV, a uklad
 * widzial 0,022 mV. Przetwornik zyl, tylko wejscie roznicowe nie docieralo
 * do bocznika.
 *
 * Dlatego sam odczyt "jakiejs" wartosci nic nie dowodzi. Test sprawdza,
 * czy prad ROSNIE WRAZ Z MOCA silnikow - tego stary uklad nie potrafil.
 */
#include <inttypes.h>
#include <math.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"

#include "bringup_power.h"
#include "flora_board.h"
#include "flora_ina226.h"
#include "flora_motors.h"

static const char *TAG = "zasilanie";

/** Usredniony odczyt - pojedyncza probka przy PWM 20 kHz skacze. */
static bool read_avg(flora_ina226_handle_t ina, int n, float *v, float *a, float *mv)
{
    float sv = 0, sa = 0, smv = 0;
    int ok = 0;
    for (int i = 0; i < n; i++) {
        flora_ina226_reading_t r;
        if (flora_ina226_read(ina, &r) == ESP_OK) {
            sv += r.bus_voltage_v;
            sa += r.current_a;
            smv += r.shunt_voltage_mv;
            ok++;
        }
        vTaskDelay(pdMS_TO_TICKS(40));
    }
    if (ok == 0) {
        return false;
    }
    *v = sv / ok; *a = sa / ok; *mv = smv / ok;
    return true;
}

void bringup_power_run(i2c_master_bus_handle_t bus)
{
    ESP_LOGI(TAG, "=== WERYFIKACJA INA226 ===");

    /* Adres zalezy od zworek A0/A1 - szukamy, zamiast zakladac. */
    flora_ina226_handle_t ina = NULL;
    flora_ina226_cfg_t cfg = FLORA_INA226_DEFAULT_CFG();
    cfg.shunt_milliohm = FLORA_SHUNT_MILLIOHM;

    for (uint8_t addr = 0x40; addr <= 0x4F; addr++) {
        if (i2c_master_probe(bus, addr, 50) != ESP_OK) {
            continue;
        }
        cfg.i2c_address = addr;
        if (flora_ina226_init(bus, &cfg, &ina) == ESP_OK) {
            ESP_LOGI(TAG, "  znaleziony pod 0x%02X", addr);
            break;
        }
    }
    if (!ina) {
        ESP_LOGE(TAG, "  BLAD: nie znaleziono INA226 na magistrali");
        ESP_LOGE(TAG, "  sprawdz zasilanie ukladu, SDA/SCL i podciagniecia");
        return;
    }

    flora_ina226_dump(ina);

    /* --- 1. Spoczynek --- */
    float v0, a0, mv0;
    if (!read_avg(ina, 10, &v0, &a0, &mv0)) {
        ESP_LOGE(TAG, "  BLAD: odczyt nie dziala");
        return;
    }
    ESP_LOGI(TAG, "  spoczynek: %.2f V, %.3f A, bocznik %+.1f uV", v0, a0, mv0 * 1000.0f);

    if (v0 < 6.0f) {
        ESP_LOGE(TAG, "  BLAD: szyna ma %.2f V - wlacz zasilanie 12 V", v0);
        ESP_LOGE(TAG, "  bez niego test obciazeniem nic nie pokaze");
        return;
    }

    /* --- 2. Prad pod rosnacym obciazeniem ---
     * Sedno testu. Stary uklad przechodzil wszystko powyzej, a tu
     * pokazywal plaska linie. */
    /* Silniki inicjalizujemy tutaj, zeby test byl samodzielny. Straznika
     * utyku NIE wlaczamy - przy kolach w powietrzu albo zablokowanych
     * odcialby zasilanie w polowie pomiaru. */
    flora_motors_cfg_t mcfg = FLORA_MOTORS_DEFAULT_CFG();
    mcfg.left  = (flora_motor_cfg_t){ .pwm_gpio = FLORA_PIN_MOTOR_L_PWM,
                                      .dir_gpio = FLORA_PIN_MOTOR_L_DIR,
                                      .invert   = false };
    mcfg.right = (flora_motor_cfg_t){ .pwm_gpio = FLORA_PIN_MOTOR_R_PWM,
                                      .dir_gpio = FLORA_PIN_MOTOR_R_DIR,
                                      .invert   = true };
    esp_err_t mret = flora_motors_init(&mcfg);
    if (mret != ESP_OK && mret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "  nie udalo sie uruchomic napedu: %s", esp_err_to_name(mret));
        return;
    }

    ESP_LOGW(TAG, "  ----------------------------------------------");
    ESP_LOGW(TAG, "  Teraz rusza silniki na 3 poziomach mocy.");
    ESP_LOGW(TAG, "  Kola moga sie krecic - zapewnij miejsce. Start za 3 s.");
    ESP_LOGW(TAG, "  ----------------------------------------------");
    vTaskDelay(pdMS_TO_TICKS(3000));

    static const float levels[] = { 25.0f, 50.0f, 80.0f };
    float current[3], shunt[3];

    for (int i = 0; i < 3; i++) {
        flora_motors_set(levels[i], levels[i]);
        vTaskDelay(pdMS_TO_TICKS(400));        /* rozbieg */
        float v;
        read_avg(ina, 8, &v, &current[i], &shunt[i]);
        ESP_LOGI(TAG, "  moc %3.0f%%: %.2f V, %.3f A, bocznik %+.1f uV",
                 levels[i], v, current[i], shunt[i] * 1000.0f);
    }
    flora_motors_stop();
    vTaskDelay(pdMS_TO_TICKS(500));

    float v1, a1, mv1;
    read_avg(ina, 8, &v1, &a1, &mv1);
    ESP_LOGI(TAG, "  po zatrzymaniu: %.2f V, %.3f A", v1, a1);

    /* --- 3. Werdykt --- */
    ESP_LOGI(TAG, "  ----------------------------------------------");

    float delta = current[2] - a0;
    bool  grows = (current[0] < current[1]) && (current[1] < current[2]);

    /* Dwa silniki na 80% bez obciazenia to co najmniej kilkaset mA.
     * Prog 50 mA jest wyraznie ponizej, a zarazem daleko od 2 mA,
     * ktore pokazywal uszkodzony egzemplarz. */
    if (delta < 0.050f) {
        ESP_LOGE(TAG, "  [BLAD] prad nie reaguje na obciazenie");
        ESP_LOGE(TAG, "    spoczynek %.3f A -> pelna moc %.3f A, przyrost %.3f A", a0, current[2], delta);
        ESP_LOGE(TAG, "    napiecie czyta sie poprawnie, wiec uklad i magistrala dzialaja -");
        ESP_LOGE(TAG, "    problem jest w torze bocznika. Sprawdz omomierzem przy");
        ESP_LOGE(TAG, "    WYLACZONYM zasilaniu ciaglosc od zaciskow IN+/IN- do nozek");
        ESP_LOGE(TAG, "    ukladu. To samo zachowanie mial poprzedni egzemplarz.");
    } else if (!grows) {
        ESP_LOGW(TAG, "  [UWAGA] prad reaguje, ale nie rosnie monotonicznie:");
        ESP_LOGW(TAG, "    %.3f -> %.3f -> %.3f A", current[0], current[1], current[2]);
        ESP_LOGW(TAG, "    moze byc poprawne, jesli kolo buksowalo przy wiekszej mocy");
    } else {
        ESP_LOGI(TAG, "  [ OK ] prad rosnie z moca: %.3f -> %.3f -> %.3f A",
                 current[0], current[1], current[2]);
        ESP_LOGI(TAG, "    przyrost wzgledem spoczynku: %.3f A", delta);
        ESP_LOGI(TAG, "    TOR POMIAROWY DZIALA - to czego brakowalo w starym ukladzie");
    }

    /* Sufit sprzetowy warto znac zawczasu: INA226 mierzy na boczniku
     * najwyzej +-81,92 mV, czyli przy R010 maksymalnie 8,19 A. */
    if (fabsf(shunt[2]) > 70.0f) {
        ESP_LOGW(TAG, "  bocznik blisko sufitu (%.1f mV z 81,92) - przy wiekszym",
                 shunt[2]);
        ESP_LOGW(TAG, "  obciazeniu odczyt zacznie sie nasycac");
    }
    ESP_LOGI(TAG, "  ----------------------------------------------");
}
