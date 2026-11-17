/**
 * @file hub_auto.c
 * @brief Start funkcji autonomicznych, opis w hub_auto.h.
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "hub_auto.h"
#include "hub_mission.h"
#include "hub_pilot.h"
#include "hub_power.h"
#include "hub_safety.h"
#include "hub_water.h"

static const char *TAG = "auto";

static void auto_task(void *arg)
{
    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(1000));
        hub_power_tick();
        hub_water_tick();
        hub_mission_tick();
    }
}

esp_err_t hub_auto_start(void)
{
    /* Kazdy modul osobno: brak jednego nie zabiera reszty, a wylaczone
     * i tak nic nie robia. */
    if (hub_safety_start() != ESP_OK) {
        ESP_LOGE(TAG, "straznik nie wystartowal - z guard_en = 1 naped bedzie stal");
    }
    if (hub_pilot_start() != ESP_OK) {
        ESP_LOGE(TAG, "pilot nie wystartowal");
    }
    if (hub_power_start() != ESP_OK) {
        ESP_LOGE(TAG, "modul energii nie wystartowal");
    }
    hub_water_start();
    hub_mission_start();
    /* Stos: kroki formatuja JSON (bufor 256 B w flora_mqtt) i licza
     * pozycje slonca w double. */
    if (xTaskCreate(auto_task, "auto", 5120, NULL, 4, NULL) != pdPASS) {
        ESP_LOGE(TAG, "zadanie autonomii nie wystartowalo");
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
