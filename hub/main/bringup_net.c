/**
 * @file bringup_net.c
 * @brief Siec bryly: nakladka na components/flora_net + test rozruchowy.
 *
 * Lancuch prob (NVS -> serwer z firmware -> broker na Macu) jest w flora_net,
 * wspolny z glowica. Dane dostepowe czytane z NVS, a stale z flora_board.h
 * sluza jako domyslne.
 */
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"

#include "bringup_net.h"
#include "flora_board.h"
#include "flora_mqtt.h"
#include "flora_net.h"

static const char *TAG = "siec";

static void on_command(const char *topic, size_t topic_len,
                       const char *data, size_t data_len, void *ctx)
{
    ESP_LOGI(TAG, "  komenda z brokera: %.*s = %.*s",
             (int)topic_len, topic, (int)data_len, data);
}

esp_err_t bringup_net_connect(void)
{
    const flora_net_cfg_t cfg = { .client_id = "flora-hub", .status_topic = "status" };
    return flora_net_connect(&cfg);
}

void bringup_net_run(void)
{
    ESP_LOGI(TAG, "=== SIEC ===");

    if (bringup_net_connect() != ESP_OK) {
        return;
    }

    /* Subskrypcja zaklada sie nawet przed polaczeniem - komponent odtworzy
     * ja po nawiazaniu i po kazdym ponownym polaczeniu. */
    flora_mqtt_subscribe("cmd/#", 1, on_command, NULL);

    /* --- probna telemetria --- */
    ESP_LOGI(TAG, "  publikuje probna telemetrie:");
    flora_mqtt_publish_fmt("telemetry/uptime", 0, false, "%" PRId64,
                           esp_timer_get_time() / 1000000);
    flora_mqtt_publish_fmt("telemetry/heap", 0, false, "%" PRIu32,
                           (uint32_t)esp_get_free_heap_size());
    flora_mqtt_publish_str("telemetry/hello", "jednostka dolna dziala", 0, false);
    ESP_LOGI(TAG, "    %s/telemetry/*", FLORA_MQTT_BASE_TOPIC);
    ESP_LOGI(TAG, "  podsluchaj: mosquitto_sub -h <broker> -t '%s/#' -v",
             FLORA_MQTT_BASE_TOPIC);
}
