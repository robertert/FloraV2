/**
 * @file head_net.c
 * @brief Siec glowicy - patrz head_net.h.
 *
 * Tematy (baza flora/robot1, ten sam login co bryla, inny client_id):
 *   headstatus     "online"/"offline", retain + testament
 *   head/log       logi W i E (i wiecej na zadanie), JSON jak tele/log bryly
 *   head/boot      wersja, powod restartu, licznik startow (retain)
 *   head/ota       stan aktualizacji
 *   headcmd/<x>    komendy, odpowiedz w headack/<x> (head_remote.c)
 *
 * Tematy head/... trafiaja do Influxa bez zmian w Telegrafie (subskrybuje flora/+/head/#),
 * z tagiem source=head. Status i komendy sa poza head/, bo to nie JSON.
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "head_net.h"
#include "head_remote.h"
#include "flora_diag.h"
#include "flora_head_board.h"
#include "flora_net.h"
#include "flora_ota.h"

static const char *TAG = "siec";

void head_net_early(void)
{
    static const flora_diag_cfg_t diag = {
        .log_topic  = "head/log",
        .boot_topic = "head/boot",
    };
    if (flora_diag_init(&diag) != ESP_OK) {
        ESP_LOGW(TAG, "diagnostyka zdalna niedostepna - logi tylko przez USB");
    }

    /* Glowica nie ma napedu ani pompki, wiec aktualizacji nic nie blokuje. */
    static const flora_ota_cfg_t ota = {
        .version_prefix = "head-v",
        .dir            = "head",
        .topic          = "head/ota",
    };
    flora_ota_init(&ota);
}

static void on_mqtt(bool connected)
{
    if (connected) {
        flora_diag_publish_boot();
    }
}

static void net_task(void *arg)
{
    /* Jak w bryle: najpierw lancuch prob polaczenia, potem subskrypcja
     * komend. Komponent MQTT odtwarza ja sam po kazdym ponownym polaczeniu,
     * wiec dziala takze wtedy, gdy broker pojawi sie dopiero pozniej. */
    static const flora_net_cfg_t cfg = {
        .client_id    = "flora-head",
        .status_topic = "headstatus",
        .on_mqtt      = on_mqtt,
    };
    esp_err_t r = flora_net_connect(&cfg);
    if (r != ESP_OK) {
        ESP_LOGW(TAG, "siec glowicy nie wstala (%s) - ponawia w tle", esp_err_to_name(r));
    }
    if (head_remote_start() != ESP_OK) {
        ESP_LOGE(TAG, "komendy zdalne glowicy niedostepne");
    }
    vTaskDelete(NULL);
}

esp_err_t head_net_start(void)
{
    if (!FLORA_HEAD_NET_DEFAULT) {
        ESP_LOGI(TAG, "siec glowicy wylaczona (FLORA_HEAD_NET_DEFAULT = 0)");
        return ESP_OK;
    }
    /* Stos jak w bryle: esp_wifi_init i start klienta MQTT z TLS. */
    return xTaskCreate(net_task, "head_net", 6144, NULL, 4, NULL) == pdPASS
           ? ESP_OK : ESP_ERR_NO_MEM;
}
