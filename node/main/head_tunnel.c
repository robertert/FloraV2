/**
 * @file head_tunnel.c
 * @brief Most MQTT dla bryly bez WiFi, opis w head_tunnel.h.
 */
#include <inttypes.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/message_buffer.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "head_tunnel.h"
#include "flora_mqtt.h"
#include "flora_msgs.h"
#include "flora_tunnel.h"

static const char *TAG = "tunel";

#define UP_BUF_BYTES 4096
/* Wiadomosc w kolejce: [flagi qos/retain, 1 B][zlozona wiadomosc tunelu]. */
#define Q_HDR 1

static flora_link_handle_t   s_down;
static MessageBufferHandle_t s_up;
static flora_tunnel_rx_t     s_rxasm;      /**< pisze tylko zadanie lacza */
static volatile bool         s_seen;       /**< bryla nadaje przez tunel */
static volatile uint32_t     s_published, s_lost, s_cmds;

/* Zadanie lacza w dol: skladamy kawalki, publikacja w osobnym zadaniu,
 * bo klient MQTT potrafi blokowac na gniezdzie. */
static void on_up_chunk(uint8_t type, const void *payload, uint8_t len, void *ctx)
{
    static uint8_t q[Q_HDR + FLORA_TUNNEL_MAX_MSG];
    if (len != sizeof(flora_mqtt_chunk_t)) {
        return;
    }
    if (!flora_tunnel_rx_feed(&s_rxasm, payload)) {
        return;
    }
    q[0] = (uint8_t)((s_rxasm.qos & 3) | (s_rxasm.retain ? 0x80 : 0));
    memcpy(q + Q_HDR, s_rxasm.buf, s_rxasm.len);
    if (xMessageBufferSend(s_up, q, Q_HDR + s_rxasm.len, 0) == 0) {
        s_lost++;
    }
    s_seen = true;
}

/* Komenda z brokera dla bryly: temat PELNY, tak jak odbiera go klient. */
static void on_cmd(const char *topic, size_t topic_len, const char *data, size_t data_len,
                   void *ctx)
{
    static uint8_t id;
    uint8_t msg[FLORA_TUNNEL_MAX_MSG];
    char t[FLORA_TUNNEL_MAX_TOPIC + 1];
    if (topic_len > FLORA_TUNNEL_MAX_TOPIC) {
        return;
    }
    memcpy(t, topic, topic_len);
    t[topic_len] = 0;
    const size_t n = flora_tunnel_pack(t, data, data_len, msg, sizeof msg);
    if (!n) {
        return;
    }
    id++;
    const int chunks = flora_tunnel_chunk_count(n);
    for (int i = 0; i < chunks; i++) {
        flora_mqtt_chunk_t c;
        flora_tunnel_chunk(msg, n, id, i, 1, false, &c);
        flora_link_send(s_down, FLORA_MSG_MQTT_DOWN, &c, sizeof c);
    }
    s_cmds++;
}

void head_tunnel_link_state(bool up)
{
    if (!up && s_seen && flora_mqtt_is_connected()) {
        /* Za bryle - bez WiFi jej testament na brokerze nie istnieje. */
        flora_mqtt_publish_str("status", "offline", 1, true);
    }
}

static void tunnel_task(void *arg)
{
    static uint8_t q[Q_HDR + FLORA_TUNNEL_MAX_MSG];
    bool subscribed = false;
    uint32_t last_log = 0;

    for (;;) {
        const size_t n = xMessageBufferReceive(s_up, q, sizeof q, pdMS_TO_TICKS(500));

        if (s_seen && !subscribed) {
            /* Dopiero gdy klient MQTT juz istnieje - inaczej sprobujemy
             * przy nastepnej wiadomosci. */
            if (flora_mqtt_subscribe("cmd/#", 1, on_cmd, NULL) == ESP_OK) {
                subscribed = true;
                ESP_LOGW(TAG, "bryla nadaje przez tunel - przekazuje jej cmd/#");
            }
        }
        if (n > Q_HDR) {
            const char *t;
            size_t tl, dl;
            const uint8_t *d;
            if (flora_tunnel_unpack(q + Q_HDR, n - Q_HDR, &t, &tl, &d, &dl) &&
                flora_mqtt_is_connected()) {
                char topic[FLORA_TUNNEL_MAX_TOPIC + 1];
                memcpy(topic, t, tl);
                topic[tl] = 0;
                if (flora_mqtt_publish(topic, d, dl, q[0] & 3, (q[0] & 0x80) != 0) == ESP_OK) {
                    s_published++;
                } else {
                    s_lost++;
                }
            } else {
                s_lost++;
            }
        }

        const uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (s_seen && now - last_log >= 60000) {
            last_log = now;
            ESP_LOGI(TAG, "tunel bryly: wystawiono %" PRIu32 ", zgubiono %" PRIu32
                          " (+%" PRIu32 " niepelnych), komend w dol %" PRIu32,
                     s_published, s_lost, s_rxasm.dropped, s_cmds);
        }
    }
}

esp_err_t head_tunnel_start(flora_link_handle_t down)
{
    if (!down) {
        return ESP_ERR_INVALID_ARG;
    }
    s_down = down;
    s_up = xMessageBufferCreate(UP_BUF_BYTES);
    if (!s_up) {
        return ESP_ERR_NO_MEM;
    }
    flora_tunnel_rx_init(&s_rxasm);
    flora_link_on(down, FLORA_MSG_MQTT_UP, on_up_chunk, NULL);
    /* Stos: bufor tematu i publikacja z TLS pod spodem. */
    if (xTaskCreate(tunnel_task, "tunel", 4096, NULL, 4, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
