/**
 * @file hub_tunnel.c
 * @brief MQTT bryly przez UART do glowicy, opis w hub_tunnel.h.
 */
#include <inttypes.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/message_buffer.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "hub_links.h"
#include "hub_tunnel.h"
#include "flora_board.h"
#include "flora_link.h"
#include "flora_mqtt.h"
#include "flora_msgs.h"
#include "flora_tunnel.h"

static const char *TAG = "tunel";

/** Przerwa po kazdym kawalku: ~72 B ramki co 10 ms = ~7 kB/s, czyli ok.
 *  60% lacza 115200 (TYMCZASOWO podniesione dla tunelu, hub_links.c). Bez tego bufor nadawczy UART-u sie zapycha i blokuje
 *  innych nadawcow (TIME_SYNC idzie z timera, wspolnego z petla napedu). */
#define PACE_MS        10
/** Zwykla telemetria: kazdy temat najwyzej raz na tyle ms. Bez tego glowica
 *  nie nadazala wystawiac na brokerze (TLS, zapis stawal na 10 s) i przez
 *  to spozniala komendy jazdy - naped stawal na dead-manie i ruszal. */
#define BULK_MIN_MS    1000
#define LIMIT_SLOTS    64
#define PRIO_BUF_BYTES 2048
#define BULK_BUF_BYTES 4096
#define RX_BUF_BYTES   1024

/* Wiadomosc w kolejce: [flagi qos/retain, 1 B][zlozona wiadomosc tunelu]. */
#define Q_HDR 1

static MessageBufferHandle_t s_prio, s_bulk, s_rx;
static SemaphoreHandle_t     s_wlock;
static flora_tunnel_rx_t     s_rxasm;      /**< pisze tylko zadanie lacza */
static volatile uint32_t     s_sent, s_dropped, s_received, s_throttled;

/** Czas ostatniej wysylki tematu (po skrocie nazwy). Pod s_wlock. */
static struct { uint32_t hash; int64_t t_us; } s_limit[LIMIT_SLOTS];

static uint32_t topic_hash(const char *t)
{
    uint32_t h = 5381;
    while (*t) {
        h = h * 33u + (uint8_t)*t++;
    }
    return h ? h : 1;
}

/** true, gdy temat byl wyslany za niedawno. Zaklada trzymana blokade. */
static bool too_soon(const char *subtopic)
{
    const uint32_t h = topic_hash(subtopic);
    const int64_t now = esp_timer_get_time();
    int free_slot = -1, oldest = 0;
    for (int i = 0; i < LIMIT_SLOTS; i++) {
        if (s_limit[i].hash == h) {
            if (now - s_limit[i].t_us < BULK_MIN_MS * 1000LL) {
                return true;
            }
            s_limit[i].t_us = now;
            return false;
        }
        if (!s_limit[i].hash && free_slot < 0) {
            free_slot = i;
        }
        if (s_limit[i].t_us < s_limit[oldest].t_us) {
            oldest = i;
        }
    }
    const int i = free_slot >= 0 ? free_slot : oldest;
    s_limit[i].hash = h;
    s_limit[i].t_us = now;
    return false;
}

/** Pierwsza kolejka dla tego, czego nie wolno zgubic przy natloku telemetrii. */
static bool is_priority(const char *subtopic, int qos, bool retain)
{
    return qos > 0 || retain || strncmp(subtopic, "ack/", 4) == 0 ||
           strcmp(subtopic, "tele/log") == 0 || strcmp(subtopic, "status") == 0;
}

/* Wolane z dowolnego zadania przez flora_mqtt_publish. BEZ logow (petla
 * przez diagnostyke). */
static esp_err_t proxy_pub(const char *subtopic, const void *data, size_t len,
                           int qos, bool retain)
{
    /* Bufor statyczny pod blokada, nie na stosie: publikuja tez zadania
     * z malym stosem (np. timery). Bufor wiadomosci FreeRTOS i tak zaklada
     * jednego pisarza naraz. */
    static uint8_t buf[Q_HDR + FLORA_TUNNEL_MAX_MSG];
    if (xSemaphoreTake(s_wlock, pdMS_TO_TICKS(5)) != pdTRUE) {
        s_dropped++;
        return ESP_ERR_TIMEOUT;
    }
    esp_err_t ret = ESP_OK;
    const bool prio = is_priority(subtopic, qos, retain);
    if (!prio && too_soon(subtopic)) {
        s_throttled++;
        xSemaphoreGive(s_wlock);
        return ESP_OK;          /* celowo pominiete, nie blad */
    }
    const size_t n = flora_tunnel_pack(subtopic, data, len, buf + Q_HDR, sizeof buf - Q_HDR);
    if (!n) {
        ret = ESP_ERR_INVALID_SIZE;
    } else {
        buf[0] = (uint8_t)((qos & 3) | (retain ? 0x80 : 0));
        MessageBufferHandle_t q = prio ? s_prio : s_bulk;
        if (xMessageBufferSend(q, buf, Q_HDR + n, 0) == 0) {
            ret = ESP_ERR_NO_MEM;
        }
    }
    xSemaphoreGive(s_wlock);
    if (ret != ESP_OK) {
        s_dropped++;
    }
    return ret;
}

static void send_msg(flora_link_handle_t link, const uint8_t *qmsg, size_t qlen)
{
    static uint8_t id;
    const int qos = qmsg[0] & 3;
    const bool retain = (qmsg[0] & 0x80) != 0;
    const uint8_t *msg = qmsg + Q_HDR;
    const size_t len = qlen - Q_HDR;
    const int chunks = flora_tunnel_chunk_count(len);
    id++;
    for (int i = 0; i < chunks; i++) {
        flora_mqtt_chunk_t c;
        flora_tunnel_chunk(msg, len, id, i, qos, retain, &c);
        flora_link_send(link, FLORA_MSG_MQTT_UP, &c, sizeof c);
        vTaskDelay(pdMS_TO_TICKS(PACE_MS));
    }
    s_sent++;
}

static void tx_task(void *arg)
{
    static uint8_t qmsg[Q_HDR + FLORA_TUNNEL_MAX_MSG];
    bool was_up = false;
    uint32_t last_stats = 0;

    for (;;) {
        flora_link_handle_t link = hub_links_handle();
        const bool up = link && flora_link_is_up(link);
        if (up != was_up) {
            was_up = up;
            flora_mqtt_proxy_set_connected(up);
            ESP_LOGW(TAG, "glowica %s - MQTT %s", up ? "osiagalna" : "nieosiagalna",
                     up ? "przez tunel" : "wstrzymane");
            if (up) {
                /* Za flora_net: bez WiFi nikt inny nie wystawi "online". */
                flora_mqtt_publish_str("status", "online", 1, true);
            }
        }
        if (!up) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        size_t n = xMessageBufferReceive(s_prio, qmsg, sizeof qmsg, 0);
        if (n == 0) {
            n = xMessageBufferReceive(s_bulk, qmsg, sizeof qmsg, pdMS_TO_TICKS(50));
        }
        if (n > Q_HDR) {
            send_msg(link, qmsg, n);
        }

        const uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (now - last_stats >= 10000) {
            last_stats = now;
            flora_mqtt_publish_fmt("tele/tunnel", 0, false,
                                   "{\"sent\":%" PRIu32 ",\"dropped\":%" PRIu32
                                   ",\"throttled\":%" PRIu32 ",\"rx\":%" PRIu32
                                   ",\"rx_lost\":%" PRIu32 "}",
                                   s_sent, s_dropped, s_throttled, s_received, s_rxasm.dropped);
        }
    }
}

/* Zadanie lacza: skladamy kawalki i oddajemy calosc do osobnego zadania,
 * bo komendy (NVS, naped) nie powinny blokowac odbioru ramek. */
static void on_down_chunk(uint8_t type, const void *payload, uint8_t len, void *ctx)
{
    if (len != sizeof(flora_mqtt_chunk_t)) {
        return;
    }
    if (flora_tunnel_rx_feed(&s_rxasm, payload)) {
        xMessageBufferSend(s_rx, s_rxasm.buf, s_rxasm.len, 0);
    }
}

static void rx_task(void *arg)
{
    static uint8_t msg[FLORA_TUNNEL_MAX_MSG];
    for (;;) {
        const size_t n = xMessageBufferReceive(s_rx, msg, sizeof msg, portMAX_DELAY);
        const char *t;
        size_t tl, dl;
        const uint8_t *d;
        if (n && flora_tunnel_unpack(msg, n, &t, &tl, &d, &dl)) {
            s_received++;
            flora_mqtt_proxy_deliver(t, tl, (const char *)d, dl);
        }
    }
}

esp_err_t hub_tunnel_start(void)
{
    flora_link_handle_t link = hub_links_handle();
    if (!link) {
        ESP_LOGE(TAG, "brak lacza do glowicy - tunel nie ruszy");
        return ESP_ERR_INVALID_STATE;
    }
    s_prio = xMessageBufferCreate(PRIO_BUF_BYTES);
    s_bulk = xMessageBufferCreate(BULK_BUF_BYTES);
    s_rx = xMessageBufferCreate(RX_BUF_BYTES);
    s_wlock = xSemaphoreCreateMutex();
    if (!s_prio || !s_bulk || !s_rx || !s_wlock) {
        return ESP_ERR_NO_MEM;
    }
    flora_tunnel_rx_init(&s_rxasm);

    esp_err_t err = flora_mqtt_proxy_enable(FLORA_MQTT_BASE_TOPIC, proxy_pub);
    if (err != ESP_OK) {
        return err;
    }
    flora_link_on(link, FLORA_MSG_MQTT_DOWN, on_down_chunk, NULL);
    if (xTaskCreate(tx_task, "tunel_tx", 4096, NULL, 3, NULL) != pdPASS ||
        xTaskCreate(rx_task, "tunel_rx", 4096, NULL, 5, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGW(TAG, "TYMCZASOWE: MQTT bryly przez UART i glowice (bez WiFi)");
    return ESP_OK;
}
