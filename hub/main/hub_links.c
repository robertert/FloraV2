/**
 * @file hub_links.c
 * @brief Odbior wiadomosci glowicy masztu po stronie bryly.
 *
 * Handlery flora_link dzialaja w zadaniu odbiorczym lacza, ktore ma wysoki
 * priorytet i nie powinno czekac na siec. Dlatego tylko kopiuja payload do
 * bufora "ostatnia wartosc", a publikacja na MQTT dzieje sie w zadaniu
 * telemetrii. Przy okazji to naturalnie dlawi ATTITUDE (20 Hz) do tempa
 * telemetrii - posrednie probki i tak nikomu w panelu nie sa potrzebne.
 */
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "esp_log.h"

#include "hub_links.h"
#include "flora_board.h"
#include "flora_msgs.h"

static const char *TAG = "lacze";

typedef struct {
    uint8_t  type;
    uint8_t  len;
    uint32_t seq;      /**< 0 = jeszcze nic nie przyszlo */
    uint8_t  data[HUB_LINKS_MAX_PAYLOAD];
} slot_t;

static slot_t s_slots[] = {
    { .type = FLORA_MSG_HELLO },
    { .type = FLORA_MSG_ATTITUDE },
    { .type = FLORA_MSG_SUN },
    { .type = FLORA_MSG_WEATHER },
    { .type = FLORA_MSG_HEAD_STATUS },
    { .type = FLORA_MSG_OBSTACLES },
    { .type = FLORA_MSG_TAGS },
    { .type = FLORA_MSG_CAM_STATUS },
};
#define SLOT_COUNT (sizeof(s_slots) / sizeof(s_slots[0]))

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static flora_link_handle_t s_link;

flora_link_handle_t hub_links_handle(void) { return s_link; }

static slot_t *find_slot(uint8_t type)
{
    for (size_t i = 0; i < SLOT_COUNT; i++) {
        if (s_slots[i].type == type) {
            return &s_slots[i];
        }
    }
    return NULL;
}

static void on_msg(uint8_t type, const void *payload, uint8_t len, void *ctx)
{
    slot_t *s = ctx;
    if (len > sizeof(s->data)) {
        return;   /* parser i tak odrzuca zle dlugosci, to tylko pas bezpieczenstwa */
    }
    portENTER_CRITICAL(&s_lock);
    memcpy(s->data, payload, len);
    s->len = len;
    s->seq++;
    portEXIT_CRITICAL(&s_lock);

    if (type == FLORA_MSG_HELLO) {
        const flora_hello_t *h = payload;
        if (h->msgs_version != FLORA_MSGS_VERSION) {
            ESP_LOGW(TAG, "glowica ma kontrakt v%u, bryla v%u",
                     h->msgs_version, FLORA_MSGS_VERSION);
        }
    }
}

static void on_state(bool up, void *ctx)
{
    ESP_LOGI(TAG, "glowica: %s", up ? "polaczona" : "rozlaczona");
}

esp_err_t hub_links_start(void)
{
    flora_link_cfg_t cfg = FLORA_LINK_DEFAULT_CFG();
    cfg.name          = "gora";
    cfg.tx_gpio       = FLORA_PIN_HEAD_TX;
    cfg.rx_gpio       = FLORA_PIN_HEAD_RX;
    cfg.proto_version = FLORA_MSGS_VERSION;
    cfg.size_of       = flora_msgs_payload_size;
    cfg.ack_type      = FLORA_MSG_ACK;
    cfg.on_state      = on_state;

    esp_err_t ret = flora_link_init(&cfg, &s_link);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "lacze do glowicy nie wystartowalo: %s", esp_err_to_name(ret));
        return ret;
    }
    for (size_t i = 0; i < SLOT_COUNT; i++) {
        flora_link_on(s_link, s_slots[i].type, on_msg, &s_slots[i]);
    }
    return ESP_OK;
}

bool hub_links_get(uint8_t type, void *buf, uint8_t len, uint32_t *seq)
{
    slot_t *s = find_slot(type);
    if (!s) {
        return false;
    }
    bool fresh = false;
    portENTER_CRITICAL(&s_lock);
    if (s->seq != *seq && s->len == len) {
        memcpy(buf, s->data, len);
        *seq  = s->seq;
        fresh = true;
    }
    portEXIT_CRITICAL(&s_lock);
    return fresh;
}
