/**
 * @file flora_tunnel.c
 * @brief Tunel MQTT przez UART, opis w flora_tunnel.h.
 */
#include <string.h>

#include "flora_tunnel.h"

#define CHUNK_DATA ((int)sizeof(((flora_mqtt_chunk_t *)0)->data))

size_t flora_tunnel_pack(const char *topic, const void *data, size_t len,
                         uint8_t *out, size_t cap)
{
    if (!topic || (len > 0 && !data) || !out) {
        return 0;
    }
    const size_t tl = strlen(topic);
    if (tl == 0 || tl > FLORA_TUNNEL_MAX_TOPIC) {
        return 0;
    }
    const size_t total = 1 + tl + len;
    if (total > cap || total > FLORA_TUNNEL_MAX_MSG) {
        return 0;
    }
    out[0] = (uint8_t)tl;
    memcpy(out + 1, topic, tl);
    if (len > 0) {
        memcpy(out + 1 + tl, data, len);
    }
    return total;
}

bool flora_tunnel_unpack(const uint8_t *msg, size_t msg_len,
                         const char **topic, size_t *topic_len,
                         const uint8_t **data, size_t *data_len)
{
    if (!msg || msg_len < 2) {
        return false;
    }
    const size_t tl = msg[0];
    if (tl == 0 || 1 + tl > msg_len) {
        return false;
    }
    *topic = (const char *)(msg + 1);
    *topic_len = tl;
    *data = msg + 1 + tl;
    *data_len = msg_len - 1 - tl;
    return true;
}

int flora_tunnel_chunk_count(size_t msg_len)
{
    return msg_len == 0 ? 0 : (int)((msg_len + CHUNK_DATA - 1) / CHUNK_DATA);
}

void flora_tunnel_chunk(const uint8_t *msg, size_t msg_len, uint8_t id, int idx,
                        int qos, bool retain, flora_mqtt_chunk_t *out)
{
    const size_t off = (size_t)idx * CHUNK_DATA;
    size_t n = msg_len > off ? msg_len - off : 0;
    if (n > (size_t)CHUNK_DATA) {
        n = CHUNK_DATA;
    }
    memset(out, 0, sizeof *out);
    out->id = id;
    out->idx = (uint8_t)idx;
    out->n = (uint8_t)n;
    out->flags = (uint8_t)(((qos & 3) << FLORA_MQTT_CHUNK_QOS_SHIFT) |
                           (retain ? FLORA_MQTT_CHUNK_RETAIN : 0) |
                           (off + n >= msg_len ? FLORA_MQTT_CHUNK_LAST : 0));
    if (n > 0) {
        memcpy(out->data, msg + off, n);
    }
}

void flora_tunnel_rx_init(flora_tunnel_rx_t *rx)
{
    memset(rx, 0, sizeof *rx);
}

bool flora_tunnel_rx_feed(flora_tunnel_rx_t *rx, const flora_mqtt_chunk_t *c)
{
    if (c->n > CHUNK_DATA) {
        rx->active = false;
        rx->dropped++;
        return false;
    }
    if (c->idx == 0) {
        if (rx->active) {
            rx->dropped++;          /* poprzednia nie doszla do konca */
        }
        rx->active = true;
        rx->id = c->id;
        rx->len = 0;
        rx->next_idx = 0;
    } else if (!rx->active || c->id != rx->id || c->idx != rx->next_idx) {
        if (rx->active) {
            rx->dropped++;
        }
        rx->active = false;         /* dziura - czekamy na nastepny poczatek */
        return false;
    }
    if (rx->len + c->n > sizeof rx->buf) {
        rx->active = false;
        rx->dropped++;
        return false;
    }
    memcpy(rx->buf + rx->len, c->data, c->n);
    rx->len += c->n;
    rx->next_idx++;
    if (c->flags & FLORA_MQTT_CHUNK_LAST) {
        rx->active = false;
        rx->qos = (c->flags >> FLORA_MQTT_CHUNK_QOS_SHIFT) & 3;
        rx->retain = (c->flags & FLORA_MQTT_CHUNK_RETAIN) != 0;
        return true;
    }
    return false;
}
