/**
 * @file flora_tunnel.h
 * @brief Wiadomosc MQTT (temat + tresc) <-> kawalki po 60 B dla UART-u.
 *
 * TYMCZASOWE (22.11.2026): bryla z uszkodzonym ESP nie moze podniesc WiFi,
 * wiec jej publikacje ida przez UART do glowicy, a komendy z powrotem.
 * Ramka flora_link miesci 64 B, wiec wiadomosc jest dzielona na kawalki
 * flora_mqtt_chunk_t i skladana po drugiej stronie.
 *
 * Uklad wiadomosci przed podzialem: [dlugosc tematu, 1 B][temat][tresc].
 *
 * Lacze jest bez potwierdzen: zgubiony kawalek = zgubiona cala wiadomosc
 * (telemetria i tak przyjdzie znowu, komende trzeba powtorzyc).
 *
 * Czysty C bez ESP-IDF, test w host/test.c.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "flora_msgs.h"

#define FLORA_TUNNEL_MAX_MSG   512   /**< temat + tresc + 1 */
#define FLORA_TUNNEL_MAX_TOPIC 127

/**
 * Sklada wiadomosc w bufor przed podzialem.
 * @return dlugosc albo 0, gdy temat lub calosc sie nie miesci
 */
size_t flora_tunnel_pack(const char *topic, const void *data, size_t len,
                         uint8_t *out, size_t cap);

/**
 * Rozbiera zlozona wiadomosc. Wskazniki pokazuja do wnetrza `msg`.
 * @return false, gdy uklad sie nie zgadza
 */
bool flora_tunnel_unpack(const uint8_t *msg, size_t msg_len,
                         const char **topic, size_t *topic_len,
                         const uint8_t **data, size_t *data_len);

/** Ile kawalkow zajmie wiadomosc o tej dlugosci. */
int flora_tunnel_chunk_count(size_t msg_len);

/**
 * Wypelnia kawalek numer `idx` wiadomosci `msg`.
 * @param qos, retain  ida we flagach kazdego kawalka
 */
void flora_tunnel_chunk(const uint8_t *msg, size_t msg_len, uint8_t id, int idx,
                        int qos, bool retain, flora_mqtt_chunk_t *out);

/** Skladanie po stronie odbiorcy. Jeden na kierunek, jeden pisarz. */
typedef struct {
    uint8_t buf[FLORA_TUNNEL_MAX_MSG];
    size_t  len;
    uint8_t id;
    uint8_t next_idx;
    bool    active;
    int     qos;
    bool    retain;
    uint32_t dropped;    /**< wiadomosci porzucone przez zgubiony kawalek */
} flora_tunnel_rx_t;

void flora_tunnel_rx_init(flora_tunnel_rx_t *rx);

/**
 * Dokleja kawalek.
 * @return true, gdy wiadomosc jest kompletna: rx->buf / rx->len, rx->qos,
 *         rx->retain sa wazne do nastepnego wywolania
 */
bool flora_tunnel_rx_feed(flora_tunnel_rx_t *rx, const flora_mqtt_chunk_t *c);
