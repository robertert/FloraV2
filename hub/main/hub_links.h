/**
 * @file hub_links.h
 * @brief Lacze bryly do glowicy masztu i bufor ostatnich wiadomosci.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "flora_link.h"

/** Najwiekszy payload glowica->bryla (flora_tags_t). */
#define HUB_LINKS_MAX_PAYLOAD 40

/**
 * @brief Podnosi lacze UART do glowicy i podpina odbior jej wiadomosci.
 *
 * Brak glowicy nie jest bledem - lacze czeka, az sie odezwie (np. po
 * zalaczeniu zasilania masztu).
 */
esp_err_t hub_links_start(void);

/** Uchwyt lacza. NULL przed hub_links_start(). */
flora_link_handle_t hub_links_handle(void);

/**
 * @brief Kopiuje ostatnio odebrana wiadomosc danego typu.
 *
 * @param seq  wejscie: numer ostatnio widzianej kopii, wyjscie: biezacy.
 *             Funkcja zwraca true tylko gdy przyszlo cos nowszego, wiec
 *             wywolujacy nie publikuje dwa razy tej samej ramki.
 * @return true gdy w buf jest nowa wiadomosc
 */
bool hub_links_get(uint8_t type, void *buf, uint8_t len, uint32_t *seq);
