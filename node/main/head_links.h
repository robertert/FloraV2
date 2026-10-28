/**
 * @file head_links.h
 * @brief Lacza glowicy: w dol do bryly, w gore do modulu znacznikow.
 */
#pragma once

#include "esp_err.h"
#include "flora_link.h"

/**
 * @brief Uruchamia oba lacza i przekazywanie ramek modulu kamery.
 *
 * Brak modulu znacznikow nie jest bledem - glowica dziala dalej, traci
 * tylko korekte pozycji ze znacznikow.
 */
esp_err_t head_links_init(void);

/** Uchwyt lacza do bryly. NULL przed head_links_init(). */
flora_link_handle_t head_link_down(void);

/** Uchwyt lacza do modulu znacznikow. NULL, gdy modul niedostepny. */
flora_link_handle_t head_link_up(void);

uint32_t head_links_tags_relayed(void);
