/**
 * @file head_tunnel.h
 * @brief TYMCZASOWE (22.11.2026): glowica jako most MQTT dla bryly bez WiFi.
 *
 * Bryla z uszkodzonym ESP wysyla swoje publikacje przez UART
 * (FLORA_MSG_MQTT_UP, hub/main/hub_tunnel.c). Glowica sklada je i wystawia
 * na brokerze pod tym samym tematem (ta sama baza flora/robot1), a komendy
 * cmd/# z brokera odsyla w dol (FLORA_MSG_MQTT_DOWN).
 *
 * Bierny, dopoki bryla nie zacznie nadawac: subskrypcja cmd/# powstaje
 * dopiero po pierwszym kawalku, wiec bryla z wlasnym WiFi nie dostaje
 * komend podwojnie.
 */
#pragma once

#include <stdbool.h>
#include "esp_err.h"
#include "flora_link.h"

esp_err_t head_tunnel_start(flora_link_handle_t down);

/** Stan lacza do bryly (z head_links.c): przy zerwaniu "status" = offline. */
void head_tunnel_link_state(bool up);
