/**
 * @file hub_tunnel.h
 * @brief TYMCZASOWE (22.11.2026): MQTT bryly przez UART do glowicy.
 *
 * ESP bryly po uszkodzeniu nie wytrzymuje startu WiFi (brownout przy
 * phy_init). Do czasu wymiany plytki bryla nie podnosi WiFi wcale: kazda
 * publikacja (telemetria, ack, logi, config) idzie kawalkami przez lacze
 * do glowicy, a glowica wystawia ja na brokerze pod tym samym tematem.
 * Komendy cmd/# glowica odbiera i odsyla w dol. Panel i Grafana nie widza
 * roznicy.
 *
 * Wlaczane w main.c (TEST_MQTT_PRZEZ_GLOWICE). Wymaga glowicy z head_tunnel.c.
 *
 * Ograniczenia:
 * - przepustowosc: lacze 57600 bodow, tunel trzyma ~3,5 kB/s, zeby nie
 *   zapchac UART-u; nadmiar telemetrii jest odrzucany (licznik na
 *   tele/tunnel). Ack, logi, retain i qos >= 1 maja osobna, pierwsza kolejke;
 * - brak OTA i czasu z SNTP (to wymaga sieci w bryle);
 * - "status" online/offline wystawia za bryle glowica.
 */
#pragma once

#include "esp_err.h"

/** Po hub_links_start(), przed hub_remote_start(). */
esp_err_t hub_tunnel_start(void);
