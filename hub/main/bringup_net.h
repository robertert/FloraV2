/**
 * @file bringup_net.h
 * @brief Test rozruchowy WiFi i MQTT.
 */
#pragma once

#include "esp_err.h"

/**
 * @brief Laczy z WiFi i brokerem MQTT, bez probnej telemetrii.
 *
 * Dane dostepowe z NVS, domyslne z flora_board.h. Ustawia testament
 * "status=offline" i publikuje "online" po kazdym polaczeniu.
 *
 * @return ESP_OK gdy broker odpowiada; ESP_ERR_TIMEOUT gdy klient dziala,
 *         ale broker jeszcze nie odpowiedzial (ponawianie trwa w tle)
 */
esp_err_t bringup_net_connect(void);

/**
 * @brief Laczy z WiFi i brokerem, publikuje probna telemetrie.
 *
 * Wymaga wczesniejszego flora_store_init(). Blokuje do 30 s w najgorszym
 * przypadku; ponawianie WiFi dziala dalej w tle takze po powrocie.
 */
void bringup_net_run(void);
