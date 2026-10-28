/**
 * @file bringup_net.h
 * @brief Test rozruchowy WiFi i MQTT.
 */
#pragma once

/**
 * @brief Laczy z WiFi i brokerem, publikuje probna telemetrie.
 *
 * Wymaga wczesniejszego flora_store_init(). Blokuje do 30 s w najgorszym
 * przypadku; ponawianie WiFi dziala dalej w tle takze po powrocie.
 */
void bringup_net_run(void);
