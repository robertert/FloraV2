/**
 * @file flora_net.h
 * @brief WiFi + MQTT z lancuchem prob: NVS -> serwer z firmware -> broker na Macu.
 */
#pragma once

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /** Identyfikator klienta MQTT - rozny dla kazdej jednostki, bo broker
     *  rozlacza starszego klienta o tym samym identyfikatorze. NULL = "flora-hub". */
    const char *client_id;
    /** Temat "online"/"offline" (retain + testament), wzgledny. NULL = "status". */
    const char *status_topic;
    /** Opcjonalnie: wolane przy kazdej zmianie stanu polaczenia z brokerem. */
    void (*on_mqtt)(bool connected);
} flora_net_cfg_t;

/**
 * Laczy z WiFi i brokerem. Blokuje do ~30 s na probe. Przy bledzie klienci
 * z ostatniej proby zostaja i ponawiaja w tle, wiec polaczenie moze dojsc
 * pozniej - brak sieci nie powinien blokowac reszty programu.
 */
esp_err_t flora_net_connect(const flora_net_cfg_t *cfg);

#ifdef __cplusplus
}
#endif
