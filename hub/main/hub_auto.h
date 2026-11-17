/**
 * @file hub_auto.h
 * @brief Start funkcji autonomicznych bryly i ich wspolne zadanie 1 Hz.
 *
 * Moduly (kazdy za wlasnym przelacznikiem, domyslnie WYLACZONE, lista
 * w hub_autocfg.h):
 *   hub_safety   straznik napedu            wlasne zadanie 50 ms
 *   hub_pilot    jazda do punktu            wlasne zadanie 50 ms
 *   hub_power    energia, klucz MDD10A, sen krok 1 s
 *   hub_water    podlewanie                 krok 1 s
 *   hub_mission  szukanie slonca            krok 1 s
 *
 * Trzy ostatnie dziela jedno zadanie: sa wolne, a kolejnosc w kroku ma
 * znaczenie (energia -> podlewanie -> misja -> decyzja o snie w kolejnym
 * kroku widzi juz swieze zgody).
 */
#pragma once

#include "esp_err.h"

/** Wymaga hub_autocfg_load(), hub_telemetry_start() i hub_links_start(). */
esp_err_t hub_auto_start(void);
