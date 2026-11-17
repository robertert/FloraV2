/**
 * @file hub_safety.h
 * @brief Straznik napedu na sprzecie: zbiera czujniki, liczy flora_guard
 *        i odpowiada petli napedu, czy wolno jechac.
 *
 * WYLACZONY DOMYSLNIE (guard_en = 0): hub_safety_permit() zwraca wtedy
 * zawsze 1 i naped dziala jak bez straznika.
 *
 * Wlaczenie: cmd/config/guard_en 1. Ktore straze dzialaja, ustala
 * guard_mask (hub_autocfg.h). Do pierwszych prob bez kamery w dol:
 * guard_mask 30 (wszystko poza kamera) - inaczej brak ramek przeszkod
 * blokuje jazde w przod, bo kamera, ktora milczy, niczego nie potwierdza.
 *
 * Zadanie co 50 ms sklada wejscia:
 *   ATTITUDE, OBSTACLES   z hub_links (wiek liczony od odbioru)
 *   dalmierze             najblizsze echo z okna filtra (hub_telemetry)
 *   napiecie              z hub_power
 * i publikuje tele/safety (co 1 s i przy zmianie).
 *
 * EGZEKUCJA jest w hub_drive: kazde zadanie mocy i kazdy takt petli pyta
 * hub_safety_permit(). Samo zadanie straznika silnikow nie dotyka.
 *
 * ZAWIESZONY STRAZNIK = STOP: werdykt starszy niz 300 ms przy wlaczonym
 * strazniku oznacza brak zgody na jazde.
 */
#pragma once

#include <stdbool.h>
#include "esp_err.h"
#include "flora_guard.h"

esp_err_t hub_safety_start(void);

/** guard_en wlaczony i zadanie dziala. */
bool hub_safety_enabled(void);

/**
 * Mnoznik 0..1 dla zadanych kol (jednostka dowolna, liczy sie znak).
 * 0 = zabronione. Bezpieczne z timera - nie blokuje.
 */
float hub_safety_permit(float left, float right);

/** Czy autonomii wolno dzialac. Przy wylaczonym strazniku zawsze true. */
bool hub_safety_autonomy_ok(void);

/** Ostatni werdykt (do telemetrii i testow). */
void hub_safety_get(flora_guard_out_t *out);
