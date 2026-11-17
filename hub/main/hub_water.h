/**
 * @file hub_water.h
 * @brief Automatyczne podlewanie: czujnik gleby -> flora_water -> pompka.
 *
 * WYLACZONE DOMYSLNIE (water_en = 0): pompka chodzi tylko z cmd/pump.
 *
 * Wymaga kalibracji wydajnosci pompki (cmd/pump_cal) - dawki sa w ml.
 * Bez niej stan to "no_cal" i nic sie nie dzieje.
 *
 * Co sekunde: wilgotnosc, poziom wody, decyzja flora_water. Dawka idzie
 * w kawalkach nie dluzszych niz HUB_PUMP_MAX_RUN_MS; twarde limity pompki
 * (sufit czasu, praca na sucho, limit dobowy czasu) zostaja w flora_pump.
 * Podane ml liczone sa z czasu pracy pompki, wiec reczne cmd/pump tez
 * wchodzi do limitu dobowego.
 *
 * Podlewa tylko, gdy robot stoi (naped bez mocy, misja nie jedzie) i tryb
 * energii na to pozwala.
 *
 * Stan na tele/water (co 10 s i przy zmianie). cmd/water "reset" zdejmuje
 * zatrzask awarii (brak reakcji gleby, pompka).
 */
#pragma once

#include <stdbool.h>
#include "esp_err.h"

esp_err_t hub_water_start(void);

/** Krok co 1 s (z zadania hub_auto). */
void hub_water_tick(void);

/** Trwa dawka (pompka pracuje albo zaraz ruszy z kolejnym kawalkiem). */
bool hub_water_busy(void);

/** cmd/water: "reset". */
esp_err_t hub_water_cmd(const char *arg);
