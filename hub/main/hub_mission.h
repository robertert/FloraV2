/**
 * @file hub_mission.h
 * @brief Misja na robocie: szukanie slonca (flora_mission + flora_seek)
 *        spiete z pilotem, straznikiem, energia i podlewaniem.
 *
 * WYLACZONA DOMYSLNIE (mission_en = 0).
 *
 * Zanim wlaczysz:
 *   1. naped w trybie pid, nastrojony (TESTY_RECZNE.md),
 *   2. pilot_en 1 i sprawdzone cmd/goto,
 *   3. guard_en 1 (bez straznika misja jezdzi na slepo),
 *   4. obszar roboczy: cmd/area "xmin ymin xmax ymax" w mm, w ukladzie
 *      mapy (tym samym co tele/pose i mapa palikow); srodek robota trzyma
 *      sie 400 mm od granicy,
 *   5. glowica dziala (jasnosc z krzyza VEML) i jest czas z sieci.
 *
 * Komendy:
 *   cmd/area     "xmin ymin xmax ymax" | "-"
 *   cmd/mission  "seek"   wymus szukanie teraz (bez czekania na cien)
 *                "reset"  zdejmij zatrzask awarii, licz postoj od nowa
 *                "stop"   przerwij jazde i wylacz misje (mission_en = 0)
 * Stan na tele/mission (co 10 s i przy zmianie).
 *
 * Reczna jazda z panelu (cmd/drive) wstrzymuje misje na MANUAL_HOLD_S.
 */
#pragma once

#include <stdbool.h>
#include "esp_err.h"

/** Tyle po ostatnim recznym cmd/drive misja nie rusza. */
#define HUB_MISSION_MANUAL_HOLD_S 60

esp_err_t hub_mission_start(void);

/** Krok co 1 s (z zadania hub_auto). */
void hub_mission_tick(void);

/** Robot stoi z punktu widzenia misji - wolno podlewac. */
bool hub_mission_water_ok(void);

/** Misja nie ma nic w toku - wolno zasnac. */
bool hub_mission_may_sleep(void);

/** Reczna komenda jazdy - misja ustepuje. */
void hub_mission_note_manual(void);

/** cmd/mission. */
esp_err_t hub_mission_cmd(const char *arg);
