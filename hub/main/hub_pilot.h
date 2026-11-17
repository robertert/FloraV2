/**
 * @file hub_pilot.h
 * @brief Jazda do punktu na robocie: poza z EKF -> flora_pilot -> petla
 *        predkosci kol.
 *
 * WYLACZONY DOMYSLNIE (pilot_en = 0): komendy zwracaja ESP_ERR_NOT_ALLOWED.
 *
 * Wymaga trybu pid (cmd/config/drive_mode pid, nastrojony) i dzialajacego
 * filtru pozycji. Wspolrzedne w ukladzie mapy, jak tele/pose.
 *
 * Komendy:
 *   cmd/goto  "<x_mm> <y_mm>"            jedz do punktu
 *   cmd/goto  "<x_mm> <y_mm> <th_deg>"   i obroc sie do kursu
 *   cmd/goto  "stop"                     przerwij
 *   cmd/turn  "<th_deg>"                 obrot w miejscu do kursu
 * Stan na tele/pilot (2 Hz w trakcie, raz na koniec).
 *
 * BEZPIECZENSTWO:
 *   - zadana predkosc ma termin waznosci 300 ms (hub_drive_set_speed_hold):
 *     zawieszone zadanie pilota nie zostawi jadacego robota,
 *   - reczne cmd/drive przerywa zadanie pilota,
 *   - straznik (hub_safety) trzyma naped niezaleznie; pilot widzi to jako
 *     brak postepu i po kilku sekundach konczy z bledem,
 *   - utyk napedu konczy zadanie od razu.
 * Bez wlaczonego straznika pilot jedzie na slepo - pierwsze proby z kolami
 * w powietrzu albo na pustym placu.
 */
#pragma once

#include <stdbool.h>
#include "esp_err.h"
#include "flora_pilot.h"

esp_err_t hub_pilot_start(void);

/**
 * @return ESP_ERR_NOT_ALLOWED gdy pilot_en = 0, ESP_ERR_INVALID_STATE gdy
 *         naped nie jest w trybie pid, brak pozy, utyk albo trwa OTA
 */
esp_err_t hub_pilot_goto(float x_mm, float y_mm);
esp_err_t hub_pilot_goto_pose(float x_mm, float y_mm, float th_deg);
esp_err_t hub_pilot_turn(float th_deg);

/** Przerywa zadanie i zatrzymuje naped. Bez zadania nic nie robi. */
void hub_pilot_cancel(void);

/** Stan ostatniego zadania; fail moze byc NULL. */
flora_pilot_state_t hub_pilot_state(flora_pilot_fail_t *fail);

/** cmd/goto i cmd/turn. */
esp_err_t hub_pilot_cmd_goto(const char *arg);
esp_err_t hub_pilot_cmd_turn(const char *arg);
