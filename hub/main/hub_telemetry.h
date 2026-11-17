/**
 * @file hub_telemetry.h
 * @brief Sprzet bryly i publikacja jego stanu na MQTT.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "flora_ina226.h"
#include "flora_pwrsw.h"

/** Najdluzsza jedna dawka pompki w trybie debugowania (max_run_ms). */
#define HUB_PUMP_MAX_RUN_MS 10000

/**
 * @brief Inicjalizuje naped, pompke i czujniki bryly, startuje zadanie
 *        publikujace telemetrie.
 *
 * Kazde urzadzenie jest opcjonalne: brak czujnika konczy sie ostrzezeniem
 * w logu i pominieciem jego tematu, a nie bledem startu - panel debugowy
 * ma dzialac wlasnie wtedy, gdy czegos brakuje.
 *
 * Zasilanie masztu zostaje ZALACZONE, zeby glowica od razu nadawala.
 *
 * @param ina  NULL gdy INA226 niedostepny (brak tele/power i detektora pradowego)
 */
esp_err_t hub_telemetry_start(flora_ina226_handle_t ina);

/** Klucz zasilania masztu. NULL gdy niedostepny. */
flora_pwrsw_handle_t hub_telemetry_mast(void);

/** INA226 bryly. NULL gdy niedostepny. */
flora_ina226_handle_t hub_telemetry_ina(void);

/** Czas pracy ostatniej dawki pompki [ms]; 0 gdy jeszcze nie pracowala. */
uint32_t hub_telemetry_pump_last_ms(void);

/**
 * Najblizsze echo dalmierza `index` (0 = A przod lewy, 1 = B przod prawy,
 * 2 = C tyl) z okna ostatnich pomiarow [mm]. NAN, gdy w oknie nie ma zadnego
 * echa, dalmierz jest wylaczony z testu (cmd/sonar) albo niedostepny.
 * Minimum, nie mediana: dla weta napedu grozny jest odczyt za daleki.
 */
float hub_telemetry_sonar_near(int index);

/**
 * Wilgotnosc gleby [%] - odczyt teraz, pod wspolna blokada z telemetria.
 * @return ESP_ERR_INVALID_STATE bez czujnika albo bez kalibracji
 */
esp_err_t hub_telemetry_soil_read(float *pct);

/** Woda w zbiorniku (stan uspokojony). Bez czujnika poziomu zwraca true -
 *  tak samo jak pompka, ktora wtedy nie ma ochrony przed praca na sucho. */
bool hub_telemetry_water_present(void);

/**
 * Wybor dalmierza do testu po kolei: "a" | "b" | "c" (tylko ten, co 200 ms)
 * albo "all" (wszystkie, co 500 ms). Nie zapisywane - po restarcie "all".
 */
esp_err_t hub_telemetry_sonar_select(const char *arg);
