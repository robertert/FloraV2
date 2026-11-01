/**
 * @file hub_telemetry.h
 * @brief Sprzet bryly i publikacja jego stanu na MQTT.
 */
#pragma once

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
