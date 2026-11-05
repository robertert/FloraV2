/**
 * @file hub_sensorcal.h
 * @brief Kalibracje czujnikow bryly: wydajnosc pompki i bocznik INA226.
 *
 * POMPKA (dziala od razu):
 *   1. cmd/pump 10000          - 10 s pompowania do menzurki
 *   2. cmd/pump_cal <ml>       - zmierzona objetosc; ml/s = ml / czas ostatniej
 *                                dawki, zapis w NVS
 *   Potem dawki w ml: cmd/pump "ml 150". Wprost: cmd/config/pump_ml_s.
 *
 * INA226 (dziala po restarcie, cmd/reboot):
 *   Bocznik R010 ma tolerancje ~1%, a prad to napiecie na nim / opor.
 *   1. przepusc staly prad >= 0,3 A (np. pompka albo silniki na stalej mocy)
 *      i zmierz go multimetrem w szeregu,
 *   2. cmd/ina_cal <A z multimetru> - nowy opor = obecny * odczyt INA / prawda.
 *   Restart, bo opor wchodzi w rejestr kalibracji ukladu przy inicjalizacji,
 *   a uchwyt INA dziela straznik utyku i telemetria. Wprost:
 *   cmd/config/ina_shunt (mOhm), "-" wraca do 10 mOhm.
 */
#pragma once

#include "esp_err.h"

/** Wydajnosc pompki z NVS [ml/s]; 0 gdy brak kalibracji. */
float hub_sensorcal_pump_ml_s(void);

/**
 * Opor bocznika do inicjalizacji INA226 [mOhm]: z NVS albo FLORA_SHUNT_MILLIOHM.
 * Wolac raz przy starcie - zapamietuje wartosc jako obowiazujaca do restartu.
 */
float hub_sensorcal_ina_shunt_boot(void);

/** Opor bocznika uzywany teraz przez uklad (ten z hub_sensorcal_ina_shunt_boot). */
float hub_sensorcal_ina_shunt_active(void);

/** @param arg zmierzona objetosc ostatniej dawki w ml */
esp_err_t hub_sensorcal_pump(const char *arg);

/** @param arg prad z multimetru w A (>= 0,3) */
esp_err_t hub_sensorcal_ina(const char *arg);
