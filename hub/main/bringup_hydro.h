/**
 * @file bringup_hydro.h
 * @brief Test rozruchowy pompki i czujnikow temperatury.
 */
#pragma once

#include "flora_ina226.h"

/**
 * @brief Test temperatury (DS18B20) i pompki wraz z zabezpieczeniami.
 *
 * Blokuje na ~20 s, z czego pompka pracuje okolo 3 s.
 * UWAGA: bez czujnika poziomu nie ma ochrony przed praca na sucho.
 *
 * @param ina uchwyt INA226 do odczytu napiecia szyny; NULL gdy niedostepny
 */
void bringup_hydro_run(flora_ina226_handle_t ina);
