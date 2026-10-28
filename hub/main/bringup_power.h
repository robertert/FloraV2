/**
 * @file bringup_power.h
 * @brief Weryfikacja toru pomiarowego INA226.
 */
#pragma once

#include "driver/i2c_master.h"

/**
 * @brief Sprawdza, czy INA226 mierzy prad, a nie tylko napiecie.
 *
 * Rusza silnikami na trzech poziomach mocy i sprawdza, czy prad rosnie.
 * Sam odczyt wartosci nie wystarcza - uszkodzony egzemplarz czytal
 * napiecie poprawnie i podawal staly prad niezaleznie od obciazenia.
 *
 * Wymaga zainicjalizowanego flora_motors. Kola moga sie krecic.
 */
void bringup_power_run(i2c_master_bus_handle_t bus);
