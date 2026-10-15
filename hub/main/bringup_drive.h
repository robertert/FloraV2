/**
 * @file bringup_drive.h
 * @brief Test rozruchowy napedu, wydzielony z main.c.
 */
#pragma once

#include "flora_ina226.h"

/**
 * @brief Pelny test napedu: diagnostyka enkoderow + 7 przebiegow jazdy.
 *
 * Blokuje na ~25 s. Kola musza byc w powietrzu.
 *
 * @param ina uchwyt INA226 do logowania pradu; NULL gdy niedostepny
 */
void bringup_drive_run(flora_ina226_handle_t ina);
