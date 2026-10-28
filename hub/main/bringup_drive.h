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

/**
 * @brief Sama kalibracja przelozenia, bez pozostalych testow.
 *
 * Kazdy przejazd to 5 s jazdy prosto na 40% mocy - dluzej niz w pelnym
 * tescie, bo przy dluzszej drodze blad pomiaru linijka mniej wazy.
 * Po kazdym przejezdzie wypisuje tabelke "zmierzone -> przelozenie".
 *
 * Potrzeba okolo metra wolnej przestrzeni na przejazd.
 *
 * @param ina   uchwyt INA226 (moze byc NULL)
 * @param runs  ile przejazdow, z przerwa 8 s na ustawienie robota
 */
void bringup_drive_calibrate(flora_ina226_handle_t ina, int runs);
