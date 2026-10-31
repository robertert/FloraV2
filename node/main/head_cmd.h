/**
 * @file head_cmd.h
 * @brief Komendy z bryly: rejestracja handlerow i flagi dla zadan.
 *
 * ZASADA: handler lacza NIE wykonuje komendy. Jest wolany z zadania
 * odbiorczego flora_link i blokuje odbior kolejnych ramek, a kalibracja
 * zyroskopu trwa dwie sekundy. Handler ustawia wiec tylko flage, a wykonuje
 * ja zadanie bedace wlascicielem danego zasobu.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "flora_link.h"

void head_cmd_register(flora_link_handle_t down);

/** Zabiera zadanie kalibracji zyroskopu; zwraca true raz na zgloszenie. */
bool head_cmd_take_gyro_cal(void);

/** Zabiera nowe tempo ATTITUDE w Hz; true gdy bylo zadanie. */
bool head_cmd_take_att_hz(uint8_t *hz);

/** Zabiera wymuszona czulosc kompasu; true gdy bylo zadanie. */
bool head_cmd_take_sun_gain(uint8_t *gain_index);

/** Przesuniecie zegara bryly wzgledem naszego, w mikrosekundach. */
int64_t head_cmd_clock_offset_us(void);
