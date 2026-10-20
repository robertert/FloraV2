/**
 * @file bringup_sensors.h
 * @brief Test rozruchowy wilgotnosci gleby i dalmierza ultradzwiekowego.
 */
#pragma once

#include <stdbool.h>

/**
 * @brief Test czujnika gleby i sonaru.
 *
 * @param live_preview true wchodzi na koncu w nieskonczony podglad odczytow
 *                     (funkcja nie wraca), false konczy po samych testach
 */
void bringup_sensors_run(bool live_preview);
