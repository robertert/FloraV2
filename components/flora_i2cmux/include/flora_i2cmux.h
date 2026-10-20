/**
 * @file flora_i2cmux.h
 * @brief Multiplekser I2C TCA9548A.
 *
 * Po co to w ogole jest: VEML7700 ma adres 0x10 wpisany na stale, bez
 * mozliwosci zmiany. Cztery sztuki na jednej magistrali odpowiadalyby
 * jednoczesnie i nie dalo by sie ich rozroznic. Multiplekser rozdziela
 * magistrale na osiem galezi i zwiera do niej tylko wybrana.
 *
 * Uklad jest prosty: jeden rejestr, jeden bajt, kazdy bit to jeden kanal.
 * Mozna wlaczyc kilka naraz, ale wtedy wraca pierwotny problem, wiec
 * flora_i2cmux_select() wybiera dokladnie jeden.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FLORA_I2CMUX_CHANNELS 8

typedef struct flora_i2cmux_s *flora_i2cmux_handle_t;

/**
 * @brief Podlacza multiplekser do istniejacej magistrali.
 *
 * @return ESP_ERR_NOT_FOUND gdy pod adresem nikt nie odpowiada.
 */
esp_err_t flora_i2cmux_init(i2c_master_bus_handle_t bus, uint8_t address,
                            uint32_t speed_hz, flora_i2cmux_handle_t *out);
esp_err_t flora_i2cmux_deinit(flora_i2cmux_handle_t h);

/**
 * @brief Zwiera do magistrali dokladnie jeden kanal.
 *
 * Powtorne wybranie tego samego kanalu nie generuje transakcji - przy
 * odpytywaniu czujnikow w petli oszczedza to sporo ruchu na magistrali.
 */
esp_err_t flora_i2cmux_select(flora_i2cmux_handle_t h, uint8_t channel);

/** Odlacza wszystkie kanaly. Przydatne przy szukaniu zwarc na galeziach. */
esp_err_t flora_i2cmux_disable_all(flora_i2cmux_handle_t h);

/** Odczytuje rejestr sterujacy - pokazuje, co uklad naprawde ma wlaczone. */
esp_err_t flora_i2cmux_get_mask(flora_i2cmux_handle_t h, uint8_t *mask);

/**
 * @brief Przechodzi po kanalach i wypisuje, co odpowiada na kazdym.
 *
 * Pierwsza rzecz do uruchomienia przy problemach - pokazuje od razu,
 * czy czujniki sa na tych kanalach, na ktorych mialy byc.
 */
esp_err_t flora_i2cmux_scan(flora_i2cmux_handle_t h);

/**
 * Adres czujnika za multiplekserem: uchwyt plus numer kanalu.
 *
 * Sluzy do wpiecia w sterowniki czujnikow przez callback, zeby nie musialy
 * zalezec od tego komponentu. Trzymaj strukture tak dlugo, jak zyje czujnik -
 * sterownik przechowuje do niej wskaznik.
 */
typedef struct {
    flora_i2cmux_handle_t mux;
    uint8_t               channel;
} flora_i2cmux_port_t;

/**
 * @brief Wybiera kanal zapisany w flora_i2cmux_port_t.
 *
 * Pasuje do sygnatury esp_err_t (*)(void *), ktorej uzywaja sterowniki
 * czujnikow do przygotowania magistrali przed dostepem.
 */
esp_err_t flora_i2cmux_port_select(void *port);

#ifdef __cplusplus
}
#endif
