/**
 * @file flora_bme280.h
 * @brief Barometr, termometr i higrometr BME280 (oraz BMP280 bez wilgotnosci).
 *
 * Komponent rozpoznaje uklad po identyfikatorze i sam ustala, czy wilgotnosc
 * jest dostepna: BME280 zglasza 0x60, BMP280 zglasza 0x58. Drugi jest czesto
 * sprzedawany jako "BME" i roznicy nie widac po obudowie, wiec sprawdzenie
 * jest tutaj, a nie w zalozeniach.
 *
 * Pomiar dziala w trybie wymuszonym: kazdy odczyt zleca jedna konwersje,
 * czeka na jej koniec i usypia uklad. Przy odczytach co kilka sekund,
 * a tak pracuje stacja meteo, jest to wlasciwy tryb - pobor w spoczynku
 * spada do mikroamperow i uklad sie nie grzeje, co jest istotne, bo
 * samonagrzewanie zawyza temperature.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Krotnosc nadprobkowania. Wiecej to mniejszy szum i dluzszy pomiar. */
typedef enum {
    FLORA_BME_OS_SKIP = 0,  /**< kanal wylaczony */
    FLORA_BME_OS_1,
    FLORA_BME_OS_2,
    FLORA_BME_OS_4,
    FLORA_BME_OS_8,
    FLORA_BME_OS_16,
} flora_bme_os_t;

/** Filtr dolnoprzepustowy IIR - tlumi skoki cisnienia od wiatru i trzasniecia drzwiami. */
typedef enum {
    FLORA_BME_FILTER_OFF = 0,
    FLORA_BME_FILTER_2,
    FLORA_BME_FILTER_4,
    FLORA_BME_FILTER_8,
    FLORA_BME_FILTER_16,
} flora_bme_filter_t;

typedef struct {
    uint8_t  address;
    uint32_t speed_hz;

    flora_bme_os_t     os_temperature;
    flora_bme_os_t     os_pressure;
    flora_bme_os_t     os_humidity;
    flora_bme_filter_t filter;
} flora_bme280_cfg_t;

/**
 * Nastawy zalecane przez producenta dla stacji pogodowej: minimalne
 * nadprobkowanie i filtr wylaczony. Przy pomiarach co kilkadziesiat sekund
 * usrednianie nic nie wnosi, a kosztuje czas i prad.
 */
#define FLORA_BME280_DEFAULT_CFG()           \
    {                                        \
        .address        = 0x76,              \
        .speed_hz       = 100000,            \
        .os_temperature = FLORA_BME_OS_1,    \
        .os_pressure    = FLORA_BME_OS_1,    \
        .os_humidity    = FLORA_BME_OS_1,    \
        .filter         = FLORA_BME_FILTER_OFF, \
    }

typedef struct flora_bme280_s *flora_bme280_handle_t;

typedef struct {
    float temperature_c;
    float pressure_pa;
    float humidity_pct;   /**< NAN na BMP280, ktory nie ma higrometru */
} flora_bme280_data_t;

esp_err_t flora_bme280_init(i2c_master_bus_handle_t bus,
                            const flora_bme280_cfg_t *cfg,
                            flora_bme280_handle_t *out);
esp_err_t flora_bme280_deinit(flora_bme280_handle_t h);

/** Zleca pomiar, czeka na wynik i zwraca komplet. Blokuje na kilkadziesiat ms. */
esp_err_t flora_bme280_read(flora_bme280_handle_t h, flora_bme280_data_t *out);

/** true dla BME280, false dla BMP280 (brak kanalu wilgotnosci). */
bool flora_bme280_has_humidity(flora_bme280_handle_t h);

/** Identyfikator ukladu odczytany przy inicjalizacji. */
uint8_t flora_bme280_chip_id(flora_bme280_handle_t h);

/**
 * @brief Przelicza cisnienie na poziom morza.
 *
 * Bez tego odczytow nie da sie porownac z prognoza ani z inna stacja -
 * sama wysokosc zmienia cisnienie o okolo 12 Pa na metr.
 *
 * @param altitude_m wysokosc miejsca pomiaru nad poziomem morza
 */
float flora_bme280_sea_level_pa(float pressure_pa, float temperature_c, float altitude_m);

#ifdef __cplusplus
}
#endif
