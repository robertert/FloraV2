/**
 * @file flora_temp.h
 * @brief Czujniki temperatury DS18B20 na magistrali 1-Wire.
 *
 * Cienka warstwa nad komponentami espressif/ds18b20 i espressif/onewire_bus
 * (1-Wire realizowany na peryferiale RMT, bez bit-bangu). Zaleznosc od nich
 * zostaje wewnatrz - aplikacja widzi tylko ten naglowek.
 *
 * Sama obsluga odczytu, bez zadnej polityki. Decyzje w rodzaju "za goraco,
 * przerwij ladowanie" naleza do warstwy sterowania.
 *
 * WAZNE O CZASIE KONWERSJI: DS18B20 potrzebuje do 750 ms przy 12 bitach.
 * Dlatego sa dwa tryby pracy:
 *   - flora_temp_read()           prosto, ale BLOKUJE na czas konwersji
 *   - start_conversion() + fetch() nieblokujaco, do petli sterowania
 * W petli regulacji uzywaj drugiego, inaczej zatrzymasz zadanie na 0,75 s.
 *
 * Uklad na magistrali wymaga rezystora podciagajacego 4,7k do 3,3V.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Maksymalna liczba czujnikow na jednej magistrali. */
#define FLORA_TEMP_MAX_SENSORS 4

/** Wartosc zwracana, gdy odczyt jest nieznany. */
#define FLORA_TEMP_INVALID (-127.0f)

/** Rozdzielczosc pomiaru. Mniej bitow to szybsza konwersja. */
typedef enum {
    FLORA_TEMP_RES_9BIT,   /**< 0,5 C    ~94 ms  */
    FLORA_TEMP_RES_10BIT,  /**< 0,25 C   ~188 ms */
    FLORA_TEMP_RES_11BIT,  /**< 0,125 C  ~375 ms */
    FLORA_TEMP_RES_12BIT,  /**< 0,0625 C ~750 ms */
} flora_temp_resolution_t;

typedef struct {
    gpio_num_t gpio;

    /**
     * Wewnetrzne podciagniecie. Zostaw false, jesli masz zewnetrzne 4,7k -
     * wewnetrzne (~45k) nie da rady wysterowac magistrali przy dluzszym
     * przewodzie i odczyty zaczna byc losowe.
     */
    bool enable_internal_pullup;

    flora_temp_resolution_t resolution;
} flora_temp_cfg_t;

/**
 * Domyslnie 11 bitow: 0,125 C w 375 ms. Dla temperatury pakietu i tak
 * nie potrzeba wiecej, a konwersja trwa o polowe krocej niz przy 12 bitach.
 */
#define FLORA_TEMP_DEFAULT_CFG()              \
    {                                         \
        .enable_internal_pullup = false,      \
        .resolution             = FLORA_TEMP_RES_11BIT, \
    }

/**
 * @brief Uruchamia magistrale i wyszukuje czujniki.
 *
 * @return ESP_ERR_NOT_FOUND gdy na magistrali nie ma zadnego DS18B20.
 */
esp_err_t flora_temp_init(const flora_temp_cfg_t *cfg);
esp_err_t flora_temp_deinit(void);

/** Liczba znalezionych czujnikow. */
int flora_temp_count(void);

/** Adres ROM czujnika, unikalny i staly. 0 gdy indeks poza zakresem. */
uint64_t flora_temp_address(int index);

/** Indeks czujnika o podanym adresie ROM, -1 gdy nie ma takiego. */
int flora_temp_index_of(uint64_t address);

/** Czas konwersji przy obecnej rozdzielczosci, z zapasem. */
uint32_t flora_temp_conversion_ms(void);

/* --- Odczyt prosty, blokujacy --- */

/** Zleca konwersje, czeka i zwraca wynik. Blokuje na flora_temp_conversion_ms(). */
esp_err_t flora_temp_read(int index, float *celsius);

/** Jak wyzej dla wszystkich czujnikow naraz (jedna wspolna konwersja). */
esp_err_t flora_temp_read_all(float *out, int max, int *count);

/* --- Odczyt dwuetapowy, nieblokujacy --- */

/** Zleca konwersje wszystkim czujnikom i wraca natychmiast. */
esp_err_t flora_temp_start_conversion(void);

/**
 * @brief Pobiera wynik ostatniej konwersji.
 *
 * Wolaj dopiero po flora_temp_conversion_ms() od start_conversion().
 * Wczesniej dostaniesz poprzedni wynik albo 85,0 C - wartosc, ktora
 * DS18B20 trzyma w rejestrze po resecie i ktora bardzo latwo wziac
 * za prawdziwy pomiar.
 */
esp_err_t flora_temp_fetch(int index, float *celsius);

#ifdef __cplusplus
}
#endif
