/**
 * @file flora_pwrsw.h
 * @brief Klucz zasilania sekcji (w tym projekcie: zasilanie masztu).
 *
 * Steruje tranzystorem odcinajacym 12 V dla calej sekcji. W robocie jest to
 * IRF4905 z NPN na GPIO39, odcinajacy przetwornice XL4015 zasilajaca glowice.
 *
 * To nie jest zwykly gpio_set_level - stad osobny komponent:
 *
 *  - WYMUSZONA PRZERWA PRZED PONOWNYM ZALACZENIEM. Przetwornica ma na
 *    wejsciu kondensatory, ktore musza sie rozladowac; szybkie przelaczanie
 *    meczy ja i grozi wgraniem glowicy w stan nieokreslony, bo jej flash
 *    dostaje napiecie zbyt krotko, by poprawnie wystartowac.
 *
 *  - CZAS ROZRUCHU. Po zalaczeniu glowica potrzebuje czasu na uruchomienie,
 *    zanim zacznie odpowiadac. Komponent zna ten czas i pozwala poczekac,
 *    zamiast zgadywac w aplikacji.
 *
 *  - CZAS PRACY. Robot jest solarny, wiec bilans energii ma znaczenie -
 *    komponent liczy, jak dlugo sekcja byla zalaczona.
 *
 * CZEGO NIE ROBI: nie wie nic o tym, co wisi po drugiej stronie. Jesli
 * trzeba uprzedzic lacze przed odcieciem (flora_link_abort_pending), robi
 * to aplikacja - ona wie, ze zamierza wylaczyc druga strone.
 *
 * UWAGA O PRADZIE ROZRUCHOWYM: zalaczenie przetwornicy z rozladowanymi
 * kondensatorami to krotki impuls pradu znacznie wiekszy niz pobor roboczy.
 * Jesli masz INA226 z progiem alarmowym, nie interpretuj go jako zwarcia.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct flora_pwrsw_s *flora_pwrsw_handle_t;

typedef struct {
    gpio_num_t gpio;
    bool       active_high;    /**< stan pinu zalaczajacy sekcje */

    /** Minimalna przerwa po wylaczeniu, zanim wolno zalaczyc ponownie. */
    uint32_t   min_off_ms;

    /** Ile trwa rozruch sekcji po zalaczeniu, zanim bedzie gotowa. */
    uint32_t   boot_ms;
} flora_pwrsw_cfg_t;

/**
 * Pol sekundy przerwy wystarczy na rozladowanie kondensatorow wejsciowych
 * typowej przetwornicy, a dwie sekundy to z zapasem czas startu ESP32-S3
 * z inicjalizacja czujnikow na I2C.
 */
#define FLORA_PWRSW_DEFAULT_CFG()  \
    {                              \
        .active_high = true,       \
        .min_off_ms  = 500,        \
        .boot_ms     = 2000,       \
    }

typedef struct {
    uint32_t on_count;        /**< ile razy zalaczano */
    uint64_t total_on_ms;     /**< lacznie w stanie zalaczonym */
    int64_t  last_change_us;
    bool     is_on;
} flora_pwrsw_stats_t;

/** Po inicjalizacji sekcja jest WYLACZONA. */
esp_err_t flora_pwrsw_init(const flora_pwrsw_cfg_t *cfg, flora_pwrsw_handle_t *out);

/** Wylacza sekcje i zwalnia uchwyt. */
esp_err_t flora_pwrsw_deinit(flora_pwrsw_handle_t h);

/**
 * @brief Zalacza sekcje.
 *
 * @return ESP_ERR_INVALID_STATE gdy nie minela jeszcze wymuszona przerwa.
 *         Uzyj flora_pwrsw_remaining_off_ms(), zeby dowiedziec sie ile zostalo.
 */
esp_err_t flora_pwrsw_on(flora_pwrsw_handle_t h);

/** Jak wyzej, ale czeka na wymuszona przerwe zamiast zwracac blad. */
esp_err_t flora_pwrsw_on_wait(flora_pwrsw_handle_t h);

esp_err_t flora_pwrsw_off(flora_pwrsw_handle_t h);

bool     flora_pwrsw_is_on(flora_pwrsw_handle_t h);

/** Ile ms zostalo do konca wymuszonej przerwy; 0 gdy mozna zalaczac. */
uint32_t flora_pwrsw_remaining_off_ms(flora_pwrsw_handle_t h);

/**
 * @brief Czeka, az minie czas rozruchu od zalaczenia.
 *
 * Nie sprawdza, czy sekcja NAPRAWDE wystartowala - tylko odmierza czas.
 * Potwierdzenie gotowosci to zadanie warstwy wyzej (w tym projekcie:
 * odebranie HELLO przez flora_link).
 */
esp_err_t flora_pwrsw_wait_boot(flora_pwrsw_handle_t h);

void flora_pwrsw_get_stats(flora_pwrsw_handle_t h, flora_pwrsw_stats_t *out);

#ifdef __cplusplus
}
#endif
