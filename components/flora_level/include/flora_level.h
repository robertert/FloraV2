/**
 * @file flora_level.h
 * @brief Bezkontaktowy czujnik poziomu wody (XKC-Y25).
 *
 * DLACZEGO TO NIE JEST gpio_get_level: woda w zbiorniku jadacego robota
 * CHLUPIE. Czujnik umieszczony na scianie zbiornika bedzie przy kazdym
 * przechyle i przy kazdym ruszeniu przelaczal sie tam i z powrotem. Surowy
 * odczyt dalby wiec "jest woda / nie ma wody" kilka razy na sekunde, a na
 * tej podstawie pompka wlaczalaby sie i wylaczala w kolko.
 *
 * Komponent probkuje wejscie okresowo i zmienia zglaszany stan dopiero po
 * kilku zgodnych odczytach z rzedu. Daje to odpornosc na chlupanie bez
 * opoznienia, ktore mialoby znaczenie - zbiornik nie oproznia sie w sekunde.
 *
 * POLARYZACJA DO SPRAWDZENIA NA EGZEMPLARZU: XKC-Y25 wystepuje w wersjach
 * NPN i PNP, a z wygladu sa nie do odroznienia. Zla flaga oznacza, ze pusty
 * zbiornik bedzie zglaszany jako pelny - czyli pompka pojdzie na sucho.
 * Zanurz czujnik i sprawdz flora_level_raw().
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct flora_level_s *flora_level_handle_t;

/** Wolane przy zmianie stabilnego stanu, z kontekstu zadania timera. */
typedef void (*flora_level_cb_t)(bool water_present, void *ctx);

typedef struct {
    gpio_num_t gpio;
    bool       water_present_level;  /**< stan pinu oznaczajacy obecnosc wody */

    /** XKC-Y25 ma wlasne podciagniecie; wlacz tylko jesli go brakuje. */
    bool       enable_pullup;

    uint32_t   sample_ms;            /**< co ile probkowac wejscie */
    uint8_t    stable_samples;       /**< ile zgodnych odczytow zmienia stan */

    flora_level_cb_t on_change;
    void            *ctx;
} flora_level_cfg_t;

/**
 * Probkowanie co 50 ms i osiem zgodnych odczytow daje 400 ms uspokajania -
 * dluzej niz typowe chlupniecie przy ruszeniu, a znacznie krocej niz czas,
 * w ktorym poziom wody moze sie realnie zmienic.
 */
#define FLORA_LEVEL_DEFAULT_CFG()       \
    {                                   \
        .water_present_level = true,     \
        .enable_pullup       = false,    \
        .sample_ms           = 50,       \
        .stable_samples      = 8,        \
    }

esp_err_t flora_level_init(const flora_level_cfg_t *cfg, flora_level_handle_t *out);
esp_err_t flora_level_deinit(flora_level_handle_t h);

/** Stabilny stan po uspokojeniu. Tego uzywaj do decyzji o podlewaniu. */
bool flora_level_water_present(flora_level_handle_t h);

/** Surowy odczyt pinu, z pominieciem uspokajania. Do sprawdzania polaryzacji. */
bool flora_level_raw(flora_level_handle_t h);

/** Ile razy stabilny stan sie zmienil - duza liczba zdradza chlupanie. */
uint32_t flora_level_change_count(flora_level_handle_t h);

/**
 * @brief Gotowy callback do wpiecia w flora_pump jako `water_cb`.
 *
 * Jako kontekst podaj uchwyt czujnika. Dzieki temu pompka dostaje stan
 * uspokojony, a nie surowy odczyt pinu.
 */
bool flora_level_pump_cb(void *handle_as_ctx);

#ifdef __cplusplus
}
#endif
