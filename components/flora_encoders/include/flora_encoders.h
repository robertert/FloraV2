/**
 * @file flora_encoders.h
 * @brief Enkodery kwadraturowe kol na sprzetowym PCNT (dekodowanie x4).
 *
 * Kazde kolo dostaje wlasna jednostke PCNT z dwoma kanalami, co daje
 * zliczanie obu zbocz obu kanalow (x4) bez obciazania CPU przerwaniami.
 *
 * Licznik sprzetowy PCNT jest 16-bitowy, wiec jednostki sa konfigurowane
 * z flaga accum_count i punktami obserwacyjnymi na obu limitach - sterownik
 * akumuluje przepelnienia sam, a odczyt zwraca ciagla wartosc.
 */
#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    gpio_num_t a_gpio;
    gpio_num_t b_gpio;
    bool       invert;  /**< true gdy kolo kreci sie w przeciwna strone niz zliczenia rosna */
} flora_encoder_cfg_t;

typedef struct {
    flora_encoder_cfg_t left;
    flora_encoder_cfg_t right;

    /** Filtr szpilek. 1000 ns odsiewa zaklocenia od szczotek silnika. */
    uint32_t glitch_filter_ns;

    /**
     * Limity licznika sprzetowego, po ktorych nastepuje akumulacja.
     * Mieszcza sie w 16 bitach ze znakiem, wiec |limit| < 32768.
     */
    int high_limit;
    int low_limit;

    /** Droga w mm na jedno zliczenie. Patrz FLORA_MM_PER_COUNT w flora_board.h. */
    float mm_per_count;

    /** Wlacza wewnetrzne podciagniecia na A i B (jesli enkoder ma otwarty kolektor). */
    bool enable_pullups;
} flora_encoders_cfg_t;

#define FLORA_ENCODERS_DEFAULT_CFG()   \
    {                                  \
        .glitch_filter_ns = 1000,      \
        .high_limit       =  16384,    \
        .low_limit        = -16384,    \
        .enable_pullups   = true,      \
    }

/** Migawka obu enkoderow pobrana w jednym momencie. */
typedef struct {
    int64_t  left_counts;
    int64_t  right_counts;
    float    left_mm;
    float    right_mm;
    int64_t  timestamp_us;   /**< esp_timer_get_time() w chwili odczytu */
} flora_encoders_sample_t;

esp_err_t flora_encoders_init(const flora_encoders_cfg_t *cfg);
esp_err_t flora_encoders_deinit(void);

/** Pobiera stan obu enkoderow. Oba odczyty sa robione obok siebie, bez blokowania. */
esp_err_t flora_encoders_read(flora_encoders_sample_t *out);

/** Zeruje oba liczniki (sprzetowy i akumulator programowy). */
esp_err_t flora_encoders_reset(void);

/**
 * @brief Predkosc kol wyliczona z dwoch kolejnych migawek.
 *
 * Funkcja pomocnicza, zeby warstwa PID nie powtarzala tego samego dzielenia.
 * Zwraca mm/s. Przy identycznych znacznikach czasu zwraca 0.
 */
void flora_encoders_speed_mm_s(const flora_encoders_sample_t *prev,
                               const flora_encoders_sample_t *now,
                               float *out_left, float *out_right);

#ifdef __cplusplus
}
#endif
