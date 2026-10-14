/**
 * @file flora_motors.h
 * @brief Naped rozniczkowy na sterowniku Cytron MDD10A (PWM + DIR).
 *
 * MDD10A pracuje w trybie znak-modul: pin DIR ustala kierunek, PWM zadaje moc.
 * Sterownik nie zna geometrii robota - przyjmuje moc -100..100 na kolo.
 * Predkosc w jednostkach fizycznych bedzie zadaniem warstwy PID wyzej.
 */
#pragma once

#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    gpio_num_t pwm_gpio;
    gpio_num_t dir_gpio;
    bool       invert;   /**< true odwraca kierunek - silniki sa montowane lustrzanie */
} flora_motor_cfg_t;

typedef struct {
    flora_motor_cfg_t left;
    flora_motor_cfg_t right;

    /**
     * Czestotliwosc PWM. 20 kHz trzyma piszczenie powyzej slyszalnosci.
     * Uwaga na zaleznosc od rozdzielczosci: freq * 2^bits musi zmiescic sie
     * w zrodle zegara (80 MHz APB), wiec przy 20 kHz maksimum to 11 bitow.
     */
    uint32_t  pwm_freq_hz;
    ledc_timer_bit_t pwm_resolution;
    ledc_timer_t     timer;
    ledc_channel_t   channel_left;
    ledc_channel_t   channel_right;
} flora_motors_cfg_t;

/** Sensowne wartosci domyslne dla MDD10A na ESP32-S3. */
#define FLORA_MOTORS_DEFAULT_CFG()                   \
    {                                                \
        .pwm_freq_hz   = 20000,                      \
        .pwm_resolution = LEDC_TIMER_11_BIT,         \
        .timer         = LEDC_TIMER_0,               \
        .channel_left  = LEDC_CHANNEL_0,             \
        .channel_right = LEDC_CHANNEL_1,             \
    }

/**
 * @brief Inicjalizuje oba kanaly napedu.
 *
 * Po powrocie oba silniki stoja (wypelnienie 0), a piny DIR sa w stanie niskim.
 */
esp_err_t flora_motors_init(const flora_motors_cfg_t *cfg);

esp_err_t flora_motors_deinit(void);

/**
 * @brief Zadaje moc obu kol naraz.
 *
 * @param left  -100..100, wartosci poza zakresem sa przycinane
 * @param right -100..100
 *
 * Znak ustawia kierunek, wartosc bezwzgledna wypelnienie PWM. Wartosc 0
 * zatrzymuje silnik wybiegiem (MDD10A nie hamuje przy wypelnieniu zero).
 */
esp_err_t flora_motors_set(float left, float right);

/** Zatrzymuje oba silniki wybiegiem. Rownowazne flora_motors_set(0, 0). */
esp_err_t flora_motors_stop(void);

/** Ostatnio zadane wartosci, przydatne do logowania i do PID-a. */
void flora_motors_get_last(float *out_left, float *out_right);

#ifdef __cplusplus
}
#endif
