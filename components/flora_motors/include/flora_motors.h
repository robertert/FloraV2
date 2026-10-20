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

/* ================= WYKRYWANIE UTYKU =================
 *
 * Stojacy silnik pod napieciem ciagnie prad zwarciowy, bo bez obrotow nie
 * powstaje sila przeciwelektromotoryczna, ktora normalnie go ogranicza.
 * Cala ta moc zamienia sie w cieplo i w kilkadziesiat sekund potrafi
 * rozkleic magnesy albo zetrzec panewke - tak zginal prawy silnik 19.10.2026.
 *
 * Straznik porownuje zadana moc z przyrostem zliczen enkodera. Brak ruchu
 * mimo mocy przez dluzej niz grace_ms odcina OBA silniki i zatrzaskuje
 * blokade, ktorej nie da sie zdjac przypadkiem.
 *
 * Dziala na sprzetowym timerze, nie w zadaniu aplikacji - zawieszona petla
 * sterowania go nie wylaczy.
 *
 * flora_motors celowo NIE zalezy od flora_encoders: zliczenia dostaje
 * callbackiem, wiec komponent nadal da sie uzyc z dowolnym zrodlem
 * informacji o ruchu albo zupelnie bez niego.
 */

typedef enum {
    FLORA_MOTORS_WHEEL_LEFT,
    FLORA_MOTORS_WHEEL_RIGHT,
} flora_motors_wheel_t;

/** Ma zwrocic biezaca, narastajaca liczbe zliczen dla wskazanego kola. */
typedef int64_t (*flora_motors_counts_cb_t)(flora_motors_wheel_t wheel, void *ctx);

/** Wolane po wykryciu utyku, z kontekstu zadania timera. */
typedef void (*flora_motors_stall_cb_t)(flora_motors_wheel_t wheel,
                                        float power, void *ctx);

typedef struct {
    flora_motors_counts_cb_t counts_cb;   /**< wymagany */
    void                    *counts_ctx;
    flora_motors_stall_cb_t  on_stall;    /**< opcjonalny */
    void                    *stall_ctx;

    uint32_t check_ms;    /**< co ile sprawdzac ruch */
    uint32_t grace_ms;    /**< ile tolerowac bezruch, zanim uznamy utyk */
    float    min_power;   /**< ponizej tej mocy nie sprawdzamy (silnik ma prawo stac) */
    int64_t  min_counts;  /**< tyle zliczen na okres uznajemy juz za ruch */
} flora_motors_stall_cfg_t;

/**
 * grace_ms = 400 to kompromis: tyle wystarczy na rozbieg z bezwladnosci
 * i na pokonanie tarcia spoczynkowego, a jednoczesnie zablokowany silnik
 * grzeje sie przez niecala sekunde, czyli bez szkody.
 */
#define FLORA_MOTORS_STALL_DEFAULT_CFG()      {                                             .check_ms   = 50,                         .grace_ms   = 400,                        .min_power  = 15.0f,                      .min_counts = 5,                      }

esp_err_t flora_motors_stall_guard_enable(const flora_motors_stall_cfg_t *cfg);
esp_err_t flora_motors_stall_guard_disable(void);

/** true, gdy blokada jest zatrzasnieta po wykrytym utyku. */
bool flora_motors_is_stalled(void);

/** Kolo, ktore utknelo. Sensowne tylko gdy flora_motors_is_stalled(). */
flora_motors_wheel_t flora_motors_stalled_wheel(void);

/**
 * @brief Zdejmuje blokade po utyku.
 *
 * Wolaj dopiero po usunieciu przyczyny. Blokada jest zatrzaskiwana celowo:
 * automatyczne ponawianie po utyku to najprostszy sposob na spalenie
 * silnika, bo kazda proba to kolejna porcja ciepla.
 */
esp_err_t flora_motors_clear_stall(void);

#ifdef __cplusplus
}
#endif
