/**
 * @file flora_sonar.h
 * @brief Dalmierze ultradzwiekowe JSN-SR04T (wspolny TRIG, osobne ECHO).
 *
 * Pomiar czasu echa opiera sie na przerwaniu GPIO: znacznik czasu zdejmowany
 * jest w samym ISR, nie w zadaniu, wiec szeregowanie FreeRTOS nie wplywa na
 * wynik. Opoznienie wejscia w przerwanie to kilka mikrosekund i dotyka obu
 * zbocz podobnie, wiec w duzej mierze sie znosi. Przy 58 us na centymetr
 * daje to blad rzedu dziesiatych milimetra - dla omijania przeszkod na
 * trawniku z duzym zapasem.
 *
 * Alternatywa sprzetowa (przechwytywanie MCPWM) byla by dokladniejsza, ale
 * zajmuje kanaly peryferiala i nie skaluje sie tak latwo na 3-4 czujniki
 * wiszace na jednym TRIG.
 *
 * UWAGA NA ZAKRES MARTWY: JSN-SR04T nie mierzy ponizej okolo 20-25 cm.
 * Blizszy obiekt zwraca bledne wyniki albo brak echa - to cecha czujnika,
 * nie bledu kodu.
 *
 * ECHO pracuje na 5 V, wiec na wejsciu ESP32 musi byc dzielnik do 3,3 V.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FLORA_SONAR_MAX_SENSORS 4

/** Zwracane, gdy pomiar sie nie powiodl. */
#define FLORA_SONAR_INVALID (-1.0f)

typedef struct {
    gpio_num_t trig_gpio;                           /**< wspolny dla wszystkich */
    gpio_num_t echo_gpio[FLORA_SONAR_MAX_SENSORS];
    int        count;

    /** Po tylu ms bez echa uznajemy brak odbicia. 40 ms to okolo 6,8 m. */
    uint32_t timeout_ms;

    /** Wymuszona przerwa miedzy pomiarami, zeby echa nie nakladaly sie. */
    uint32_t settle_ms;

    /** Temperatura powietrza do korekty predkosci dzwieku. */
    float temperature_c;

    /* Granice wiarygodnosci. Wynik poza nimi jest odrzucany. */
    float min_distance_mm;
    float max_distance_mm;
} flora_sonar_cfg_t;

#define FLORA_SONAR_DEFAULT_CFG()       \
    {                                   \
        .timeout_ms      = 40,          \
        .settle_ms       = 60,          \
        .temperature_c   = 20.0f,       \
        .min_distance_mm = 200.0f,      \
        .max_distance_mm = 6000.0f,     \
    }

esp_err_t flora_sonar_init(const flora_sonar_cfg_t *cfg);
esp_err_t flora_sonar_deinit(void);

/**
 * @brief Pojedynczy pomiar. Blokuje do echa albo do uplywu timeout_ms.
 *
 * @return ESP_ERR_TIMEOUT gdy nie wrocilo echo (brak przeszkody w zasiegu
 *         albo urwany przewod), ESP_ERR_INVALID_RESPONSE gdy wynik wypadl
 *         poza granicami wiarygodnosci.
 */
esp_err_t flora_sonar_measure(int index, float *distance_mm);

/**
 * @brief Mediana z kilku pomiarow.
 *
 * Ultradzwieki lubia pojedyncze odczyty kompletnie od czapy - odbicie od
 * trawy, zaklocenie od silnikow. Mediana z 3-5 probek usuwa je bez
 * wygladzania prawdziwych zmian, w przeciwienstwie do sredniej.
 */
esp_err_t flora_sonar_measure_median(int index, int samples, float *distance_mm);

/** Aktualizuje temperature do korekty predkosci dzwieku. */
void flora_sonar_set_temperature(float celsius);

/** Predkosc dzwieku przy obecnej temperaturze, w m/s. */
float flora_sonar_speed_of_sound(void);

#ifdef __cplusplus
}
#endif
