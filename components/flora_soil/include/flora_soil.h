/**
 * @file flora_soil.h
 * @brief Pojemnosciowy czujnik wilgotnosci gleby na ADC.
 *
 * Sama obsluga odczytu, bez polityki podlewania.
 *
 * JAK CZYTAC WYNIK: czujnik pojemnosciowy oddaje napiecie ODWROTNIE
 * proporcjonalne do wilgotnosci - sucha gleba daje WYZSZE napiecie niz
 * mokra. Dlatego procent wymaga dwupunktowej kalibracji i bez niej
 * flora_soil_read_percent() zwraca blad zamiast zmyslonej liczby.
 *
 * Wartosci surowe i miliwolty sa dostepne zawsze.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    gpio_num_t gpio;      /**< musi lezec na ADC1; kanal wyliczamy sami */

    /**
     * Tlumienie wejscia. ADC_ATTEN_DB_12 daje najszerszy zakres
     * (do ~3,1 V) i o to chodzi przy czujniku zasilanym z 3,3 V.
     * Uwaga: w ESP-IDF 6 ta stala nazywa sie DB_12, w 5.x byla DB_11.
     */
    int atten;

    /** Ile odczytow usrednic. Gleba zmienia sie wolno, warto usredniac. */
    int samples;

    /* --- Kalibracja dwupunktowa ---
     * raw_dry  odczyt czujnika w powietrzu (sucho)
     * raw_wet  odczyt czujnika zanurzonego w wodzie (mokro)
     * Rowne wartosci wylaczaja przeliczanie na procenty. */
    int raw_dry;
    int raw_wet;
} flora_soil_cfg_t;

#define FLORA_SOIL_DEFAULT_CFG()   \
    {                              \
        .atten   = 3,              /* ADC_ATTEN_DB_12 */ \
        .samples = 16,             \
        .raw_dry = 0,              \
        .raw_wet = 0,              \
    }

esp_err_t flora_soil_init(const flora_soil_cfg_t *cfg);
esp_err_t flora_soil_deinit(void);

/** Surowy odczyt ADC, usredniony z cfg.samples probek. */
esp_err_t flora_soil_read_raw(int *raw);

/** Napiecie w miliwoltach. Wymaga dzialajacej kalibracji fabrycznej ADC. */
esp_err_t flora_soil_read_mv(int *mv);

/**
 * @brief Wilgotnosc w procentach, 0 = sucho, 100 = mokro.
 *
 * @return ESP_ERR_INVALID_STATE gdy brak kalibracji (raw_dry == raw_wet).
 *         Wynik jest przycinany do zakresu 0-100.
 */
esp_err_t flora_soil_read_percent(float *percent);

/** Ustawia punkty kalibracyjne w czasie pracy. */
esp_err_t flora_soil_set_calibration(int raw_dry, int raw_wet);

/** true, gdy fabryczna kalibracja ADC jest dostepna i mV maja sens. */
bool flora_soil_has_voltage_calibration(void);

#ifdef __cplusplus
}
#endif
