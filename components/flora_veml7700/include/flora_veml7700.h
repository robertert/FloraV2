/**
 * @file flora_veml7700.h
 * @brief Czujnik natezenia swiatla VEML7700.
 *
 * Sterownik nie wie nic o multiplekserze - przed kazdym dostepem wola
 * opcjonalny callback, ktory ma przygotowac magistrale. Dzieki temu ten
 * sam kod obsluguje czujnik wpiety wprost i cztery sztuki za TCA9548A.
 *
 * DOBOR CZULOSCI: czujnik ma zakres od ulamkow luksa do dziesiatek tysiecy,
 * ale nie naraz - trzeba wybrac wzmocnienie i czas calkowania. Do kompasu
 * slonecznego, ktory patrzy w niebo, wlasciwe sa NAJMNIEJSZE wzmocnienie
 * i NAJKROTSZY czas (1/8 i 25 ms), bo pelne slonce to ponad 100 000 lx.
 * Przy ustawieniach na slabe swiatlo odczyt nasyci sie i wszystkie cztery
 * czujniki pokaza to samo maksimum, czyli kompas przestanie dzialac.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Wzmocnienie toru analogowego. */
typedef enum {
    FLORA_VEML_GAIN_1,      /**< x1    */
    FLORA_VEML_GAIN_2,      /**< x2, najczulsze */
    FLORA_VEML_GAIN_1_8,    /**< x1/8, do jasnego slonca */
    FLORA_VEML_GAIN_1_4,    /**< x1/4 */
} flora_veml_gain_t;

/** Czas calkowania. Krotszy to mniejsza czulosc i szybszy odczyt. */
typedef enum {
    FLORA_VEML_IT_25MS,
    FLORA_VEML_IT_50MS,
    FLORA_VEML_IT_100MS,
    FLORA_VEML_IT_200MS,
    FLORA_VEML_IT_400MS,
    FLORA_VEML_IT_800MS,
} flora_veml_it_t;

/** Wolane przed kazda transakcja; ma przygotowac magistrale (np. wybrac kanal). */
typedef esp_err_t (*flora_veml_prepare_cb_t)(void *ctx);

typedef struct {
    uint8_t  address;
    uint32_t speed_hz;

    flora_veml_gain_t gain;
    flora_veml_it_t   integration_time;

    /**
     * Korekta nieliniowosci wg noty aplikacyjnej producenta. Powyzej
     * mniej wiecej 1000 lx czujnik zanizą odczyt i bez niej blad rosnie
     * z jasnoscia - przy slonecznym niebie bardzo wyraznie.
     */
    bool apply_correction;

    flora_veml_prepare_cb_t prepare;   /**< NULL gdy czujnik wisi wprost na magistrali */
    void                   *prepare_ctx;
} flora_veml7700_cfg_t;

#define FLORA_VEML7700_DEFAULT_CFG()              \
    {                                             \
        .address          = 0x10,                 \
        .speed_hz         = 100000,               \
        .gain             = FLORA_VEML_GAIN_1_8,  \
        .integration_time = FLORA_VEML_IT_25MS,   \
        .apply_correction = true,                 \
    }

typedef struct flora_veml7700_s *flora_veml7700_handle_t;

esp_err_t flora_veml7700_init(i2c_master_bus_handle_t bus,
                              const flora_veml7700_cfg_t *cfg,
                              flora_veml7700_handle_t *out);
esp_err_t flora_veml7700_deinit(flora_veml7700_handle_t h);

/** Natezenie swiatla w luksach. */
esp_err_t flora_veml7700_read_lux(flora_veml7700_handle_t h, float *lux);

/** Surowy odczyt kanalu ALS, bez przeliczania. */
esp_err_t flora_veml7700_read_raw(flora_veml7700_handle_t h, uint16_t *raw);

/** Kanal bialy - szersze pasmo niz ALS, przydatny przy swietle slonecznym. */
esp_err_t flora_veml7700_read_white(flora_veml7700_handle_t h, float *lux);

/** Zmienia czulosc w czasie pracy. */
esp_err_t flora_veml7700_set_sensitivity(flora_veml7700_handle_t h,
                                         flora_veml_gain_t gain,
                                         flora_veml_it_t it);

/** Luksy na jedno zliczenie przy obecnych ustawieniach. */
float flora_veml7700_resolution(flora_veml7700_handle_t h);

/** Czas, po ktorym odczyt jest wiarygodny po zmianie ustawien albo wlaczeniu. */
uint32_t flora_veml7700_settle_ms(flora_veml7700_handle_t h);

#ifdef __cplusplus
}
#endif
