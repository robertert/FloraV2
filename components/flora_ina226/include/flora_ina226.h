/**
 * @file flora_ina226.h
 * @brief INA226: pomiar pradu i napiecia szyny 12V (bocznik high-side).
 *
 * Komponent NIE tworzy magistrali I2C - przyjmuje gotowy uchwyt, zeby dalo sie
 * powiesic na niej kolejne uklady bez konfliktu o port.
 *
 * Ograniczenie sprzetowe, o ktore latwo sie potknac: INA226 mierzy na boczniku
 * maksymalnie +-81,92 mV. Przy rezystorze R010 (10 mOhm) daje to sufit
 * 8,19 A - czyli MNIEJ niz bezpiecznik 7,5-10 A. Powyzej tej wartosci odczyt
 * sie nasyca, nie rosnie dalej. flora_ina226_init zglasza to w logu.
 */
#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Adres przy A0=A1=GND. */
#define FLORA_INA226_ADDR_DEFAULT   0x40

/** Maksymalne napiecie bocznika wg noty katalogowej. */
#define FLORA_INA226_MAX_SHUNT_MV   81.92f

typedef struct {
    uint8_t  i2c_address;
    uint32_t i2c_speed_hz;

    float shunt_milliohm;          /**< rezystor bocznikowy, dla R010 to 10.0 */
    float max_expected_current_a;  /**< zakres pomiarowy, wyznacza rozdzielczosc */

    /**
     * Usrednianie sprzetowe. Silniki PWM-owane 20 kHz wstrzykuja w pomiar
     * sporo smieci, wiec domyslnie 16 probek. Czas konwersji rosnie liniowo:
     * 16 * (1,1 ms + 1,1 ms) = okolo 35 ms na komplet.
     */
    uint16_t averaging_samples;    /**< 1,4,16,64,128,256,512,1024 */
} flora_ina226_cfg_t;

#define FLORA_INA226_DEFAULT_CFG()              \
    {                                           \
        .i2c_address            = FLORA_INA226_ADDR_DEFAULT, \
        .i2c_speed_hz           = 400000,       \
        .shunt_milliohm         = 10.0f,        \
        .max_expected_current_a = 8.0f,         \
        .averaging_samples      = 16,           \
    }

typedef struct flora_ina226_s *flora_ina226_handle_t;

/** Komplet odczytow pobrany jednym przejsciem. */
typedef struct {
    float bus_voltage_v;   /**< napiecie szyny wzgledem masy */
    float shunt_voltage_mv;
    float current_a;       /**< dodatni = pobor z baterii */
    float power_w;
} flora_ina226_reading_t;

/**
 * @brief Inicjalizuje uklad na podanej magistrali.
 *
 * Sprawdza identyfikator producenta, wiec blad ESP_ERR_NOT_FOUND oznacza
 * realny problem z adresem albo okablowaniem, a nie tylko brak odpowiedzi.
 */
esp_err_t flora_ina226_init(i2c_master_bus_handle_t bus,
                            const flora_ina226_cfg_t *cfg,
                            flora_ina226_handle_t *out);

esp_err_t flora_ina226_deinit(flora_ina226_handle_t h);

esp_err_t flora_ina226_read(flora_ina226_handle_t h, flora_ina226_reading_t *out);

/** Pojedyncze wielkosci, gdy nie potrzeba kompletu. */
esp_err_t flora_ina226_read_bus_voltage(flora_ina226_handle_t h, float *volts);
esp_err_t flora_ina226_read_current(flora_ina226_handle_t h, float *amps);

/** Rozdzielczosc pomiaru pradu wyliczona przy inicjalizacji, w amperach na bit. */
float flora_ina226_current_lsb(flora_ina226_handle_t h);

#ifdef __cplusplus
}
#endif
