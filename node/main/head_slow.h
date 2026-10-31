/**
 * @file head_slow.h
 * @brief Zadanie 1 Hz: kompas sloneczny, BME280, HEAD_STATUS.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "flora_bme280.h"
#include "flora_i2cmux.h"
#include "flora_link.h"
#include "flora_veml7700.h"

/** Stan zdrowia jednego czujnika. */
typedef enum {
    HEAD_SENSOR_ABSENT,   /**< nigdy nie odpowiedzial */
    HEAD_SENSOR_OK,
    HEAD_SENSOR_SUSPECT,  /**< 1..3 bledy z rzedu, dane nadal uzywane */
    HEAD_SENSOR_DEAD,     /**< odpytywany rzadko, tylko by wykryc powrot */
} head_sensor_state_t;

typedef struct {
    flora_i2cmux_handle_t   mux;
    flora_veml7700_handle_t sun[4];
    flora_bme280_handle_t   bme;
    flora_link_handle_t     down;
} head_slow_cfg_t;

esp_err_t head_slow_start(const head_slow_cfg_t *cfg);

/** Mapa bitowa FLORA_SENSOR_* czujnikow faktycznie odpowiadajacych. */
uint8_t head_slow_sensors_ok(void);

/** Liczba nieudanych odczytow od startu, na wszystkich czujnikach. */
uint16_t head_slow_errors(void);

const char *head_sensor_state_name(head_sensor_state_t s);
