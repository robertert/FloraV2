/**
 * @file head_fusion.h
 * @brief Zadanie 60 Hz: IMU, fuzja kursu, telemetria ATTITUDE.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "flora_icm20948.h"
#include "flora_link.h"

typedef enum {
    HEAD_FUSION_INIT,       /**< IMU nie odpowiada */
    HEAD_FUSION_BIAS_LOAD,  /**< czyta offset zyro z NVS */
    HEAD_FUSION_BIAS_CAL,   /**< mierzy offset, wymaga bezruchu */
    HEAD_FUSION_ALIGN,      /**< czeka na pierwszy odczyt magnetometru */
    HEAD_FUSION_RUN,        /**< normalna praca */
    HEAD_FUSION_FAULT,      /**< IMU przestal odpowiadac */
} head_fusion_state_t;

/**
 * @brief Startuje zadanie fuzji.
 *
 * @param imu  uchwyt IMU; moze byc NULL, wtedy zadanie czeka w INIT
 * @param down lacze do bryly
 */
esp_err_t head_fusion_start(flora_icm20948_handle_t imu, flora_link_handle_t down);

head_fusion_state_t head_fusion_state(void);
const char *head_fusion_state_name(head_fusion_state_t s);

/** Biezacy kurs w stopniach, do podgladu i do HEAD_STATUS. */
float head_fusion_heading(void);

/** Czy kurs jest wiarygodny (po ALIGN, z dzialajacym IMU). */
bool head_fusion_heading_valid(void);

/** Liczba nieudanych odczytow IMU od startu. */
uint32_t head_fusion_errors(void);
