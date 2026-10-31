/**
 * @file bringup_head.h
 * @brief Test rozruchowy czujnikow glowicy.
 */
#pragma once

#include "driver/i2c_master.h"
#include "flora_bme280.h"
#include "flora_i2cmux.h"
#include "flora_icm20948.h"
#include "flora_veml7700.h"

/**
 * Uchwyty do zainicjalizowania przez test.
 *
 * Test SAM inicjalizuje czujniki i wpisuje uchwyty pod te wskazniki -
 * dzieki temu app_main nie duplikuje konfiguracji, a petla sterowania
 * dostaje te same uchwyty, ktore test juz sprawdzil.
 */
typedef struct {
    i2c_master_bus_handle_t  bus;
    flora_i2cmux_handle_t    mux;        /**< wypelniane przez test */
    flora_i2cmux_port_t     *sun_port;   /**< tablica 4, zyje u wolajacego */
    flora_veml7700_handle_t *sun;        /**< tablica 4, wypelniana przez test */
    flora_bme280_handle_t    bme;
    flora_icm20948_handle_t  imu;
} bringup_head_ctx_t;

void bringup_head_run(const bringup_head_ctx_t *ctx);

/* Uchwyty wyznaczone przez test - petla sterowania bierze je stad. */
flora_i2cmux_handle_t    bringup_head_mux(void);
flora_veml7700_handle_t  bringup_head_sun(int i);
flora_bme280_handle_t    bringup_head_bme(void);
flora_icm20948_handle_t  bringup_head_imu(void);
