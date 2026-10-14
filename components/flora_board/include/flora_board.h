/**
 * @file flora_board.h
 * @brief Mapa pinow jednostki dolnej (bryla): ESP32-S3 DevKitC-1 N16R8.
 *
 * Jedno miejsce prawdy dla pinoutu. Zmieniasz tu, nie w sterownikach.
 *
 * Piny, ktorych NIE wolno uzywac na tej plytce:
 *   0, 3, 45, 46  - strapping (stan przy boot decyduje o trybie startu)
 *   19, 20        - USB D-/D+
 *   35, 36, 37    - PSRAM (wariant N16R8)
 *   43, 44        - UART0 (konsola, idf.py monitor)
 *   38, 48        - dioda RGB
 */
#pragma once

#include "driver/gpio.h"

/* ---- Naped: Cytron MDD10A (PWM + DIR, logika 3,3V) ---- */
#define FLORA_PIN_MOTOR_L_PWM   GPIO_NUM_4
#define FLORA_PIN_MOTOR_L_DIR   GPIO_NUM_5
#define FLORA_PIN_MOTOR_R_PWM   GPIO_NUM_6
#define FLORA_PIN_MOTOR_R_DIR   GPIO_NUM_7

/* ---- Enkodery Halla JGB37-520, 11 PPR na wale silnika ---- */
#define FLORA_PIN_ENC_L_A       GPIO_NUM_15
#define FLORA_PIN_ENC_L_B       GPIO_NUM_16
#define FLORA_PIN_ENC_R_A       GPIO_NUM_17
#define FLORA_PIN_ENC_R_B       GPIO_NUM_18

/* ---- I2C (INA226) ---- */
#define FLORA_PIN_I2C_SDA       GPIO_NUM_8
#define FLORA_PIN_I2C_SCL       GPIO_NUM_9

/* ---- UART do glowicy masztu ---- */
#define FLORA_PIN_HEAD_TX       GPIO_NUM_11
#define FLORA_PIN_HEAD_RX       GPIO_NUM_12

/* ---- Ultradzwieki JSN-SR04T (wspolny TRIG, echo przez dzielnik) ---- */
#define FLORA_PIN_US_TRIG       GPIO_NUM_13
#define FLORA_PIN_US_ECHO_1     GPIO_NUM_14
#define FLORA_PIN_US_ECHO_2     GPIO_NUM_21
#define FLORA_PIN_US_ECHO_3     GPIO_NUM_47

/* ---- Pozostale peryferia ---- */
#define FLORA_PIN_SOIL_ADC      GPIO_NUM_1   /**< wilgotnosc gleby, ADC1 kanal 0 */
#define FLORA_PIN_WATER_LEVEL   GPIO_NUM_10  /**< XKC-Y25, wyjscie podciagniete do 3,3V */
#define FLORA_PIN_DS18B20       GPIO_NUM_41  /**< 1-Wire, + 4,7k do 3,3V */
#define FLORA_PIN_MAST_SWITCH   GPIO_NUM_39  /**< IRF4905 + NPN, klucz 12V masztu */
#define FLORA_PIN_PUMP          GPIO_NUM_40  /**< D4184, pompka perystaltyczna */

/* ---- Parametry mechaniczne napedu ----
 * Enkoder daje 11 impulsow na obrot WALU SILNIKA. PCNT w trybie kwadraturowym
 * zlicza 4 zbocza na impuls, wiec na obrot wyjscia przekladni przypada:
 *     11 * 4 * przelozenie
 * Przy 270:1 to 11880 zliczen na obrot kola.
 */
#define FLORA_ENCODER_PPR         11      /**< impulsow na obrot walu silnika */
#define FLORA_ENCODER_QUAD_MULT   4       /**< dekodowanie x4 (oba zbocza, oba kanaly) */
#define FLORA_GEAR_RATIO          270.0f  /**< TODO: potwierdzic po zakupie (270:1 albo 394:1) */
#define FLORA_WHEEL_DIAMETER_MM   130.0f

#define FLORA_COUNTS_PER_WHEEL_REV \
    (FLORA_ENCODER_PPR * FLORA_ENCODER_QUAD_MULT * FLORA_GEAR_RATIO)

/** Droga w mm na jedno zliczenie enkodera. */
#define FLORA_MM_PER_COUNT \
    ((3.14159265f * FLORA_WHEEL_DIAMETER_MM) / FLORA_COUNTS_PER_WHEEL_REV)

/* ---- Zasilanie ---- */
#define FLORA_SHUNT_MILLIOHM      10.0f  /**< INA226 z rezystorem R010 */
#define FLORA_BATTERY_CELLS       4      /**< LiFePO4 4S, 12,8V nominalnie */
