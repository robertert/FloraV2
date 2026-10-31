/**
 * @file flora_head_board.h
 * @brief Mapa pinow glowicy masztu: ESP32-S3 z gniazdem kamery.
 *
 * ================== DO POTWIERDZENIA ==================
 * Numery pinow I2C sa WSTEPNE. Plytki S3 z gniazdem kamery (Freenove
 * i podobne) maja wiekszosc wyprowadzen zajeta przez interfejs DVP
 * kamery, wiec wolne piny zaleza od konkretnego modelu.
 *
 * Zanim uruchomisz - sprawdz w dokumentacji swojej plytki, ktore GPIO
 * sa faktycznie wolne, i popraw ponizej. Zle piny oznaczaja albo cisze
 * na magistrali, albo konflikt z kamera.
 *
 * Przy wyborze omijaj:
 *   - piny DVP kamery (zwykle kilkanascie, w tym XCLK, PCLK, VSYNC, HREF, D0-D7)
 *   - 35, 36, 37    PSRAM w wariantach N*R8
 *   - 19, 20        USB
 *   - 43, 44        UART0 (konsola)
 *   - 0, 3, 45, 46  strapping
 * ======================================================
 */
#pragma once

#include "driver/gpio.h"
#include "driver/uart.h"

/* ---- Magistrala I2C czujnikow glowicy ---- */
#define FLORA_HEAD_I2C_SDA      GPIO_NUM_21   /**< potwierdzone na plytce 31.10.2026 */
#define FLORA_HEAD_I2C_SCL      GPIO_NUM_47   /**< potwierdzone na plytce 31.10.2026 */
#define FLORA_HEAD_I2C_FREQ_HZ  100000        /**< 100 kHz: bezpiecznie przy dluzszych przewodach pod kopula */

/* ---- UART w dol masztu (do bryly) ---- */
#define FLORA_HEAD_UART_TX      GPIO_NUM_42   /**< TODO potwierdzic */
#define FLORA_HEAD_UART_RX      GPIO_NUM_41   /**< TODO potwierdzic */
#define FLORA_HEAD_UART_PORT    UART_NUM_1

/* ---- UART w gore (do modulu kamery znacznikow) ----
 * Glowica prowadzi DWA lacza. Modul znacznikow nie ma wlasnego kabla
 * w dol - jest tylko jeden - wiec gada z glowica, a ta przekazuje jego
 * ramki do bryly.
 *
 * Odcinek jest krotki, w obrebie masztu, wiec zaklocenia sa mniejszym
 * problemem niz na dlugim kablu i mozna tu isc szybciej niz 57600.
 */
#define FLORA_HEAD_CAM_TX       GPIO_NUM_38   /**< TODO potwierdzic */
#define FLORA_HEAD_CAM_RX       GPIO_NUM_39   /**< TODO potwierdzic */
#define FLORA_HEAD_CAM_PORT     UART_NUM_2

/* ---- Klucze NVS glowicy ----
 * Wlasne, nie z flora_board.h - ten opisuje bryle, a glowica ma osobna
 * partycje NVS we wlasnym flashu. Limit NVS to 15 znakow na klucz.
 */
#define FLORA_HEAD_NVS_NS        "head"
#define FLORA_HEAD_NVS_GYRO_X    "gyro_bias_x"
#define FLORA_HEAD_NVS_GYRO_Y    "gyro_bias_y"
#define FLORA_HEAD_NVS_GYRO_Z    "gyro_bias_z"
#define FLORA_HEAD_NVS_GYRO_OK   "gyro_cal_ok"

/* ---- Adresy na magistrali ---- */
#define FLORA_HEAD_ADDR_MUX     0x70  /**< TCA9548A, A0-A2 do masy */
#define FLORA_HEAD_ADDR_VEML    0x10  /**< VEML7700, adres staly - stad multiplekser */
#define FLORA_HEAD_ADDR_BME280  0x76  /**< 0x77 gdy SDO podciagniete */
#define FLORA_HEAD_ADDR_ICM     0x68  /**< 0x69 gdy AD0 w stanie wysokim */
#define FLORA_HEAD_ADDR_AK09916 0x0C  /**< magnetometr w ICM-20948, widoczny po wlaczeniu bypass */

/* ---- Kompas sloneczny: ktory VEML na ktorym kanale multipleksera ----
 * Krzyz z przegrodami, czujniki patrza w cztery strony. Przypisanie
 * kanalow do kierunkow zalezy od montazu w uchwycie - sprawdzic przy
 * pierwszym uruchomieniu, zaslaniajac po jednym czujniku.
 */
#define FLORA_HEAD_SUN_CH_N     0
#define FLORA_HEAD_SUN_CH_E     1
#define FLORA_HEAD_SUN_CH_S     2
#define FLORA_HEAD_SUN_CH_W     3
#define FLORA_HEAD_SUN_COUNT    4
