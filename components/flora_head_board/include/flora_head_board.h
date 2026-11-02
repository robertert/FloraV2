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

/* ---- Interfejs DVP kamery ----
 * Pinout plytki Freenove ESP32-S3 WROVER CAM. Oznaczenia Y2..Y9 z
 * dokumentacji Freenove odpowiadaja liniom danych D0..D7.
 *
 * SPRAWDZ, czy masz te sama plytke - kazdy inny modul S3 z gniazdem kamery
 * ma swoj pinout, a wtedy esp_camera_init zglasza blad przy starcie.
 *
 * Te piny sa ZAJETE przez kamere i nie wolno ich uzyc do niczego innego:
 *   4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 15, 16, 17, 18
 * Stad I2C czujnikow na 21/47 i lacza UART na 38-42.
 *
 * SCCB (4/5) to OSOBNA magistrala, obslugiwana przez sterownik kamery.
 * Nie ma nic wspolnego z I2C czujnikow glowicy.
 *
 * OBIEKTYW 160 STOPNI: przy obrazie 320x240 (przekatna/szerokosc = 1,25)
 *   FOV poziomy = 2 * atan( tan(80 st) / 1,25 ) = 155 st
 *   ogniskowa   = 160 / tan(77,6 st) = 35 px
 * czyli PIEC razy krocej niz 297 px dla obiektywu 68-stopniowego z
 * flora_cam_board.h. Pole widzenia rosnie, ale znacznik 300 mm da sie
 * zdekodowac tylko z okolo 1,3 m zamiast 11 m - do znacznikow na palikach
 * ten obiektyw NIE nadaje sie, do patrzenia pod kopule i omijania
 * przeszkod owszem. Przy takim kacie liczy sie tez dystorsja, wiec do
 * jakiejkolwiek geometrii trzeba bedzie kalibracji, nie jednej liczby.
 */
#define FLORA_HEAD_DVP_D0      GPIO_NUM_11   /**< Y2 */
#define FLORA_HEAD_DVP_D1      GPIO_NUM_9    /**< Y3 */
#define FLORA_HEAD_DVP_D2      GPIO_NUM_8    /**< Y4 */
#define FLORA_HEAD_DVP_D3      GPIO_NUM_10   /**< Y5 */
#define FLORA_HEAD_DVP_D4      GPIO_NUM_12   /**< Y6 */
#define FLORA_HEAD_DVP_D5      GPIO_NUM_18   /**< Y7 */
#define FLORA_HEAD_DVP_D6      GPIO_NUM_17   /**< Y8 */
#define FLORA_HEAD_DVP_D7      GPIO_NUM_16   /**< Y9 */
#define FLORA_HEAD_DVP_XCLK    GPIO_NUM_15
#define FLORA_HEAD_DVP_PCLK    GPIO_NUM_13
#define FLORA_HEAD_DVP_VSYNC   GPIO_NUM_6
#define FLORA_HEAD_DVP_HREF    GPIO_NUM_7
#define FLORA_HEAD_DVP_SDA     GPIO_NUM_4    /**< SIOD */
#define FLORA_HEAD_DVP_SCL     GPIO_NUM_5    /**< SIOC */
#define FLORA_HEAD_DVP_PWDN    (-1)          /**< niewyprowadzony na Freenove */
#define FLORA_HEAD_DVP_RESET   (-1)

/**
 * Zegar wejsciowy czujnika.
 *
 * CIEPLO, NIE WYDAJNOSC, decyduje o tej liczbie. Modul OV5640 przy 20 MHz
 * i ciaglym strumieniowaniu robi sie goracy w dotyku w kilka minut, bo
 * czujnik pracuje caly czas z pelna szybkoscia niezaleznie od tego, ile
 * klatek faktycznie odbieramy. Polowa zegara to polowa pracy wewnetrznej
 * i wyraznie nizsza temperatura, a przy wykrywaniu przeszkod i tak
 * potrzebujemy 4 klatek na sekunde, nie trzydziestu.
 *
 * Jesli czujnik nie wystartuje przy tej wartosci, aplikacja powinna
 * sprobowac 20 MHz i zapisac to w logu - tak robi bringup_cam.c.
 */
#define FLORA_HEAD_DVP_XCLK_HZ 10000000

/** Makro wypelniajace flora_camera_pins_t pinami tej plytki. */
#define FLORA_HEAD_DVP_PINS()                   \
    {                                           \
        .pin_pwdn = FLORA_HEAD_DVP_PWDN,        \
        .pin_reset = FLORA_HEAD_DVP_RESET,      \
        .pin_xclk = FLORA_HEAD_DVP_XCLK,        \
        .pin_sccb_sda = FLORA_HEAD_DVP_SDA,     \
        .pin_sccb_scl = FLORA_HEAD_DVP_SCL,     \
        .pin_d7 = FLORA_HEAD_DVP_D7, .pin_d6 = FLORA_HEAD_DVP_D6, \
        .pin_d5 = FLORA_HEAD_DVP_D5, .pin_d4 = FLORA_HEAD_DVP_D4, \
        .pin_d3 = FLORA_HEAD_DVP_D3, .pin_d2 = FLORA_HEAD_DVP_D2, \
        .pin_d1 = FLORA_HEAD_DVP_D1, .pin_d0 = FLORA_HEAD_DVP_D0, \
        .pin_vsync = FLORA_HEAD_DVP_VSYNC,      \
        .pin_href = FLORA_HEAD_DVP_HREF,        \
        .pin_pclk = FLORA_HEAD_DVP_PCLK,        \
        .xclk_freq_hz = FLORA_HEAD_DVP_XCLK_HZ, \
    }

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
