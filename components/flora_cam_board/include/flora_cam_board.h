/**
 * @file flora_cam_board.h
 * @brief Mapa pinow modulu kamery przeszkod (trzeci modul, w maszcie).
 *
 * Modul ma JEDNO zadanie: wykrywanie przeszkod kamera z obiektywem
 * 160 stopni patrzaca w dol (flora_ground) i wysylanie 16 sektorow
 * (FLORA_MSG_OBSTACLES) w dol. Od 21.11.2026 - wczesniej robila to
 * glowica, a ten modul szukal znacznikow AprilTag (te sa teraz na
 * glowicy). Nie ma czujnikow I2C ani nawigacji.
 *
 * TOPOLOGIA: w maszcie biegnie tylko JEDEN kabel w dol, wiec ten modul nie
 * ma wlasnego lacza z bryla. Gada z glowica, a ta przekazuje jego ramki
 * dalej (patrz flora_link_relay).
 *
 *   MODUL PRZESZKOD  --UART--  GLOWICA  --UART--  BRYLA
 *
 * ================== DO POTWIERDZENIA ==================
 * Numery pinow sa WSTEPNE. Na plytkach z gniazdem kamery wiekszosc
 * wyprowadzen zajmuje interfejs DVP, wiec wolne piny zaleza od modelu.
 * Sprawdz dokumentacje swojej plytki przed uruchomieniem.
 * ======================================================
 */
#pragma once

#include "driver/gpio.h"
#include "driver/uart.h"

/* ---- UART w dol (do glowicy) ---- */
#define FLORA_CAM_UART_TX    GPIO_NUM_38   /**< TODO potwierdzic */
#define FLORA_CAM_UART_RX    GPIO_NUM_39   /**< TODO potwierdzic */
#define FLORA_CAM_UART_PORT  UART_NUM_1

/** Krotki odcinek w maszcie - mozna szybciej niz lacze w dol. */
#define FLORA_CAM_BAUD       115200

/* ---- Interfejs DVP kamery ----
 * Pinout plytki Freenove ESP32-S3 WROVER CAM. Oznaczenia Y2..Y9 z
 * dokumentacji Freenove odpowiadaja liniom danych D0..D7.
 *
 * SPRAWDZ, czy masz te sama plytke - inny modul S3 z gniazdem kamery ma
 * inny pinout, a wtedy esp_camera_init zglosi blad przy starcie.
 *
 * Te piny sa ZAJETE przez kamere i nie wolno ich uzyc do niczego innego:
 *   4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 15, 16, 17, 18
 * Stad lacze UART w dol wypada na 38/39.
 */
#define FLORA_CAM_PIN_D0      GPIO_NUM_11   /**< Y2 */
#define FLORA_CAM_PIN_D1      GPIO_NUM_9    /**< Y3 */
#define FLORA_CAM_PIN_D2      GPIO_NUM_8    /**< Y4 */
#define FLORA_CAM_PIN_D3      GPIO_NUM_10   /**< Y5 */
#define FLORA_CAM_PIN_D4      GPIO_NUM_12   /**< Y6 */
#define FLORA_CAM_PIN_D5      GPIO_NUM_18   /**< Y7 */
#define FLORA_CAM_PIN_D6      GPIO_NUM_17   /**< Y8 */
#define FLORA_CAM_PIN_D7      GPIO_NUM_16   /**< Y9 */
#define FLORA_CAM_PIN_XCLK    GPIO_NUM_15
#define FLORA_CAM_PIN_PCLK    GPIO_NUM_13
#define FLORA_CAM_PIN_VSYNC   GPIO_NUM_6
#define FLORA_CAM_PIN_HREF    GPIO_NUM_7
#define FLORA_CAM_PIN_SDA     GPIO_NUM_4    /**< SIOD */
#define FLORA_CAM_PIN_SCL     GPIO_NUM_5    /**< SIOC */
#define FLORA_CAM_PIN_PWDN    (-1)          /**< niewyprowadzony na Freenove */
#define FLORA_CAM_PIN_RESET   (-1)

/** Makro wypelniajace flora_camera_pins_t pinami tej plytki. */
#define FLORA_CAM_PINS()                        \
    {                                           \
        .pin_pwdn = FLORA_CAM_PIN_PWDN,         \
        .pin_reset = FLORA_CAM_PIN_RESET,       \
        .pin_xclk = FLORA_CAM_PIN_XCLK,         \
        .pin_sccb_sda = FLORA_CAM_PIN_SDA,      \
        .pin_sccb_scl = FLORA_CAM_PIN_SCL,      \
        .pin_d7 = FLORA_CAM_PIN_D7, .pin_d6 = FLORA_CAM_PIN_D6, \
        .pin_d5 = FLORA_CAM_PIN_D5, .pin_d4 = FLORA_CAM_PIN_D4, \
        .pin_d3 = FLORA_CAM_PIN_D3, .pin_d2 = FLORA_CAM_PIN_D2, \
        .pin_d1 = FLORA_CAM_PIN_D1, .pin_d0 = FLORA_CAM_PIN_D0, \
        .pin_vsync = FLORA_CAM_PIN_VSYNC,       \
        .pin_href = FLORA_CAM_PIN_HREF,         \
        .pin_pclk = FLORA_CAM_PIN_PCLK,         \
        .xclk_freq_hz = 20000000,               \
    }

/* ---- Wykrywanie przeszkod ---- */

/**
 * Wysokosc obiektywu nad ziemia w milimetrach. ZERO znaczy "nie zmierzono".
 *
 * Bez tej liczby detektor przeszkod dziala, ale nie umie zamienic
 * pierscienia obrazu na odleglosc i kazdy sektor idzie na kabel jako zero,
 * czyli "nie wiem" - co dla straznika w bryle znaczy "nie jedz". Czyli:
 * bez pomiaru wykrywanie dziala w logu, a nie dziala w praktyce.
 *
 * Geometria nadiru to `odleglosc = h * tan(promien_px / f)`, czyli skaluje
 * sie WPROST PROPORCJONALNIE do h. Blad wysokosci przeklada sie jeden do
 * jednego na blad odleglosci - zmierz miarka, nie szacuj.
 *
 * Tagcam nie ma sieci, wiec zmiana wymaga przeflashowania (dawniej na
 * glowicy szlo przez http://<ip>/height i NVS).
 */
#define FLORA_CAM_HEIGHT_MM 0

/**
 * Zegar wejsciowy czujnika. OV5640 przy 20 MHz robi sie goracy w kilka
 * minut, a do przeszkod wystarcza ~4 kl/s. Gdy czujnik nie ruszy przy tej
 * wartosci, main.c probuje 20 MHz.
 */
#define FLORA_CAM_XCLK_HZ 10000000
