/**
 * @file flora_cam_board.h
 * @brief Mapa pinow modulu kamery znacznikow (trzeci modul, w maszcie).
 *
 * Modul ma JEDNO zadanie: szukac znacznikow AprilTag na palikach w rogach
 * obszaru roboczego i wysylac surowe detekcje w dol. Nie ma czujnikow
 * I2C ani nawigacji - te sa w glowicy.
 *
 * TOPOLOGIA: w maszcie biegnie tylko JEDEN kabel w dol, wiec ten modul nie
 * ma wlasnego lacza z bryla. Gada z glowica, a ta przekazuje jego ramki
 * dalej (patrz flora_link_relay).
 *
 *   MODUL ZNACZNIKOW  --UART--  GLOWICA  --UART--  BRYLA
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

/* ---- Parametry wyszukiwania znacznikow ----
 * Do ustalenia po wybraniu rozmiaru obszaru roboczego: od niego zalezy
 * fizyczny rozmiar znacznikow na palikach, a od tego zasieg wykrywania.
 */
/**
 * Bok czarnego kwadratu znacznika. 300 mm wybrane pod maksymalny zasieg:
 * AprilTag potrzebuje okolo 8 pikseli na bok, zeby zdekodowac 36h11, co
 * przy ogniskowej 297 px daje
 *     zasieg = 297 * 300 / 8 = 11,1 m
 * Przy 150 mm byloby to 5,6 m.
 */
#define FLORA_CAM_TAG_SIZE_MM   300
/**
 * Zasieg wyliczony z geometrii, do potwierdzenia pomiarem w polu -
 * swiatlo, ostrosc obiektywu i kontrast wydruku moga go skrocic.
 */
#define FLORA_CAM_MAX_RANGE_MM  11000

/**
 * Decymacja przed szukaniem krawedzi. JEDEN znaczy pelna rozdzielczosc,
 * czyli maksymalny zasieg kosztem tempa - detektor pracuje na
 * czterokrotnie wiekszym obrazie niz przy decymacji 2.
 *
 * Spodziewane tempo to 0,3-0,5 klatki na sekunde. Przy robocie jadacym
 * ponizej 0,2 m/s znaczy to poprawke pozycji co 2-3 sekundy i kilkadziesiat
 * centymetrow przejechanych miedzy nimi - filtr Kalmana pokrywa te luke
 * odometria. Biblioteka przyjmuje tez wartosci niecalkowite, wiec 1,5 jest
 * sensownym kompromisem, gdyby tempo okazalo sie za niskie.
 */
#define FLORA_CAM_QUAD_DECIMATE 1.0f

/**
 * ASYMETRIA DOKLADNOSCI, wazna dla filtra na dole.
 *
 * Azymut liczymy z polozenia srodka znacznika, wiec jego rozdzielczosc to
 * 0,19 stopnia na CALYM zasiegu. Odleglosc liczymy z pozornego rozmiaru,
 * a ten maleje z kwadratem - blad jednego piksela w pomiarze boku daje:
 *
 *    1 m  ->   11 mm bledu
 *    3 m  ->   98 mm
 *    6 m  ->  379 mm
 *    9 m  ->  826 mm
 *   11 m  -> 1209 mm
 *
 * Czyli na granicy zasiegu odleglosc ma ponad metr niepewnosci, a azymut
 * jest nadal dokladny. Filtr powinien ufac KIERUNKOWI znacznie bardziej
 * niz ODLEGLOSCI - dlatego pole `quality` w flora_tag_t maleje razem
 * z pozornym rozmiarem.
 */

/**
 * Ogniskowa w pikselach, wyliczona z katalogowego kata widzenia obiektywu.
 *
 * Obiektyw ma 68 stopni PO PRZEKATNEJ. Przy obrazie 320x240 daje to
 *   FOV poziomy = 2 * atan( tan(34 st) / (przekatna/szerokosc) ) = 56,7 st
 *   ogniskowa   = 160 / tan(28,35 st) = 297 px
 *
 * To WARTOSC WYJSCIOWA, nie koncowa. Katalogowy kat jest podawany
 * z tolerancja, a montaz czujnika wzgledem obiektywu tez wnosi blad.
 * Uscislij pomiarem: postaw znacznik na zmierzonej odleglosci, odczytaj
 * z logu pozorny bok w pikselach i policz
 *     ogniskowa = bok_px * odleglosc_mm / bok_znacznika_mm
 * Przy tym obiektywie dystorsja jest niewielka (to nie szerokokatny),
 * wiec jeden pomiar wystarczy.
 */
#define FLORA_CAM_FOCAL_PX      297.0f
