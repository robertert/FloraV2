/**
 * @file flora_camera.h
 * @brief Kamera DVP w odcieniach szarosci.
 *
 * Cienkie opakowanie nad espressif/esp32-camera. Dwa powody istnienia:
 * trzyma pinout DVP w jednym miejscu i wymusza ustawienia, ktore na
 * ESP32-S3 sa jedyne sensowne dla przetwarzania obrazu.
 *
 * DLACZEGO ODCIENIE SZAROSCI, NIE JPEG: wykrywanie znacznikow pracuje na
 * jasnosci pikseli, wiec JPEG trzeba by dekodowac z powrotem - czas i pamiec
 * wyrzucone. Format GRAYSCALE daje gotowy bufor jednobajtowy na piksel.
 *
 * DLACZEGO MALA ROZDZIELCZOSC: AprilTag na S3 przy 320x240 daje pojedyncze
 * klatki na sekunde, a wyzej robi sie bezuzyteczny. Przy znacznikach na
 * stacjonarnych palikach i wolno jezdzacym robocie 1-2 klatki na sekunde
 * calkowicie wystarczaja do korekty pozycji.
 *
 * TRZY FORMATY, JEDNA SCIEZKA INICJALIZACJI. Odcienie szarosci zostaja
 * domyslne (znaczniki), RGB565 doszlo dla wykrywania przeszkod, JPEG dla
 * podgladu na komputerze:
 *
 *   GRAY    wykrywanie znacznikow - detektor pracuje na jasnosci
 *   RGB565  wykrywanie przeszkod - na betonie i kostce cien trzeba odrzucac
 *           po BARWIE, nie po jasnosci, wiec szarosc tu nie wystarcza
 *   JPEG    podglad i zbieranie danych - czujnik pakuje sprzetowo, wiec po
 *           sieci leci kilkadziesiat kB zamiast setek
 *
 * STEROWNIK PRZYJMUJE JEDEN FORMAT NA URUCHOMIENIE. Zmiana wymaga
 * flora_camera_deinit() i ponownego init, a dwa tryby naraz nie istnieja -
 * podglad JPEG i detekcja wykluczaja sie na tym samym ukladzie.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Piny interfejsu DVP. Zalezne od plytki. */
typedef struct {
    int pin_pwdn, pin_reset, pin_xclk, pin_sccb_sda, pin_sccb_scl;
    int pin_d7, pin_d6, pin_d5, pin_d4, pin_d3, pin_d2, pin_d1, pin_d0;
    int pin_vsync, pin_href, pin_pclk;
    int xclk_freq_hz;
} flora_camera_pins_t;

typedef enum {
    FLORA_CAMERA_GRAY,      /**< 1 B/px, bez kompresji - znaczniki */
    FLORA_CAMERA_RGB565,    /**< 2 B/px, bez kompresji - przeszkody */
    FLORA_CAMERA_JPEG,      /**< kompresja w czujniku - podglad przez siec */
} flora_camera_format_t;

typedef struct {
    flora_camera_pins_t pins;

    /** Domyslnie FLORA_CAMERA_GRAY, zeby stare wywolania nie zmienily sensu. */
    flora_camera_format_t format;

    /**
     * Szerokosc i wysokosc obrazu. Dla znacznikow 320x240 to gorna granica
     * rozsadku; dla podgladu JPEG wolno siegnac po 640x480 albo 800x600, bo
     * czujnik kompresuje sprzetowo i nic tego nie przelicza na ESP.
     */
    uint16_t width;
    uint16_t height;

    /**
     * Jakosc JPEG, 10-12 to czysty obraz, 20 to widoczne bloki.
     * Ignorowane w pozostalych formatach. Zero bierze 12.
     */
    uint8_t  jpeg_quality;

    /**
     * Ile buforow klatek. Dwa pozwalaja przetwarzac jedna, gdy druga sie
     * naped - przy jednym kamera czeka na zwolnienie i tempo spada.
     */
    uint8_t  fb_count;
} flora_camera_cfg_t;

/** 320x240, dwa bufory w PSRAM, 20 MHz zegara. */
#define FLORA_CAMERA_DEFAULT_CFG()  \
    {                               \
        .width    = 320,            \
        .height   = 240,            \
        .fb_count = 2,              \
    }

/** Jedna klatka. Bufor nalezy do sterownika - oddaj go przez release(). */
typedef struct {
    const uint8_t *data;
    /**
     * Dlugosc danych w bajtach. Przy JPEG to JEDYNY sposob poznania
     * rozmiaru - klatka jest skompresowana, wiec width*height nic nie mowi.
     */
    uint32_t len;
    uint16_t width, height;
    flora_camera_format_t format;
    int64_t  timestamp_us;
    void    *_internal;
} flora_camera_frame_t;

esp_err_t flora_camera_init(const flora_camera_cfg_t *cfg);
esp_err_t flora_camera_deinit(void);

/**
 * @brief Pobiera klatke. Blokuje do gotowosci.
 *
 * KAZDA pobrana klatka MUSI wrocic przez flora_camera_release(), inaczej
 * sterownik wyczerpie bufory i kolejne pobranie zawiesi sie na zawsze.
 */
esp_err_t flora_camera_grab(flora_camera_frame_t *out, uint32_t timeout_ms);
void      flora_camera_release(flora_camera_frame_t *frame);

bool flora_camera_is_ready(void);

/**
 * @brief Czy w ogole da sie probowac autofokusu.
 *
 * Falsz znaczy, ze obsluga nie zostala wkompilowana (CONFIG_CAMERA_AF_SUPPORT)
 * albo ten czujnik jej nie ma. Prawda NIE znaczy, ze modul ma silniczek:
 * sterownik OV5640 odpowiada na to pytanie na sztywno "tak", bo mowi o
 * ukladzie, a nie o obiektywie. Jedynym dowodem na silniczek jest udana
 * proba, czyli flora_camera_focus_trigger().
 */
bool flora_camera_focus_available(void);

/**
 * @brief Wyzwala pojedyncze ustawienie ostrosci i czeka na wynik.
 *
 * Pierwsze wywolanie laduje firmware AF do czujnika przez SCCB, co trwa,
 * stad hojny limit czasu. Ladowanie jest unievazniane przez deinit kamery,
 * wiec po uspieniu i pobudce zrobi sie samo od nowa.
 *
 * @param out_focused czujnik zglosil ostrosc; NULL gdy nieinteresujace
 * @return ESP_ERR_NOT_SUPPORTED gdy brak obslugi, ESP_ERR_TIMEOUT gdy
 *         firmware sie nie zaladowal albo ostrosc nie przyszla
 */
esp_err_t flora_camera_focus_trigger(uint32_t timeout_ms, bool *out_focused);

#ifdef __cplusplus
}
#endif
