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

typedef struct {
    flora_camera_pins_t pins;

    /** Szerokosc i wysokosc obrazu. 320x240 to gorna granica rozsadku. */
    uint16_t width;
    uint16_t height;

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
    const uint8_t *data;    /**< jeden bajt na piksel */
    uint16_t width, height;
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

#ifdef __cplusplus
}
#endif
