/**
 * @file flora_tags.h
 * @brief Wykrywanie znacznikow AprilTag i przeliczanie na odleglosc i azymut.
 *
 * Opakowuje port biblioteki AprilRobotics i zamienia detekcje w pikselach
 * na wielkosci fizyczne.
 *
 * WYDAJNOSC: na ESP32-S3 przy 320x240 spodziewaj sie pojedynczych klatek
 * na sekunde. To nie jest problem w tym zastosowaniu - znaczniki sa na
 * stacjonarnych palikach, a robot jedzie wolno, wiec jedna poprawka pozycji
 * na sekunde z nadmiarem wystarcza filtrowi Kalmana do korekty dryfu
 * odometrii.
 *
 * DOKLADNOSC ZALEZY OD KALIBRACJI: odleglosc liczymy z pozornego rozmiaru
 * znacznika, a azymut z przesuniecia jego srodka wzgledem osi obrazu. Oba
 * wymagaja ogniskowej w pikselach, ktorej NIE DA SIE zgadnac z katalogu
 * obiektywu z sensowna dokladnoscia. Trzeba ja wyznaczyc pomiarem -
 * patrz flora_tags_focal_from_measurement().
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FLORA_TAGS_MAX_DETECTIONS 8

/** Rodzina znacznikow. */
typedef enum {
    FLORA_TAGS_FAMILY_36H11,  /**< domyslna; najmniej falszywych wykryc */
    FLORA_TAGS_FAMILY_25H9,   /**< mniej bitow, czytelna z wiekszej odleglosci */
    FLORA_TAGS_FAMILY_16H5,   /**< najmniej bitow, ale duzo falszywych wykryc */
} flora_tags_family_t;

typedef struct {
    flora_tags_family_t family;

    /**
     * Zmniejszenie obrazu przed szukaniem krawedzi. 2 oznacza dwukrotnie
     * mniejszy obraz, czyli okolo czterokrotnie szybciej, kosztem zasiegu.
     * Na S3 bez tego detektor jest zbyt wolny.
     */
    float quad_decimate;

    /** Wygladzanie przed progowaniem. 0 wylacza; przy szumie pomaga 0,8. */
    float quad_sigma;

    /**
     * Ile bitow bledu wolno poprawic. ZERO to wlasciwy wybor w polu:
     * kazdy dopuszczony bit gwaltownie zwieksza liczbe falszywych wykryc,
     * a falszywy znacznik wprowadza filtr Kalmana w blad gorzej niz brak
     * jakiejkolwiek detekcji.
     */
    int max_hamming;

    /* --- geometria, do przeliczenia na wielkosci fizyczne --- */
    float tag_size_mm;    /**< bok czarnego kwadratu znacznika */
    float focal_px;       /**< ogniskowa w pikselach, DO WYZNACZENIA POMIAREM */

    /** Minimalna ostrosc krawedzi; ponizej odrzucamy jako niepewne. */
    float min_decision_margin;
} flora_tags_cfg_t;

/**
 * quad_decimate = 2 i max_hamming = 0 to nastawy pod prace w polu:
 * szybciej i bez falszywych wykryc, kosztem zasiegu.
 */
#define FLORA_TAGS_DEFAULT_CFG()           \
    {                                      \
        .family              = FLORA_TAGS_FAMILY_36H11, \
        .quad_decimate       = 2.0f,       \
        .quad_sigma          = 0.0f,       \
        .max_hamming         = 0,          \
        .tag_size_mm         = 150.0f,     \
        .focal_px            = 0.0f,       \
        .min_decision_margin = 30.0f,      \
    }

/** Jedna detekcja, przeliczona na wielkosci fizyczne. */
typedef struct {
    uint16_t id;
    float    distance_mm;    /**< z pozornego rozmiaru; 0 gdy brak ogniskowej */
    float    bearing_deg;    /**< dodatni w prawo od osi obrazu */
    float    center_x, center_y;  /**< srodek w pikselach, do diagnozy */
    float    decision_margin;
    uint8_t  quality;        /**< 0..100 */
} flora_tag_detection_t;

typedef struct {
    flora_tag_detection_t items[FLORA_TAGS_MAX_DETECTIONS];
    uint8_t count;
    uint32_t process_ms;     /**< ile zajelo przetworzenie klatki */
} flora_tags_result_t;

esp_err_t flora_tags_init(const flora_tags_cfg_t *cfg);
esp_err_t flora_tags_deinit(void);

/**
 * @brief Szuka znacznikow w klatce w odcieniach szarosci.
 *
 * Blokuje na czas przetwarzania - na S3 setki milisekund. Wolaj z zadania
 * o niskim priorytecie, zeby nie zaglodzic telemetrii.
 */
esp_err_t flora_tags_detect(const uint8_t *gray, uint16_t w, uint16_t h,
                            flora_tags_result_t *out);

/**
 * @brief Wyznacza ogniskowa w pikselach z jednego pomiaru.
 *
 * Postaw znacznik na ZMIERZONEJ odleglosci, odczytaj z logu jego pozorny
 * rozmiar w pikselach i podaj tu obie liczby. Zwrocona wartosc wpisz
 * w `focal_px`.
 *
 * Bez tego odleglosc i azymut sa niewyznaczone - katalogowy kat widzenia
 * obiektywu jest zbyt niedokladny, a przy obiektywach szerokokatnych
 * dochodzi dystorsja.
 *
 * @param known_distance_mm zmierzona odleglosc do znacznika
 * @param observed_px       pozorny bok znacznika w pikselach
 * @param tag_size_mm       rzeczywisty bok znacznika
 */
static inline float flora_tags_focal_from_measurement(float known_distance_mm,
                                                      float observed_px,
                                                      float tag_size_mm)
{
    if (tag_size_mm <= 0.0f) {
        return 0.0f;
    }
    return observed_px * known_distance_mm / tag_size_mm;
}

#ifdef __cplusplus
}
#endif
