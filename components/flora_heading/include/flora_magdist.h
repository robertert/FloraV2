/**
 * @file flora_magdist.h
 * @brief Wykrywanie zaklocen pola magnetycznego.
 *
 * Kalibracja usuwa znieksztalcenia STALE (od samego robota). Zaklocenia
 * zmienne - stalowy przedmiot obok trasy, kabel pod ziemia, prad silnikow,
 * magnes w kieszeni - trzeba wykrywac na biezaco i wtedy NIE ufac
 * magnetometrowi. Trzy niezalezne objawy:
 *
 *   1 wartosc pola poziomego rozni sie od tej z kalibracji o > 15%,
 *   2 inklinacja (kat pola wzgledem poziomu) rozni sie o > 8 st,
 *   3 filtr kursu odrzuca magnetometr dluzej niz 2 s - kierunek pola
 *     nie zgadza sie z tym, co mowi zyroskop.
 * 1 i 2 lapia zaklocenia, ktore zmieniaja pole; 3 lapie te, ktore tylko je
 * obracaja (ich wartosc i inklinacja moga zostac w normie).
 * Bez kalibracji (brak odniesienia) dzialaja tylko 3 i zgrubna wiarygodnosc.
 *
 * Histereza: powrot do "czysto" dopiero po 2 s bez objawow - na granicy
 * zaklocenia flaga nie mruga.
 *
 * Czysty C, bez ESP-IDF - sprawdzane w host/test.c.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float    h_tol;        /**< wzgledna tolerancja pola poziomego (0.15) */
    float    dip_tol_deg;  /**< tolerancja inklinacji (8) */
    uint32_t reject_ms;    /**< ile odrzucen z rzedu to zaklocenie (2000) */
    uint32_t clear_ms;     /**< ile spokoju do zdjecia flagi (2000) */
} flora_magdist_cfg_t;

#define FLORA_MAGDIST_DEFAULT_CFG()   \
    {                                 \
        .h_tol       = 0.15f,         \
        .dip_tol_deg = 8.0f,          \
        .reject_ms   = 2000,          \
        .clear_ms    = 2000,          \
    }

/** Ktory objaw zadzialal ostatnio - do diagnostyki. */
enum {
    FLORA_MAGDIST_CAUSE_NONE   = 0,
    FLORA_MAGDIST_CAUSE_FIELD  = 1,
    FLORA_MAGDIST_CAUSE_DIP    = 2,
    FLORA_MAGDIST_CAUSE_GYRO   = 3,
    FLORA_MAGDIST_CAUSE_RANGE  = 4,   /**< poza zakresem ziemskim, bez kalibracji */
};

typedef struct {
    flora_magdist_cfg_t cfg;
    bool     have_ref;
    float    h_ref, dip_ref;
    bool     disturbed;
    uint8_t  cause;
    uint32_t reject_ms_run;
    uint32_t good_ms;
} flora_magdist_t;

void flora_magdist_init(flora_magdist_t *d, const flora_magdist_cfg_t *cfg);

/** Odniesienie z kalibracji; have=false gdy kalibracji brak. */
void flora_magdist_set_ref(flora_magdist_t *d, bool have, float h_ref_ut, float dip_ref_deg);

/**
 * Jeden pomiar magnetometru.
 *
 * @param dt_ms       czas od poprzedniego pomiaru
 * @param h_ut        pole poziome po korekcji i poziomowaniu
 * @param dip_deg     inklinacja
 * @param total_ut    wartosc calkowita (zgrubna wiarygodnosc bez kalibracji)
 * @param gyro_agrees czy filtr kursu przyjalby ten pomiar
 * @return true gdy pole jest zaklocone i magnetometrowi nie wolno ufac
 */
bool flora_magdist_step(flora_magdist_t *d, uint32_t dt_ms, float h_ut, float dip_deg,
                        float total_ut, bool gyro_agrees);

#ifdef __cplusplus
}
#endif
