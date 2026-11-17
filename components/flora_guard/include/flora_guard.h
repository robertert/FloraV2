/**
 * @file flora_guard.h
 * @brief Straznik napedu: z czujnikow robi zgode na jazde osobno w przod,
 *        w tyl i na obrot.
 *
 * Czysty C bez zaleznosci od ESP-IDF (host/test.c). Nie dotyka silnikow -
 * daje tylko werdykt, a jedyne miejsce, ktore go egzekwuje, to hub_drive.
 *
 * STRAZNICY (kazdy ma bit w cfg.mask, wylaczony nie istnieje):
 *   CAM      sektory przeszkod z kamery glowicy. Przeszkoda w korytarzu
 *            jazdy blizej niz stop_front_mm blokuje jazde W PRZOD. Sektor
 *            "nie wiem" w stozku na wprost liczy sie jak zajety, tak samo
 *            brak swiezej ramki - kamera, ktora milczy, niczego nie
 *            potwierdza.
 *   SONAR    dwa przednie blokuja jazde w przod, tylny jazde w tyl.
 *            Brak echa NIE blokuje: dalmierz nie odroznia "pusto" od
 *            "odpiety", wiec sam nie moze byc jedynym zrodlem zgody -
 *            do jazdy w przod wlacz tez CAM.
 *   TILT     przechyl albo pochylenie ponad prog zatrzymuje wszystko
 *            (maszt podnosi srodek ciezkosci).
 *   VOLTAGE  napiecie szyny ponizej progu przez v_hold_ms zatrzymuje
 *            jazde, zanim BMS odetnie cala reszte. Czas podtrzymania
 *            przepuszcza przysiady przy rozruchu silnikow.
 *   STALE    ATTITUDE starsze niz att_max_age_ms: autonomia stoi, reczna
 *            jazda zostaje z limitem mocy crawl_limit.
 *
 * Obrot w miejscu blokuja tylko TILT i VOLTAGE: przeszkoda z przodu ma
 * pozwolic sie odwrocic i odjechac.
 *
 * Zdjecie blokady wymaga release_ms nieprzerwanego spokoju - bez tego
 * przeszkoda na granicy progu dawalaby szarpanie start / stop.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FLORA_GUARD_SECTORS 16
#define FLORA_GUARD_SONARS  3

/** Znaczenie wartosci w sektorze - jak w flora_msgs.h (FLORA_OBSTACLE_*). */
#define FLORA_GUARD_SECTOR_UNKNOWN 0
#define FLORA_GUARD_SECTOR_CLEAR   254

/** Indeksy dalmierzy. */
#define FLORA_GUARD_SONAR_FRONT_L 0
#define FLORA_GUARD_SONAR_FRONT_R 1
#define FLORA_GUARD_SONAR_REAR    2

/** Bity straznikow: w cfg.mask wlaczone, w out.reasons aktywne. */
#define FLORA_GUARD_CAM      (1u << 0)
#define FLORA_GUARD_SONAR    (1u << 1)
#define FLORA_GUARD_TILT     (1u << 2)
#define FLORA_GUARD_VOLTAGE  (1u << 3)
#define FLORA_GUARD_STALE    (1u << 4)
#define FLORA_GUARD_ALL      0x1Fu

typedef struct {
    uint32_t mask;

    float stop_front_mm;     /**< przeszkoda blizej = stop jazdy w przod */
    float stop_rear_mm;      /**< jak wyzej dla cofania (tylny sonar) */
    float corridor_half_mm;  /**< polowa szerokosci korytarza jazdy (bryla + zapas) */
    float unknown_cone_deg;  /**< "nie wiem" w +-tylu stopniach od osi = zajete */
    float fov_deg;           /**< szerokosc wachlarza sektorow kamery */
    uint32_t obs_max_age_ms; /**< starsza ramka przeszkod = brak wiedzy */

    float tilt_stop_deg;
    float tilt_clear_deg;

    float v_stop;            /**< V */
    float v_clear;           /**< V, powrot z blokady */
    uint32_t v_hold_ms;

    uint32_t att_max_age_ms;
    float crawl_limit;       /**< 0..1, limit mocy przy starych danych */

    uint32_t release_ms;
} flora_guard_cfg_t;

/**
 * Progi domyslne. corridor_half_mm: bryla ma 360 mm szerokosci, kola
 * wystaja do ok. 200 mm od osi, plus 50 mm zapasu. stop_front_mm liczone
 * od osi kamery (maszt, x = 295), czyli ok. 330 mm przed przednia sciana.
 */
#define FLORA_GUARD_DEFAULT_CFG()        \
    {                                    \
        .mask             = FLORA_GUARD_ALL, \
        .stop_front_mm    = 350.0f,      \
        .stop_rear_mm     = 300.0f,      \
        .corridor_half_mm = 250.0f,      \
        .unknown_cone_deg = 30.0f,       \
        .fov_deg          = 160.0f,      \
        .obs_max_age_ms   = 1000,        \
        .tilt_stop_deg    = 20.0f,       \
        .tilt_clear_deg   = 15.0f,       \
        .v_stop           = 11.8f,       \
        .v_clear          = 12.2f,       \
        .v_hold_ms        = 2000,        \
        .att_max_age_ms   = 500,         \
        .crawl_limit      = 0.3f,        \
        .release_ms       = 500,         \
    }

typedef struct {
    uint32_t now_ms;         /**< zegar monotoniczny, zawijanie dozwolone */

    bool     att_valid;      /**< kiedykolwiek odebrano ATTITUDE */
    uint32_t att_age_ms;
    float    pitch_deg, roll_deg;

    bool     obs_valid;      /**< jest ramka i jej pole valid != 0 */
    uint32_t obs_age_ms;
    uint8_t  sectors[FLORA_GUARD_SECTORS];  /**< od lewej do prawej */

    /** Najblizsze echo z okna filtra [mm]; NAN = brak echa. */
    float    sonar_mm[FLORA_GUARD_SONARS];

    bool     bus_v_valid;
    float    bus_v;
} flora_guard_in_t;

typedef struct {
    bool     allow_fwd, allow_rev, allow_turn;
    /** false = autonomia ma stac (stare dane, przechyl, napiecie). */
    bool     autonomy_ok;
    float    power_limit;    /**< 0..1 */
    uint32_t reasons;        /**< bity aktywnych straznikow */
    /** Najblizsza przeszkoda w korytarzu z przodu [mm]; < 0 gdy brak. */
    float    front_mm;
    float    rear_mm;
} flora_guard_out_t;

typedef struct {
    flora_guard_cfg_t cfg;
    bool     tilt_active;
    bool     v_active;
    bool     v_low;          /**< napiecie pod progiem, trwa odliczanie */
    uint32_t v_low_since_ms;
    /* Ostatnia chwila z surowa blokada kazdego kierunku + czy byla. */
    bool     blk_seen[3];
    uint32_t blk_ms[3];
    flora_guard_out_t last;
} flora_guard_t;

void flora_guard_init(flora_guard_t *g, const flora_guard_cfg_t *cfg);

/** Podmiana progow bez zerowania stanu (zmiana ustawien w locie). */
void flora_guard_set_cfg(flora_guard_t *g, const flora_guard_cfg_t *cfg);

/** Jeden krok. Wynik zostaje tez w g->last. */
flora_guard_out_t flora_guard_update(flora_guard_t *g, const flora_guard_in_t *in);

/**
 * Czy wolno zadac kolom (left, right) - jednostka dowolna, liczy sie znak
 * i proporcja. Zwraca mnoznik 0..1: 0 = zabronione, inaczej limit mocy.
 * Zero na obu kolach jest zawsze dozwolone (zwraca 1).
 */
float flora_guard_permit(const flora_guard_out_t *out, float left, float right);

#ifdef __cplusplus
}
#endif
