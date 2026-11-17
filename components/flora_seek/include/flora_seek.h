/**
 * @file flora_seek.h
 * @brief Ucieczka z cienia: wykrywa, ze doniczka stoi w cieniu, i szuka
 *        jasniejszego miejsca probnymi odcinkami w granicach obszaru.
 *
 * Czysty C bez zaleznosci od ESP-IDF (host/test.c z symulowanym ogrodem).
 * Nie jezdzi sam: wystawia polecenie "jedz do (x, y)" i czeka na wynik
 * od warstwy wyzej (flora_mission -> hub_pilot).
 *
 * KIEDY SZUKA (wszystko naraz):
 *   - jest dzien: slonce nad min_elev_deg, a bez zegara lux ponad min_lux,
 *   - cien trwa shade_hold_s: brak swiatla bezposredniego (maly kontrast
 *     miedzy czterema czujnikami krzyza) ALBO jasnosc spadla ponizej
 *     shade_ratio wzgledem niedawnego maksimum,
 *   - od poprzedniego ruchu minelo min_dwell_s,
 *   - minela przerwa po nieudanym szukaniu (backoff),
 *   - obszar roboczy jest ustawiony i nic nie wstrzymuje (hold).
 *
 * JAK SZUKA: punkty probne na luku wokol miejsca startu, w promieniu
 * probe_len_mm, co step_deg, po stronie PRZECIWNEJ do slonca. Przeszkoda
 * rzucajaca cien stoi miedzy robotem a sloncem, wiec cien konczy sie
 * w kierunku od slonca albo na boki - jazda w strone slonca prowadzi
 * pod sam plot. Kolejne punkty sa sasiednie, wiec przejazd miedzy nimi
 * jest krotszy niz powrot do srodka.
 *
 * W kazdym punkcie robot stoi settle_s i usrednia jasnosc. Jesli jest
 * swiatlo bezposrednie i jasnosc wzrosla gain_ratio razy - zostaje od
 * razu. Po ostatnim punkcie jedzie do najlepszego z odwiedzonych (takze
 * z powrotem na start, gdy nic lepszego nie bylo).
 *
 * POCHMURNY DZIEN wyglada jak cien wszedzie: szukanie nic nie da. Dlatego
 * po szukaniu bez zysku przerwa rosnie (backoff_s, podwajana do
 * backoff_max_s) i wraca do zera dopiero po udanym szukaniu.
 *
 * OGRANICZENIE: stan zyje w RAM. Po glebokim snie liczniki (postoj, cien,
 * przerwa) startuja od nowa - pierwsze szukanie najwczesniej po
 * min_dwell_s od wybudzenia.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FLORA_SEEK_MAX_PROBES 8

typedef enum {
    FLORA_SEEK_WATCH,      /**< stoi i obserwuje */
    FLORA_SEEK_MOVING,     /**< jedzie do punktu probnego */
    FLORA_SEEK_MEASURE,    /**< stoi w punkcie i mierzy */
    FLORA_SEEK_RETURN,     /**< jedzie do najlepszego punktu */
} flora_seek_state_t;

typedef enum {
    FLORA_SEEK_NAV_IDLE,
    FLORA_SEEK_NAV_BUSY,
    FLORA_SEEK_NAV_ARRIVED,
    FLORA_SEEK_NAV_FAILED,
} flora_seek_nav_t;

typedef enum {
    FLORA_SEEK_CMD_NONE,
    FLORA_SEEK_CMD_GOTO,   /**< jedz do (gx, gy) */
    FLORA_SEEK_CMD_STOP,   /**< przerwij jazde */
} flora_seek_cmd_t;

typedef enum {
    FLORA_SEEK_RES_NONE,
    FLORA_SEEK_RES_IMPROVED,   /**< stoi w jasniejszym miejscu */
    FLORA_SEEK_RES_NO_GAIN,    /**< nic lepszego w zasiegu */
    FLORA_SEEK_RES_ABORTED,    /**< przerwane (hold) */
    FLORA_SEEK_RES_BLOCKED,    /**< zaden przejazd sie nie udal */
} flora_seek_result_t;

typedef struct {
    bool  area_valid;
    float xmin, ymin, xmax, ymax;   /**< obszar roboczy w ukladzie mapy, mm */
    float margin_mm;                /**< tyle od granicy trzyma sie srodek robota */

    float   probe_len_mm;
    float   min_len_mm;      /**< krotszy odcinek (po przycieciu do obszaru) pomijany */
    float   step_deg;
    uint8_t max_probes;

    float    shade_ratio;
    float    contrast_min;   /**< kontrast krzyza, od ktorego swiatlo jest bezposrednie */
    uint32_t shade_hold_s;
    uint32_t min_dwell_s;
    uint32_t settle_s;
    float    gain_ratio;
    float    min_lux;
    float    min_elev_deg;
    float    lux_tau_s;      /**< filtr jasnosci i kontrastu */
    float    ref_tau_s;      /**< zanik "niedawnego maksimum" */
    uint32_t backoff_s;
    uint32_t backoff_max_s;
} flora_seek_cfg_t;

#define FLORA_SEEK_DEFAULT_CFG()      \
    {                                 \
        .area_valid    = false,       \
        .margin_mm     = 400.0f,      \
        .probe_len_mm  = 1500.0f,     \
        .min_len_mm    = 500.0f,      \
        .step_deg      = 45.0f,       \
        .max_probes    = 5,           \
        .shade_ratio   = 0.5f,        \
        .contrast_min  = 0.25f,       \
        .shade_hold_s  = 600,         \
        .min_dwell_s   = 1800,        \
        .settle_s      = 20,          \
        .gain_ratio    = 1.3f,        \
        .min_lux       = 1000.0f,     \
        .min_elev_deg  = 10.0f,       \
        .lux_tau_s     = 30.0f,       \
        .ref_tau_s     = 7200.0f,     \
        .backoff_s     = 3600,        \
        .backoff_max_s = 14400,       \
    }

typedef struct {
    uint32_t now_s;          /**< zegar monotoniczny, krok ok. 1 s */
    bool     lux_valid;      /**< swiezy pomiar z glowicy */
    float    lux;            /**< srednia z czterech czujnikow krzyza */
    float    contrast;       /**< 0..1, patrz flora_seek_contrast() */
    bool     elev_known;     /**< jest czas UTC, wiec znamy pozycje slonca */
    float    sun_elev_deg;
    bool     sun_dir_valid;
    float    sun_map_deg;    /**< kierunek NA slonce w ukladzie mapy, CCW od osi X */
    float    x_mm, y_mm;     /**< poza robota */
    flora_seek_nav_t nav;
    bool     hold;           /**< nie zaczynaj, a trwajace szukanie przerwij */
} flora_seek_in_t;

typedef struct {
    flora_seek_state_t  state;
    flora_seek_cmd_t    cmd;       /**< jednorazowe, w kroku w ktorym zapada */
    float               gx, gy;
    flora_seek_result_t result;    /**< jednorazowe, w kroku konczacym szukanie */
    bool     shade;                /**< warunek cienia spelniony teraz */
    bool     day;
    float    lux_filt;
    float    lux_ref;
    uint32_t shade_s;              /**< jak dlugo trwa cien */
    uint32_t wait_s;               /**< ile jeszcze do mozliwego szukania (postoj, przerwa) */
    uint8_t  probe;                /**< numer biezacego punktu, od 1 */
} flora_seek_out_t;

typedef struct {
    flora_seek_cfg_t cfg;
    flora_seek_state_t state;

    bool     started;
    uint32_t last_s;
    float    lux_f, con_f, ref;
    bool     have_lux;

    bool     force;          /**< jednorazowe wymuszenie szukania */
    bool     in_shade;
    uint32_t shade_since_s;
    uint32_t dwell_since_s;
    uint32_t backoff_cur_s;
    uint32_t backoff_until_s;
    bool     backoff_on;

    /* Biezace szukanie. */
    float    ox, oy;         /**< srodek luku */
    float    lux0;           /**< jasnosc na starcie */
    float    base_deg;       /**< kierunek srodka luku */
    uint8_t  next;           /**< indeks kolejnego punktu */
    uint8_t  probe;
    uint8_t  moved_ok, moved_fail;
    float    tx, ty;         /**< cel biezacego przejazdu */
    float    bx, by, blux;   /**< najlepszy punkt i jego jasnosc */
    bool     best_is_here;
    uint32_t meas_since_s;
    float    meas_sum, meas_con;
    uint32_t meas_n;

    flora_seek_out_t last;
} flora_seek_t;

void flora_seek_init(flora_seek_t *s, const flora_seek_cfg_t *cfg);

/** Podmiana ustawien bez zerowania stanu. */
void flora_seek_set_cfg(flora_seek_t *s, const flora_seek_cfg_t *cfg);

/** Liczy postoj od nowa i porzuca trwajace szukanie (bez polecenia STOP). */
void flora_seek_restart(flora_seek_t *s, uint32_t now_s);

/**
 * Wymusza szukanie przy najblizszym kroku, bez czekania na cien, postoj
 * i przerwe - do prob w terenie. Obszar roboczy i hold nadal obowiazuja.
 */
void flora_seek_force(flora_seek_t *s);

flora_seek_out_t flora_seek_update(flora_seek_t *s, const flora_seek_in_t *in);

/**
 * Kontrast krzyza: (max - min) / (max + min) z czterech natezen.
 * Swiatlo rozproszone oswietla wszystkie cwiartki podobnie (blisko 0),
 * slonce bezposrednie zostawia czesc w cieniu przegrody (blisko 1).
 */
float flora_seek_contrast(const float lux[4]);

#ifdef __cplusplus
}
#endif
