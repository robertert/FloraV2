/**
 * @file flora_water.h
 * @brief Polityka podlewania: kiedy i ile podlac, na podstawie wilgotnosci
 *        gleby.
 *
 * Czysty C bez zaleznosci od ESP-IDF (host/test.c). Nie dotyka pompki:
 * zglasza zadanie dawki, a warstwa wyzej (hub_water) uruchamia pompke
 * i odsyla, ile faktycznie poszlo. Twarde zabezpieczenia (sufit czasu,
 * praca na sucho) zostaja w flora_pump - to tutaj jest tylko decyzja.
 *
 * ZASADA: histereza na medianie wilgotnosci.
 *   mediana < dry_pct  -> trzeba podlac
 *   mediana > wet_pct  -> dosc
 * Miedzy dawkami czas wsiakania soak_s: woda dochodzi do czujnika po
 * minutach, wiec podlewanie "az czujnik pokaze mokro" przelaloby doniczke.
 *
 * ZABEZPIECZENIA:
 *   - limit dobowy w ml (okno 24 h od pierwszej dawki),
 *   - brak wody w zbiorniku wstrzymuje, powrot wody wznawia,
 *   - brak reakcji: max_doses_no_rise dawek z rzedu bez wzrostu wilgotnosci
 *     o rise_min_pct to czujnik wyjety z doniczki, spadnieta rurka albo
 *     zatkana pompka - zatrzask do flora_water_reset_fault(),
 *   - czujnik bez odczytu dluzej niz sensor_timeout_s wstrzymuje,
 *   - pompka, ktora nie konczy dawki przez pump_timeout_s, to awaria.
 *
 * DAWKA W KAWALKACH: pompka ma sufit czasu jednej pracy, wiec dawka moze
 * isc w kilku uruchomieniach. Wejscie max_chunk_ml mowi, ile wolno naraz.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FLORA_WATER_MEDIAN_N 7

typedef enum {
    FLORA_WATER_OK,          /**< wilgotno, nic do roboty */
    FLORA_WATER_DRY,         /**< sucho, czeka na zgode (robot jedzie) */
    FLORA_WATER_DOSING,      /**< dawka w toku */
    FLORA_WATER_SOAK,        /**< wsiakanie po dawce */
    FLORA_WATER_NO_WATER,    /**< pusty zbiornik */
    FLORA_WATER_LIMIT,       /**< wyczerpany limit dobowy */
    FLORA_WATER_NO_SENSOR,   /**< brak odczytu wilgotnosci */
    FLORA_WATER_FAULT,       /**< zatrzask: brak reakcji albo pompka */
} flora_water_state_t;

typedef enum {
    FLORA_WATER_FAULT_NONE,
    FLORA_WATER_FAULT_NO_RISE,
    FLORA_WATER_FAULT_PUMP,
} flora_water_fault_t;

typedef struct {
    float    dry_pct;
    float    wet_pct;
    float    dose_ml;
    uint32_t soak_s;
    float    daily_max_ml;
    uint8_t  max_doses_no_rise;
    float    rise_min_pct;
    uint32_t sensor_timeout_s;
    uint32_t pump_timeout_s;
} flora_water_cfg_t;

#define FLORA_WATER_DEFAULT_CFG()     \
    {                                 \
        .dry_pct           = 35.0f,   \
        .wet_pct           = 55.0f,   \
        .dose_ml           = 50.0f,   \
        .soak_s            = 600,     \
        .daily_max_ml      = 500.0f,  \
        .max_doses_no_rise = 3,       \
        .rise_min_pct      = 2.0f,    \
        .sensor_timeout_s  = 60,      \
        .pump_timeout_s    = 180,     \
    }

typedef struct {
    uint32_t now_s;          /**< zegar monotoniczny w sekundach */
    bool     soil_valid;     /**< soil_pct to swiezy odczyt */
    float    soil_pct;
    bool     water_present;
    bool     allowed;        /**< wolno podlewac teraz (robot stoi, tryb pozwala) */
    bool     pump_busy;      /**< pompka pracuje */
    /** Ile ml poszlo od poprzedniego kroku (po zakonczeniu pracy pompki). */
    float    delivered_ml;
    /** Najwieksza porcja na jedno uruchomienie pompki; <= 0 = bez limitu. */
    float    max_chunk_ml;
} flora_water_in_t;

typedef struct {
    flora_water_state_t state;
    flora_water_fault_t fault;
    /** > 0: uruchom pompke na tyle ml. Powtarzane co krok, az pump_busy. */
    float    request_ml;
    float    median_pct;     /**< NAN przed pierwszym pelnym oknem */
    float    day_ml;         /**< podane w biezacym oknie dobowym */
    float    remaining_ml;   /**< reszta biezacej dawki */
} flora_water_out_t;

typedef struct {
    flora_water_cfg_t cfg;
    flora_water_state_t state;
    flora_water_fault_t fault;

    float    win[FLORA_WATER_MEDIAN_N];
    int      win_n, win_head;
    float    median;
    bool     have_median;
    uint32_t last_valid_s;
    bool     seen_valid;

    bool     need;           /**< histereza: trzeba podlewac */
    float    remaining_ml;
    uint32_t t_state_s;      /**< wejscie w DOSING / SOAK */
    float    pct_before;     /**< mediana przed dawka */
    uint8_t  no_rise;

    float    day_ml;
    uint32_t day_start_s;
    bool     day_open;

    flora_water_out_t last;
} flora_water_t;

void flora_water_init(flora_water_t *w, const flora_water_cfg_t *cfg);
void flora_water_set_cfg(flora_water_t *w, const flora_water_cfg_t *cfg);

/** Jeden krok, wolany co ok. 1 s. */
flora_water_out_t flora_water_update(flora_water_t *w, const flora_water_in_t *in);

/** Zdejmuje zatrzask FAULT (po usunieciu przyczyny). */
void flora_water_reset_fault(flora_water_t *w);

#ifdef __cplusplus
}
#endif
