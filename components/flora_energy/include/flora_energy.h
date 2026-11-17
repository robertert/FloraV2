/**
 * @file flora_energy.h
 * @brief Polityka energii: z napiecia baterii robi tryb pracy i decyzje,
 *        co wolno, a co trzeba wylaczyc.
 *
 * Czysty C bez zaleznosci od ESP-IDF (host/test.c).
 *
 * LiFePO4 4S ma plaska charakterystyke: wiekszosc pojemnosci miesci sie
 * miedzy 13,3 a 12,8 V, a ponizej 12,5 V napiecie leci szybko. Dlatego
 * progi sa nisko i blisko siebie, a filtr dlugi - pojedynczy przysiad
 * pod silnikami nie moze zmienic trybu.
 *
 *   NORMAL    wszystko wolno
 *   SAVE      bez jazdy; maszt tylko wtedy, gdy robot nie spi; sen takze
 *             w dzien - panel ma doladowac baterie
 *   SURVIVAL  bez jazdy i bez podlewania, maszt wylaczony, dlugi sen
 *
 * Probki z chwil duzego poboru (jazda, pompka) sa pomijane: napiecie pod
 * obciazeniem mowi o rezystancji przewodow, nie o stanie baterii.
 *
 * Powrot do lepszego trybu wymaga napiecia o v_hyst wyzszego niz prog,
 * inaczej na granicy tryby przelaczalyby sie co chwile.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    FLORA_ENERGY_NORMAL,
    FLORA_ENERGY_SAVE,
    FLORA_ENERGY_SURVIVAL,
} flora_energy_mode_t;

typedef struct {
    float    v_save;         /**< ponizej: SAVE */
    float    v_survival;     /**< ponizej: SURVIVAL */
    float    v_hyst;         /**< zapas na powrot */
    float    tau_s;          /**< stala czasowa filtra napiecia */
    uint32_t sleep_night_s;  /**< sen w nocy w trybie NORMAL; 0 = nie spi */
    uint32_t sleep_day_s;    /**< sen w dzien w trybie NORMAL; 0 = nie spi */
    uint32_t sleep_save_s;
    uint32_t sleep_survival_s;
} flora_energy_cfg_t;

#define FLORA_ENERGY_DEFAULT_CFG()     \
    {                                  \
        .v_save           = 12.6f,     \
        .v_survival       = 12.2f,     \
        .v_hyst           = 0.3f,      \
        .tau_s            = 30.0f,     \
        .sleep_night_s    = 1800,      \
        .sleep_day_s      = 0,         \
        .sleep_save_s     = 1800,      \
        .sleep_survival_s = 3600,      \
    }

typedef struct {
    float dt_s;
    bool  v_valid;
    float bus_v;
    bool  load_high;       /**< jazda albo pompka - probka pomijana */
    bool  night_known;
    bool  night;
} flora_energy_in_t;

typedef struct {
    flora_energy_mode_t mode;
    float    v_filt;       /**< NAN przed pierwsza probka */
    bool     allow_drive;
    bool     allow_water;
    bool     mast_on;      /**< false = maszt ma byc wylaczony takze na jawie */
    uint32_t sleep_s;      /**< 0 = nie zasypiac */
} flora_energy_out_t;

typedef struct {
    flora_energy_cfg_t cfg;
    flora_energy_mode_t mode;
    float v_filt;
    bool  have_v;
    flora_energy_out_t last;
} flora_energy_t;

void flora_energy_init(flora_energy_t *e, const flora_energy_cfg_t *cfg);
void flora_energy_set_cfg(flora_energy_t *e, const flora_energy_cfg_t *cfg);
flora_energy_out_t flora_energy_update(flora_energy_t *e, const flora_energy_in_t *in);

#ifdef __cplusplus
}
#endif
