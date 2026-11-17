/**
 * @file flora_mission.h
 * @brief Maszyna stanow doniczki: stoi w sloncu, w cieniu szuka jasniejszego
 *        miejsca, ustepuje podlewaniu, niskiej baterii i awariom.
 *
 * Czysty C bez zaleznosci od ESP-IDF (host/test.c). Nie dotyka sprzetu:
 * dostaje stan robota w strukturze, oddaje polecenie dla pilota i zgody
 * dla reszty (podlewanie, sen). Szukanie slonca robi flora_seek; misja
 * decyduje, kiedy wolno mu dzialac.
 *
 *   IDLE          misja wylaczona
 *   DWELL         stoi, obserwuje swiatlo
 *   SEEK_MOVE     jedzie do punktu probnego albo wraca do najlepszego
 *   SEEK_MEASURE  stoi w punkcie probnym i mierzy
 *   WATER         stoi, trwa podlewanie
 *   LOW_BATT      stoi, energia nie pozwala jechac
 *   HOLD          stoi, cos wstrzymuje autonomie (straznik, reczna jazda,
 *                 brak pozy)
 *   FAULT         zatrzask: utyk napedu albo kolejne szukania bez zadnego
 *                 udanego przejazdu; zdejmuje flora_mission_reset()
 *
 * Podlewanie jest dozwolone zawsze, gdy robot stoi - takze w FAULT:
 * roslina nie moze uschnac dlatego, ze kolo sie zaklinowalo.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "flora_seek.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    FLORA_MISSION_IDLE,
    FLORA_MISSION_DWELL,
    FLORA_MISSION_SEEK_MOVE,
    FLORA_MISSION_SEEK_MEASURE,
    FLORA_MISSION_WATER,
    FLORA_MISSION_LOW_BATT,
    FLORA_MISSION_HOLD,
    FLORA_MISSION_FAULT,
} flora_mission_state_t;

typedef enum {
    FLORA_MISSION_FAULT_NONE,
    FLORA_MISSION_FAULT_STALL,     /**< straznik utyku zatrzasnal naped */
    FLORA_MISSION_FAULT_BLOCKED,   /**< szukania bez zadnego udanego przejazdu */
} flora_mission_fault_t;

typedef struct {
    /** Tyle szukan z rzedu zakonczonych BLOCKED daje FAULT. */
    uint8_t max_blocked_seeks;
} flora_mission_cfg_t;

#define FLORA_MISSION_DEFAULT_CFG() { .max_blocked_seeks = 2 }

typedef struct {
    bool  enabled;
    /* Wejscia flora_seek (pole hold misja ustawia sama). */
    flora_seek_in_t seek;
    bool  pose_ok;         /**< filtr pozycji dziala */
    bool  drive_allowed;   /**< polityka energii pozwala jechac */
    bool  autonomy_ok;     /**< straznik pozwala na autonomie */
    bool  manual;          /**< niedawna reczna jazda z panelu */
    bool  water_busy;      /**< trwa dawka podlewania */
    bool  stalled;         /**< zatrzask utyku w flora_motors */
} flora_mission_in_t;

typedef struct {
    flora_mission_state_t state;
    flora_mission_fault_t fault;
    flora_seek_cmd_t      cmd;        /**< polecenie dla pilota, jednorazowe */
    float                 gx, gy;
    bool                  water_ok;   /**< robot stoi - wolno podlewac */
    bool                  may_sleep;  /**< nic nie trwa - wolno zasnac */
    flora_seek_out_t      seek;       /**< do telemetrii */
} flora_mission_out_t;

typedef struct {
    flora_mission_cfg_t   cfg;
    flora_seek_t          seek;
    bool                  was_enabled;
    flora_mission_fault_t fault;
    uint8_t               blocked_seeks;
    flora_mission_out_t   last;
} flora_mission_t;

void flora_mission_init(flora_mission_t *m, const flora_mission_cfg_t *cfg,
                        const flora_seek_cfg_t *seek_cfg);

/** Podmiana ustawien szukania bez zerowania stanu. */
void flora_mission_set_seek_cfg(flora_mission_t *m, const flora_seek_cfg_t *seek_cfg);

flora_mission_out_t flora_mission_update(flora_mission_t *m, const flora_mission_in_t *in);

/** Wymusza szukanie przy najblizszym kroku (flora_seek_force). */
void flora_mission_force_seek(flora_mission_t *m);

/** Zdejmuje FAULT i liczy postoj od nowa. */
void flora_mission_reset(flora_mission_t *m, uint32_t now_s);

#ifdef __cplusplus
}
#endif
