/*
 * Sprawdzenie flora_water na komputerze, z symulowana doniczka i pompka.
 *
 * Model: pompka podaje PUMP_ML_S, jedno uruchomienie najwyzej CHUNK_ML.
 * Gleba: kazdy ml podnosi wilgotnosc o GAIN (z opoznieniem LAG_S), a czas
 * ja obniza o DRY_RATE na sekunde.
 *
 *   cc -O2 -Wall -I../include -o test test.c ../flora_water.c -lm
 *   ./test            # kod wyjscia 0 = OK
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "flora_water.h"

#define PUMP_ML_S  1.5f
#define CHUNK_ML  15.0f
#define GAIN       0.12f      /* pkt proc. na ml */
#define LAG_S    120
#define DRY_RATE   0.0005f    /* pkt proc. na sekunde */

static int s_fail;

static void check(int ok, const char *what)
{
    printf("%s  %s\n", ok ? "OK  " : "FAIL", what);
    if (!ok) {
        s_fail++;
    }
}

typedef struct {
    flora_water_t w;
    uint32_t now;
    float soil;            /* prawdziwa wilgotnosc przy czujniku */
    float pending_ml;      /* woda w drodze do czujnika */
    uint32_t pending_at;
    bool  water;           /* zbiornik */
    bool  allowed;
    bool  sensor;          /* czujnik odpowiada */
    bool  plumbed;         /* woda trafia do doniczki */
    bool  pump_dead;       /* pompka odmawia startu */
    float pump_left_ml;    /* biezaca porcja */
    float pump_run_ml;
    float total_ml;
    int   starts;
    flora_water_out_t o;
} sim_t;

static void sim_init(sim_t *s, const flora_water_cfg_t *cfg, float soil)
{
    memset(s, 0, sizeof *s);
    flora_water_init(&s->w, cfg);
    s->soil = soil;
    s->water = s->allowed = s->sensor = s->plumbed = true;
}

/** Jedna sekunda symulacji. */
static void sim_tick(sim_t *s)
{
    float delivered = 0.0f;
    if (s->pump_left_ml > 0.0f) {
        float d = PUMP_ML_S;
        if (d > s->pump_left_ml) d = s->pump_left_ml;
        s->pump_left_ml -= d;
        s->pump_run_ml += d;
        if (!s->water) {
            s->pump_left_ml = 0.0f;       /* flora_pump ucina na sucho */
        }
        if (s->pump_left_ml <= 0.0f) {
            delivered = s->pump_run_ml;
            s->total_ml += delivered;
            if (s->plumbed) {
                s->pending_ml += delivered;
                s->pending_at = s->now + LAG_S;
            }
            s->pump_run_ml = 0.0f;
        }
    }
    if (s->pending_ml > 0.0f && s->now >= s->pending_at) {
        s->soil += s->pending_ml * GAIN;
        s->pending_ml = 0.0f;
    }
    s->soil -= DRY_RATE;
    if (s->soil < 0.0f) s->soil = 0.0f;

    flora_water_in_t in = {
        .now_s = s->now,
        .soil_valid = s->sensor,
        .soil_pct = s->soil,
        .water_present = s->water,
        .allowed = s->allowed,
        .pump_busy = s->pump_left_ml > 0.0f,
        .delivered_ml = delivered,
        .max_chunk_ml = CHUNK_ML,
    };
    s->o = flora_water_update(&s->w, &in);
    if (s->o.request_ml > 0.0f && s->pump_left_ml <= 0.0f && !s->pump_dead && s->water) {
        s->pump_left_ml = s->o.request_ml;
        s->starts++;
    }
    s->now++;
}

static void sim_run(sim_t *s, uint32_t seconds)
{
    for (uint32_t i = 0; i < seconds; i++) {
        sim_tick(s);
    }
}

int main(void)
{
    flora_water_cfg_t cfg = FLORA_WATER_DEFAULT_CFG();
    sim_t s;

    /* --- wilgotno: nic nie robi --- */
    sim_init(&s, &cfg, 60.0f);
    sim_run(&s, 3);
    check(s.o.state == FLORA_WATER_NO_SENSOR && isnan(s.o.median_pct),
          "przed pelnym oknem mediany: NO_SENSOR");
    sim_run(&s, 600);
    check(s.o.state == FLORA_WATER_OK && s.starts == 0, "wilgotno: bez podlewania");

    /* --- sucho: dawka w kawalkach, wsiakanie, dojscie do progu mokrego --- */
    sim_init(&s, &cfg, 30.0f);
    sim_run(&s, 20);
    check(s.o.state == FLORA_WATER_DOSING, "sucho: zaczyna dawke");
    sim_run(&s, 60);
    check(fabsf(s.total_ml - 50.0f) < 0.6f && s.starts == 4,
          "dawka 50 ml w 4 kawalkach po max 15 ml");
    check(s.o.state == FLORA_WATER_SOAK, "po dawce: wsiakanie");
    sim_run(&s, 300);
    check(s.o.state == FLORA_WATER_SOAK && fabsf(s.total_ml - 50.0f) < 0.6f,
          "w czasie wsiakania pompka stoi");
    sim_run(&s, 3600);
    check(s.o.state == FLORA_WATER_OK && s.soil > cfg.wet_pct, "po kilku dawkach: mokro, koniec");
    check(s.total_ml >= 200.0f && s.total_ml <= 260.0f, "razem 200-260 ml (30 -> ponad 55 proc.)");
    /* Powoli wysycha w pasmie histerezy: bez kolejnych dawek. */
    sim_run(&s, 3 * 3600);
    check(s.o.state == FLORA_WATER_OK && s.total_ml <= 260.0f && s.soil < cfg.wet_pct,
          "wysychanie w pasmie histerezy: bez dawek");

    /* --- histereza: w pasmie miedzy progami bez dawek --- */
    sim_init(&s, &cfg, 45.0f);
    sim_run(&s, 3600);
    check(s.starts == 0 && s.o.state == FLORA_WATER_OK, "45 proc. od gory: nie podlewa");

    /* --- limit dobowy i jego odnowienie --- */
    cfg.daily_max_ml = 120.0f;
    sim_init(&s, &cfg, 20.0f);
    sim_run(&s, 6 * 3600);
    check(s.o.state == FLORA_WATER_LIMIT && fabsf(s.total_ml - 120.0f) < 1.0f,
          "limit dobowy 120 ml: stop na limicie");
    sim_run(&s, 19 * 3600);
    check(s.total_ml > 121.0f, "po 24 h okno sie odnawia i podlewa dalej");
    cfg.daily_max_ml = 500.0f;

    /* --- pusty zbiornik i dolanie --- */
    sim_init(&s, &cfg, 30.0f);
    s.water = false;
    sim_run(&s, 600);
    check(s.o.state == FLORA_WATER_NO_WATER && s.starts == 0, "pusty zbiornik: NO_WATER, pompka stoi");
    s.water = true;
    sim_run(&s, 30);
    check(s.o.state == FLORA_WATER_DOSING && s.starts > 0, "dolana woda: wznawia");

    /* --- woda konczy sie w trakcie dawki --- */
    sim_init(&s, &cfg, 30.0f);
    sim_run(&s, 15);
    s.water = false;
    sim_run(&s, 30);
    check(s.o.state == FLORA_WATER_NO_WATER, "woda skonczyla sie w trakcie: NO_WATER");

    /* --- brak zgody (robot jedzie) --- */
    sim_init(&s, &cfg, 30.0f);
    s.allowed = false;
    sim_run(&s, 900);
    check(s.o.state == FLORA_WATER_DRY && s.starts == 0, "brak zgody: DRY, czeka");
    s.allowed = true;
    sim_run(&s, 5);
    check(s.o.state == FLORA_WATER_DOSING, "zgoda: zaczyna");
    /* zgoda znika w polowie dawki na dluzej niz limit pompki */
    sim_run(&s, 12);
    s.allowed = false;
    sim_run(&s, 600);
    check(s.o.state == FLORA_WATER_DOSING && s.o.fault == FLORA_WATER_FAULT_NONE,
          "zgoda znika w polowie dawki: czeka bez awarii");
    s.allowed = true;
    sim_run(&s, 120);
    check(s.o.state == FLORA_WATER_SOAK, "zgoda wraca: konczy dawke");

    /* --- czujnik wyjety / rurka spadla: brak reakcji --- */
    sim_init(&s, &cfg, 30.0f);
    s.plumbed = false;
    sim_run(&s, 3 * 3600);
    check(s.o.state == FLORA_WATER_FAULT && s.o.fault == FLORA_WATER_FAULT_NO_RISE,
          "3 dawki bez wzrostu wilgotnosci: FAULT / NO_RISE");
    check(fabsf(s.total_ml - 150.0f) < 1.5f, "do zatrzasku poszlo 3 x 50 ml");
    float before = s.total_ml;
    sim_run(&s, 3600);
    check(s.total_ml == before, "po zatrzasku pompka stoi");
    flora_water_reset_fault(&s.w);
    s.plumbed = true;
    sim_run(&s, 30);
    check(s.o.state == FLORA_WATER_DOSING, "reset_fault: wznawia");

    /* --- pompka nie startuje --- */
    sim_init(&s, &cfg, 30.0f);
    s.pump_dead = true;
    sim_run(&s, 400);
    check(s.o.state == FLORA_WATER_FAULT && s.o.fault == FLORA_WATER_FAULT_PUMP,
          "pompka nie startuje: FAULT / PUMP");

    /* --- czujnik milknie --- */
    sim_init(&s, &cfg, 60.0f);
    sim_run(&s, 30);
    s.sensor = false;
    s.soil = 10.0f;
    sim_run(&s, 200);
    check(s.o.state == FLORA_WATER_NO_SENSOR && s.starts == 0, "czujnik milknie: NO_SENSOR, bez dawek");

    /* --- pojedynczy skok odczytu nie uruchamia dawki --- */
    sim_init(&s, &cfg, 60.0f);
    sim_run(&s, 30);
    s.soil = 5.0f;
    sim_tick(&s);
    sim_tick(&s);
    s.soil = 60.0f;
    sim_run(&s, 30);
    check(s.starts == 0, "dwa falszywe odczyty: mediana je wycina");

    printf("WYNIK: %s\n", s_fail ? "BLEDY" : "OK");
    return s_fail ? 1 : 0;
}
