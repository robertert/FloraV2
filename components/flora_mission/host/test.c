/*
 * Sprawdzenie flora_mission na komputerze. Szukanie slonca ma wlasny test
 * (flora_seek/host/test.c) - tu tylko to, co dodaje misja: kiedy szukaniu
 * wolno dzialac, stany, zatrzaski i zgody.
 *
 * Ogrod jak w tescie flora_seek: cien dla x < 2000, slonce dalej.
 *
 *   cc -O2 -Wall -I../include -I../../flora_seek/include -o test test.c \
 *      ../flora_mission.c ../../flora_seek/flora_seek.c -lm
 *   ./test            # kod wyjscia 0 = OK
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "flora_mission.h"

#define DRIVE_S 15

static int s_fail;

static void check(int ok, const char *what)
{
    printf("%s  %s\n", ok ? "OK  " : "FAIL", what);
    if (!ok) {
        s_fail++;
    }
}

typedef struct {
    flora_mission_t m;
    uint32_t now;
    float x, y;
    bool  all_shade;
    bool  enabled, pose_ok, drive_allowed, autonomy_ok, manual, water_busy, stalled;
    bool  nav_fails;
    flora_seek_nav_t nav;
    uint32_t arrive_at;
    float tx, ty;
    int   gotos, stops;
    bool  moved_while_water;   /* jazda zlecona w czasie dawki */
    flora_mission_out_t o;
} sim_t;

static void sim_init(sim_t *s, float x, float y)
{
    flora_mission_cfg_t cfg = FLORA_MISSION_DEFAULT_CFG();
    flora_seek_cfg_t sc = FLORA_SEEK_DEFAULT_CFG();
    sc.area_valid = true;
    sc.xmin = 0.0f;  sc.xmax = 5000.0f;
    sc.ymin = 0.0f;  sc.ymax = 4000.0f;
    memset(s, 0, sizeof *s);
    flora_mission_init(&s->m, &cfg, &sc);
    s->x = x;
    s->y = y;
    s->enabled = s->pose_ok = s->drive_allowed = s->autonomy_ok = true;
    s->nav = FLORA_SEEK_NAV_IDLE;
}

static void sim_tick(sim_t *s)
{
    if (s->nav == FLORA_SEEK_NAV_BUSY && s->now >= s->arrive_at) {
        if (s->nav_fails) {
            s->nav = FLORA_SEEK_NAV_FAILED;
        } else {
            s->x = s->tx;
            s->y = s->ty;
            s->nav = FLORA_SEEK_NAV_ARRIVED;
        }
    }
    const bool sun = !s->all_shade && s->x >= 2000.0f;
    flora_mission_in_t in = {
        .enabled = s->enabled,
        .seek = {
            .now_s = s->now,
            .lux_valid = true,
            .lux = sun ? 60000.0f : 8000.0f,
            .contrast = sun ? 0.7f : 0.05f,
            .elev_known = true,
            .sun_elev_deg = 40.0f,
            .sun_dir_valid = true,
            .sun_map_deg = 180.0f,
            .x_mm = s->x, .y_mm = s->y,
            .nav = s->nav,
        },
        .pose_ok = s->pose_ok,
        .drive_allowed = s->drive_allowed,
        .autonomy_ok = s->autonomy_ok,
        .manual = s->manual,
        .water_busy = s->water_busy,
        .stalled = s->stalled,
    };
    s->o = flora_mission_update(&s->m, &in);
    if (s->o.cmd == FLORA_SEEK_CMD_GOTO) {
        s->gotos++;
        if (s->water_busy) {
            s->moved_while_water = true;
        }
        s->tx = s->o.gx;
        s->ty = s->o.gy;
        s->nav = FLORA_SEEK_NAV_BUSY;
        s->arrive_at = s->now + DRIVE_S;
    } else if (s->o.cmd == FLORA_SEEK_CMD_STOP) {
        s->stops++;
        s->nav = FLORA_SEEK_NAV_FAILED;
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
    sim_t s;

    /* --- wylaczona --- */
    sim_init(&s, 1000, 2000);
    s.enabled = false;
    sim_run(&s, 3 * 3600);
    check(s.o.state == FLORA_MISSION_IDLE && s.gotos == 0 && s.o.water_ok && s.o.may_sleep,
          "wylaczona: IDLE, nie rusza, podlewac i spac wolno");

    /* --- wlaczenie: postoj liczony od wlaczenia, potem szukanie i slonce --- */
    s.enabled = true;
    sim_run(&s, 1700);
    check(s.o.state == FLORA_MISSION_DWELL && s.gotos == 0,
          "po wlaczeniu: DWELL, postoj liczony od wlaczenia");
    sim_run(&s, 110);
    check(s.o.state == FLORA_MISSION_SEEK_MOVE && !s.o.water_ok && !s.o.may_sleep,
          "szukanie: SEEK_MOVE, bez podlewania i snu");
    sim_run(&s, DRIVE_S + 2);
    check(s.o.state == FLORA_MISSION_SEEK_MEASURE && !s.o.water_ok,
          "w punkcie probnym: SEEK_MEASURE");
    sim_run(&s, 600);
    check(s.o.state == FLORA_MISSION_DWELL && s.x >= 2000.0f && s.o.water_ok && s.o.may_sleep,
          "po szukaniu: DWELL w sloncu");

    /* --- niska bateria: stoi, podlewac wolno --- */
    sim_init(&s, 1000, 2000);
    s.drive_allowed = false;
    sim_run(&s, 3 * 3600);
    check(s.o.state == FLORA_MISSION_LOW_BATT && s.gotos == 0 && s.o.water_ok && s.o.may_sleep,
          "niska bateria: LOW_BATT, stoi");
    s.drive_allowed = true;
    sim_run(&s, 10);
    check(s.gotos == 1, "bateria wrocila: rusza");

    /* --- niska bateria w trakcie jazdy: stop --- */
    sim_run(&s, 3);
    s.drive_allowed = false;
    sim_tick(&s);
    check(s.stops == 1 && s.o.state == FLORA_MISSION_LOW_BATT,
          "bateria siada w trakcie jazdy: STOP i LOW_BATT");

    /* --- straznik / reczna jazda / brak pozy --- */
    sim_init(&s, 1000, 2000);
    s.autonomy_ok = false;
    sim_run(&s, 3 * 3600);
    check(s.o.state == FLORA_MISSION_HOLD && s.gotos == 0, "straznik wstrzymuje: HOLD");
    sim_init(&s, 1000, 2000);
    s.pose_ok = false;
    sim_run(&s, 3 * 3600);
    check(s.o.state == FLORA_MISSION_HOLD && s.gotos == 0, "brak pozy: HOLD");
    sim_init(&s, 1000, 2000);
    sim_run(&s, 1805);
    check(s.gotos == 1, "(przygotowanie) jedzie");
    s.manual = true;
    s.nav = FLORA_SEEK_NAV_FAILED;          /* reczna jazda anulowala pilota */
    sim_tick(&s);
    check(s.stops == 0 && s.o.state == FLORA_MISSION_HOLD && !s.o.may_sleep,
          "reczna jazda w trakcie: bez STOP (pilot juz stoi), HOLD, bez snu");
    s.manual = false;
    int g = s.gotos;
    sim_run(&s, 1700);
    check(s.gotos == g, "po recznej jezdzie: postoj od nowa");

    /* --- podlewanie wstrzymuje start szukania --- */
    sim_init(&s, 1000, 2000);
    s.water_busy = true;
    sim_run(&s, 2 * 3600);
    check(s.o.state == FLORA_MISSION_WATER && s.gotos == 0 && !s.o.may_sleep,
          "dawka w toku: WATER, szukanie czeka, bez snu");
    s.water_busy = false;
    sim_run(&s, 5);
    check(s.gotos == 1 && !s.moved_while_water, "po dawce: rusza");

    /* --- utyk: zatrzask --- */
    sim_init(&s, 1000, 2000);
    sim_run(&s, 1805);
    s.stalled = true;
    sim_tick(&s);
    check(s.o.state == FLORA_MISSION_FAULT && s.o.fault == FLORA_MISSION_FAULT_STALL &&
          s.stops == 1, "utyk w trakcie jazdy: FAULT / STALL, STOP");
    s.stalled = false;
    g = s.gotos;
    sim_run(&s, 6 * 3600);
    check(s.o.state == FLORA_MISSION_FAULT && s.gotos == g && s.o.water_ok,
          "FAULT trzyma po zdjeciu utyku, podlewac wolno");
    flora_mission_reset(&s.m, s.now);
    sim_run(&s, 1700);
    check(s.o.state == FLORA_MISSION_DWELL && s.gotos == g, "reset: DWELL, postoj od nowa");
    sim_run(&s, 200);
    check(s.gotos > g, "reset: po postoju szuka dalej");

    /* --- dwa szukania bez udanego przejazdu: zatrzask --- */
    sim_init(&s, 1000, 2000);
    s.nav_fails = true;
    sim_run(&s, 1800 + 200);
    check(s.o.state == FLORA_MISSION_DWELL && s.m.blocked_seeks == 1,
          "pierwsze zablokowane szukanie: jeszcze nie FAULT");
    sim_run(&s, 2 * 3600);
    check(s.o.state == FLORA_MISSION_FAULT && s.o.fault == FLORA_MISSION_FAULT_BLOCKED,
          "drugie zablokowane szukanie: FAULT / BLOCKED");

    /* --- udane szukanie zeruje licznik zablokowanych --- */
    sim_init(&s, 1000, 2000);
    s.all_shade = true;
    s.nav_fails = true;
    sim_run(&s, 1800 + 200);
    s.nav_fails = false;
    sim_run(&s, 2 * 3600);
    check(s.o.fault == FLORA_MISSION_FAULT_NONE && s.m.blocked_seeks == 0,
          "szukanie z udanymi przejazdami zeruje licznik");

    /* --- wylaczenie w trakcie jazdy --- */
    sim_init(&s, 1000, 2000);
    sim_run(&s, 1805);
    s.enabled = false;
    sim_tick(&s);
    check(s.stops == 1 && s.o.state == FLORA_MISSION_IDLE, "wylaczenie w trakcie jazdy: STOP, IDLE");
    sim_run(&s, 100);
    check(s.stops == 1, "wylaczona: STOP tylko raz");

    printf("WYNIK: %s\n", s_fail ? "BLEDY" : "OK");
    return s_fail ? 1 : 0;
}
