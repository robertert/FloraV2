/*
 * Sprawdzenie flora_pilot na komputerze, na idealnym robocie rozniczkowym
 * (kola jada dokladnie z zadana predkoscia, bez poslizgu).
 *
 *   cc -O2 -Wall -I../include -o test test.c ../flora_pilot.c -lm
 *   ./test            # kod wyjscia 0 = OK
 *   ./test --csv      # tor t,x,y,th do wykresu (pierwszy przejazd)
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "flora_pilot.h"

#define DT      0.05f
#define TRACK 390.0f
#define RAD2DEG 57.29578f

static int s_fail;

static void check(int ok, const char *what)
{
    printf("%s  %s\n", ok ? "OK  " : "FAIL", what);
    if (!ok) {
        s_fail++;
    }
}

typedef struct { float x, y, th; } sim_t;    /* th w radianach */

static void sim_step(sim_t *s, float vl, float vr)
{
    const float v = 0.5f * (vl + vr);
    const float w = (vr - vl) / TRACK;
    s->x += v * cosf(s->th) * DT;
    s->y += v * sinf(s->th) * DT;
    s->th += w * DT;
}

/** Symulacja do konca zadania; `blocked` = naped stoi mimo zadanej (weto). */
static flora_pilot_out_t run(flora_pilot_t *p, sim_t *s, float max_s, int blocked, int csv,
                             float *max_speed)
{
    flora_pilot_out_t o = { 0 };
    float vmax = 0.0f;
    for (float t = 0.0f; t < max_s; t += DT) {
        o = flora_pilot_update(p, s->x, s->y, s->th * RAD2DEG, DT);
        if (!flora_pilot_busy(p)) {
            break;
        }
        if (fabsf(o.v_l) > vmax) vmax = fabsf(o.v_l);
        if (fabsf(o.v_r) > vmax) vmax = fabsf(o.v_r);
        if (!blocked) {
            sim_step(s, o.v_l, o.v_r);
        }
        if (csv) {
            printf("%.2f,%.1f,%.1f,%.1f\n", t, s->x, s->y, s->th * RAD2DEG);
        }
    }
    if (max_speed) {
        *max_speed = vmax;
    }
    return o;
}

static float dist_to(const sim_t *s, float x, float y)
{
    return hypotf(s->x - x, s->y - y);
}

int main(int argc, char **argv)
{
    const int csv = argc > 1 && strcmp(argv[1], "--csv") == 0;
    flora_pilot_cfg_t cfg = FLORA_PILOT_DEFAULT_CFG();
    cfg.track_mm = TRACK;
    flora_pilot_t p;
    flora_pilot_out_t o;
    sim_t s;
    float vmax;

    /* --- cel z boku i z tylu: najpierw obrot, potem jazda --- */
    flora_pilot_init(&p, &cfg);
    s = (sim_t){ 0, 0, 0 };
    flora_pilot_goto(&p, -1000.0f, 1500.0f);
    o = flora_pilot_update(&p, s.x, s.y, 0.0f, DT);
    if (csv) {
        run(&p, &s, 120.0f, 0, 1, NULL);
        return 0;
    }
    check(o.state == FLORA_PILOT_TURN && o.v_l < 0.0f && o.v_r > 0.0f && o.v_l == -o.v_r,
          "cel z tylu po lewej: obrot w lewo w miejscu");
    o = run(&p, &s, 120.0f, 0, 0, &vmax);
    check(o.state == FLORA_PILOT_DONE, "goto: dojechal");
    check(dist_to(&s, -1000.0f, 1500.0f) <= cfg.pos_tol_mm, "goto: w tolerancji pozycji");
    check(vmax <= cfg.v_max + cfg.w_max_dps / RAD2DEG * 0.5f * TRACK + 1.0f,
          "goto: predkosc kol w limicie");

    /* --- po zakonczeniu zero na wyjsciu --- */
    o = flora_pilot_update(&p, s.x, s.y, s.th * RAD2DEG, DT);
    check(o.v_l == 0.0f && o.v_r == 0.0f && o.state == FLORA_PILOT_DONE,
          "po dojezdzie: zero na kolach");

    /* --- cel na wprost: bez obrotu w miejscu --- */
    flora_pilot_init(&p, &cfg);
    s = (sim_t){ 0, 0, 0 };
    flora_pilot_goto(&p, 2000.0f, 50.0f);
    o = flora_pilot_update(&p, s.x, s.y, 0.0f, DT);
    check(o.state == FLORA_PILOT_DRIVE && o.v_l > 0.0f && o.v_r > 0.0f,
          "cel na wprost: od razu jedzie");
    o = run(&p, &s, 60.0f, 0, 0, NULL);
    check(o.state == FLORA_PILOT_DONE && dist_to(&s, 2000.0f, 50.0f) <= cfg.pos_tol_mm,
          "cel na wprost: dojechal");

    /* --- kurs koncowy --- */
    flora_pilot_init(&p, &cfg);
    s = (sim_t){ 0, 0, 0 };
    flora_pilot_goto_pose(&p, 800.0f, 0.0f, -90.0f);
    o = run(&p, &s, 120.0f, 0, 0, NULL);
    check(o.state == FLORA_PILOT_DONE, "goto z kursem: zakonczone");
    check(fabsf(s.th * RAD2DEG + 90.0f) <= cfg.th_tol_deg, "goto z kursem: kurs -90 st");

    /* --- sam obrot, krotsza strona przez +-180 --- */
    flora_pilot_init(&p, &cfg);
    s = (sim_t){ 0, 0, 170.0f / RAD2DEG };
    flora_pilot_turn(&p, -170.0f);
    o = flora_pilot_update(&p, s.x, s.y, s.th * RAD2DEG, DT);
    check(o.v_r > 0.0f && o.v_l < 0.0f, "obrot 170 -> -170: w lewo (20 st), nie w prawo (340)");
    o = run(&p, &s, 60.0f, 0, 0, NULL);
    check(o.state == FLORA_PILOT_DONE && dist_to(&s, 0.0f, 0.0f) < 1.0f,
          "sam obrot: gotowe, robot nie odjechal");

    /* --- juz na miejscu --- */
    flora_pilot_init(&p, &cfg);
    flora_pilot_goto(&p, 20.0f, -10.0f);
    o = flora_pilot_update(&p, 0.0f, 0.0f, 0.0f, DT);
    check(o.state == FLORA_PILOT_DONE && o.v_l == 0.0f, "cel w tolerancji: od razu gotowe");

    /* --- naped zablokowany: brak postepu --- */
    flora_pilot_init(&p, &cfg);
    s = (sim_t){ 0, 0, 0 };
    flora_pilot_goto(&p, 2000.0f, 0.0f);
    o = run(&p, &s, 60.0f, 1, 0, NULL);
    check(o.state == FLORA_PILOT_FAILED && o.fail == FLORA_PILOT_FAIL_NO_PROGRESS,
          "naped stoi: FAILED / NO_PROGRESS");
    check(p.t > cfg.no_progress_s && p.t < cfg.no_progress_s + 1.0f,
          "naped stoi: po no_progress_s");

    /* --- limit czasu --- */
    cfg.timeout_s = 5.0f;
    flora_pilot_init(&p, &cfg);
    s = (sim_t){ 0, 0, 0 };
    flora_pilot_goto(&p, 5000.0f, 0.0f);
    o = run(&p, &s, 60.0f, 0, 0, NULL);
    check(o.state == FLORA_PILOT_FAILED && o.fail == FLORA_PILOT_FAIL_TIMEOUT,
          "za daleko na limit czasu: FAILED / TIMEOUT");
    cfg.timeout_s = 180.0f;

    /* --- przerwanie --- */
    flora_pilot_init(&p, &cfg);
    flora_pilot_goto(&p, 2000.0f, 0.0f);
    flora_pilot_update(&p, 0.0f, 0.0f, 0.0f, DT);
    flora_pilot_cancel(&p);
    o = flora_pilot_update(&p, 0.0f, 0.0f, 0.0f, DT);
    check(o.state == FLORA_PILOT_FAILED && o.fail == FLORA_PILOT_FAIL_CANCELLED &&
          o.v_l == 0.0f && o.v_r == 0.0f, "cancel: FAILED / CANCELLED, zero na kolach");
    flora_pilot_init(&p, &cfg);
    flora_pilot_cancel(&p);
    check(p.state == FLORA_PILOT_IDLE, "cancel bez zadania: zostaje IDLE");

    /* --- zepchniecie z kursu w trakcie jazdy: wraca do obrotu --- */
    flora_pilot_init(&p, &cfg);
    s = (sim_t){ 0, 0, 0 };
    flora_pilot_goto(&p, 3000.0f, 0.0f);
    for (int i = 0; i < 40; i++) {
        o = flora_pilot_update(&p, s.x, s.y, s.th * RAD2DEG, DT);
        sim_step(&s, o.v_l, o.v_r);
    }
    s.th = 60.0f / RAD2DEG;
    o = flora_pilot_update(&p, s.x, s.y, s.th * RAD2DEG, DT);
    check(o.state == FLORA_PILOT_TURN && o.v_l > 0.0f && o.v_r < 0.0f,
          "obrocony o 60 st w trakcie: staje i obraca w prawo");
    o = run(&p, &s, 120.0f, 0, 0, NULL);
    check(o.state == FLORA_PILOT_DONE, "po zepchnieciu: dojechal");

    printf("WYNIK: %s\n", s_fail ? "BLEDY" : "OK");
    return s_fail ? 1 : 0;
}
