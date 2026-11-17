/*
 * Sprawdzenie flora_seek na komputerze, w symulowanym ogrodzie.
 *
 * Ogrod: prostokat 5 x 4 m. Funkcja `garden` mowi, czy w punkcie jest
 * slonce (60 klx, kontrast 0,7), czy cien (8 klx, kontrast 0,05).
 * Przejazd trwa DRIVE_S sekund i konczy sie dokladnie w celu, chyba ze
 * test kaze mu sie nie udac.
 *
 *   cc -O2 -Wall -I../include -o test test.c ../flora_seek.c -lm
 *   ./test            # kod wyjscia 0 = OK
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "flora_seek.h"

#define DRIVE_S 15

static int s_fail;

static void check(int ok, const char *what)
{
    printf("%s  %s\n", ok ? "OK  " : "FAIL", what);
    if (!ok) {
        s_fail++;
    }
}

typedef enum { G_SHADE_LEFT, G_ALL_SHADE, G_ALL_SUN } garden_t;

typedef struct {
    flora_seek_t s;
    garden_t garden;
    uint32_t now;
    float x, y;
    float elev;
    bool  elev_known;
    bool  hold;
    bool  nav_fails;           /* kazdy przejazd konczy sie porazka */
    flora_seek_nav_t nav;
    uint32_t arrive_at;
    float tx, ty;
    /* statystyki */
    int   gotos, stops;
    int   res[5];              /* licznik wynikow wg flora_seek_result_t */
    uint32_t last_res_s;       /* chwila ostatniego wyniku */
    uint32_t first_goto_s;
    bool  out_of_box;
    flora_seek_out_t o;
} sim_t;

static bool sunny(const sim_t *m, float x)
{
    switch (m->garden) {
    case G_SHADE_LEFT: return x >= 2000.0f;
    case G_ALL_SUN:    return true;
    default:           return false;
    }
}

static void sim_init(sim_t *m, const flora_seek_cfg_t *cfg, garden_t g, float x, float y)
{
    memset(m, 0, sizeof *m);
    flora_seek_init(&m->s, cfg);
    m->garden = g;
    m->x = x;
    m->y = y;
    m->elev = 40.0f;
    m->elev_known = true;
    m->nav = FLORA_SEEK_NAV_IDLE;
}

static void sim_tick(sim_t *m)
{
    if (m->nav == FLORA_SEEK_NAV_BUSY && m->now >= m->arrive_at) {
        if (m->nav_fails) {
            m->nav = FLORA_SEEK_NAV_FAILED;
        } else {
            m->x = m->tx;
            m->y = m->ty;
            m->nav = FLORA_SEEK_NAV_ARRIVED;
        }
    }
    const bool sun = sunny(m, m->x);
    flora_seek_in_t in = {
        .now_s = m->now,
        .lux_valid = true,
        .lux = sun ? 60000.0f : 8000.0f,
        .contrast = sun ? 0.7f : 0.05f,
        .elev_known = m->elev_known,
        .sun_elev_deg = m->elev,
        .sun_dir_valid = true,
        .sun_map_deg = 180.0f,       /* slonce od strony -x, cien pada w +x */
        .x_mm = m->x, .y_mm = m->y,
        .nav = m->nav,
        .hold = m->hold,
    };
    m->o = flora_seek_update(&m->s, &in);
    if (m->o.cmd == FLORA_SEEK_CMD_GOTO) {
        if (m->gotos++ == 0) {
            m->first_goto_s = m->now;
        }
        const flora_seek_cfg_t *c = &m->s.cfg;
        if (m->o.gx < c->xmin + c->margin_mm - 1.0f || m->o.gx > c->xmax - c->margin_mm + 1.0f ||
            m->o.gy < c->ymin + c->margin_mm - 1.0f || m->o.gy > c->ymax - c->margin_mm + 1.0f) {
            m->out_of_box = true;
        }
        m->tx = m->o.gx;
        m->ty = m->o.gy;
        m->nav = FLORA_SEEK_NAV_BUSY;
        m->arrive_at = m->now + DRIVE_S;
    } else if (m->o.cmd == FLORA_SEEK_CMD_STOP) {
        m->stops++;
        m->nav = FLORA_SEEK_NAV_FAILED;
    }
    if (m->o.result != FLORA_SEEK_RES_NONE) {
        m->res[m->o.result]++;
        m->last_res_s = m->now;
    }
    m->now++;
}

static void sim_run(sim_t *m, uint32_t seconds)
{
    for (uint32_t i = 0; i < seconds; i++) {
        sim_tick(m);
    }
}

/** Symulacja do chwili `t` (zegar symulacji). */
static void sim_until(sim_t *m, uint32_t t)
{
    while (m->now < t) {
        sim_tick(m);
    }
}

int main(void)
{
    flora_seek_cfg_t cfg = FLORA_SEEK_DEFAULT_CFG();
    cfg.area_valid = true;
    cfg.xmin = 0.0f;  cfg.xmax = 5000.0f;
    cfg.ymin = 0.0f;  cfg.ymax = 4000.0f;
    sim_t m;

    /* --- kontrast krzyza --- */
    check(flora_seek_contrast((float[]){ 100, 100, 100, 100 }) == 0.0f, "kontrast: rowne = 0");
    check(fabsf(flora_seek_contrast((float[]){ 900, 300, 100, 500 }) - 0.8f) < 1e-6f,
          "kontrast: (900-100)/(900+100) = 0,8");
    check(flora_seek_contrast((float[]){ 0, 0, 0, 0 }) == 0.0f, "kontrast: ciemno = 0, bez dzielenia przez zero");

    /* --- w sloncu: stoi --- */
    sim_init(&m, &cfg, G_ALL_SUN, 2500, 2000);
    sim_run(&m, 6 * 3600);
    check(m.gotos == 0 && !m.o.shade, "w sloncu: bez szukania");

    /* --- cien po lewej, robot w cieniu: znajduje slonce --- */
    sim_init(&m, &cfg, G_SHADE_LEFT, 1000, 2000);
    sim_run(&m, 1700);
    check(m.gotos == 0 && m.o.shade, "cien: przed min_dwell_s stoi");
    check(m.o.wait_s > 0 && m.o.wait_s <= 110, "cien: wait_s odlicza do konca postoju");
    sim_run(&m, 1200);
    check(m.first_goto_s >= cfg.min_dwell_s && m.first_goto_s < cfg.min_dwell_s + 5,
          "cien: rusza po min_dwell_s");
    check(m.res[FLORA_SEEK_RES_IMPROVED] == 1 && m.x >= 2000.0f, "cien: konczy w sloncu (IMPROVED)");
    check(m.gotos <= 3, "cien: zostaje w pierwszym jasnym punkcie, bez objazdu reszty");
    check(!m.out_of_box, "cien: cele w obszarze z marginesem");
    int g = m.gotos;
    sim_run(&m, 6 * 3600);
    check(m.gotos == g, "po znalezieniu slonca: stoi");

    /* --- pochmurno: szukanie bez zysku, powrot, rosnaca przerwa --- */
    sim_init(&m, &cfg, G_ALL_SHADE, 2500, 2000);
    sim_run(&m, 1800 + 600);
    check(m.res[FLORA_SEEK_RES_NO_GAIN] == 1, "pochmurno: NO_GAIN");
    check(m.gotos == cfg.max_probes + 1, "pochmurno: wszystkie punkty i powrot");
    check(fabsf(m.x - 2500.0f) < 1.0f && fabsf(m.y - 2000.0f) < 1.0f, "pochmurno: wraca na start");
    uint32_t t_end = m.last_res_s;
    g = m.gotos;
    sim_until(&m, t_end + cfg.backoff_s - 10);
    check(m.gotos == g, "pochmurno: w czasie przerwy stoi");
    check(m.o.wait_s > 0 && m.o.wait_s <= 15, "pochmurno: wait_s pokazuje reszte przerwy");
    sim_until(&m, t_end + cfg.backoff_s + 600);
    check(m.res[FLORA_SEEK_RES_NO_GAIN] == 2, "pochmurno: druga proba po godzinie");
    t_end = m.last_res_s;
    g = m.gotos;
    sim_until(&m, t_end + 2 * cfg.backoff_s - 10);
    check(m.gotos == g, "pochmurno: druga przerwa dluzsza (2 h)");
    sim_until(&m, t_end + 2 * cfg.backoff_s + 600);
    check(m.res[FLORA_SEEK_RES_NO_GAIN] == 3, "pochmurno: trzecia proba po 2 h");

    /* --- po udanym szukaniu przerwa wraca do zera --- */
    m.garden = G_SHADE_LEFT;
    m.x = 1000.0f;
    sim_run(&m, 5 * 3600);
    check(m.res[FLORA_SEEK_RES_IMPROVED] == 1 && m.s.backoff_cur_s == 0,
          "po IMPROVED: przerwa wyzerowana");

    /* --- noc --- */
    sim_init(&m, &cfg, G_ALL_SHADE, 2500, 2000);
    m.elev = -5.0f;
    sim_run(&m, 6 * 3600);
    check(m.gotos == 0 && !m.o.day, "noc: bez szukania");

    /* --- bez zegara: dzien wg jasnosci --- */
    sim_init(&m, &cfg, G_ALL_SHADE, 2500, 2000);
    m.elev_known = false;
    sim_run(&m, 2400);
    check(m.o.day && m.gotos > 0, "bez zegara: 8 klx to dzien, szuka");

    /* --- bez obszaru --- */
    flora_seek_cfg_t noarea = cfg;
    noarea.area_valid = false;
    sim_init(&m, &noarea, G_SHADE_LEFT, 1000, 2000);
    sim_run(&m, 6 * 3600);
    check(m.gotos == 0, "bez obszaru roboczego: nie rusza");

    /* --- hold przed startem --- */
    sim_init(&m, &cfg, G_SHADE_LEFT, 1000, 2000);
    m.hold = true;
    sim_run(&m, 2 * 3600);
    check(m.gotos == 0, "hold: nie zaczyna");
    m.hold = false;
    sim_run(&m, 5);
    check(m.gotos == 1, "hold zdjety: rusza od razu");

    /* --- hold w trakcie jazdy --- */
    sim_run(&m, 5);
    m.hold = true;
    sim_tick(&m);
    check(m.stops == 1 && m.res[FLORA_SEEK_RES_ABORTED] == 1 && m.o.state == FLORA_SEEK_WATCH,
          "hold w trakcie jazdy: STOP i ABORTED");
    m.hold = false;
    g = m.gotos;
    sim_run(&m, 1700);
    check(m.gotos == g, "po przerwaniu: postoj liczony od nowa");

    /* --- kazdy przejazd zablokowany --- */
    sim_init(&m, &cfg, G_SHADE_LEFT, 1000, 2000);
    m.nav_fails = true;
    sim_run(&m, 1800 + 300);
    check(m.res[FLORA_SEEK_RES_BLOCKED] == 1 && m.gotos == cfg.max_probes,
          "wszystkie przejazdy zablokowane: BLOCKED po wszystkich probach");

    /* --- robot w rogu: cele przyciete, za krotkie pominiete --- */
    sim_init(&m, &cfg, G_ALL_SHADE, 4500, 500);
    sim_run(&m, 1800 + 600);
    check(m.gotos > 0 && !m.out_of_box, "w rogu: cele przyciete do obszaru");
    check(m.gotos < cfg.max_probes + 1, "w rogu: za krotkie odcinki pominiete");

    /* --- robot poza obszarem: luk wokol najblizszego punktu w srodku --- */
    sim_init(&m, &cfg, G_ALL_SHADE, -800, 2000);
    sim_run(&m, 1800 + 600);
    check(m.gotos > 0 && !m.out_of_box, "start poza obszarem: cele w obszarze");

    /* --- obszar mniejszy niz marginesy --- */
    flora_seek_cfg_t tiny = cfg;
    tiny.xmax = 700.0f;
    sim_init(&m, &tiny, G_ALL_SHADE, 300, 2000);
    sim_run(&m, 3 * 3600);
    check(m.gotos == 0, "obszar wezszy niz 2 marginesy: nie rusza");

    /* --- wymuszenie: w sloncu, zaraz po starcie --- */
    sim_init(&m, &cfg, G_ALL_SUN, 2500, 2000);
    sim_run(&m, 5);
    flora_seek_force(&m.s);
    sim_run(&m, 2);
    check(m.gotos == 1, "force: rusza bez cienia i bez postoju");
    sim_run(&m, 600);
    g = m.gotos;
    sim_run(&m, 3600);
    check(m.gotos == g, "force: jednorazowe");
    sim_init(&m, &noarea, G_ALL_SUN, 2500, 2000);
    sim_run(&m, 5);
    flora_seek_force(&m.s);
    sim_run(&m, 60);
    check(m.gotos == 0, "force bez obszaru: nie rusza");

    printf("WYNIK: %s\n", s_fail ? "BLEDY" : "OK");
    return s_fail ? 1 : 0;
}
