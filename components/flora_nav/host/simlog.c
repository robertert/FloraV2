/*
 * Syntetyczne nagranie w formacie hub_rec (flora_navlog.h): kwadrat 5 x 5 m
 * z postojami w rogach, poslizg kol, kolo prawe o 1% wieksze niz zaklada
 * kalibracja, kurs z glowicy z szumem, paliki w rogach. Do sprawdzenia
 * replay.c bez robota i jako przyklad formatu.
 *
 *   cc -O2 -Wall -I../include -o simlog simlog.c -lm
 *   ./simlog > sim.log            # robot konczy w (0, 0)
 *   ./simlog --drop 40 > sim.log  # bez paczki nr 40 (test luk)
 */
#include <math.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "flora_navlog.h"

#define DT        0.02
#define SIDE      5000.0
#define SPEED     100.0
#define TURN_DPS  30.0
#define W_TRUE    400.0     /* filtr zaklada 390 (linia P) */
#define COUNTS    11880.0
#define D_CFG     124.0     /* srednica w kalibracji */
#define D_L_TRUE  124.0
#define D_R_TRUE  125.24    /* +1% */
#define B_TRUE_DEG 57.3     /* kurs kompasowy osi X mapy */

static uint64_t s_rng = 4242;
static double rnd_n(double sigma)
{
    s_rng = s_rng * 6364136223846793005ULL + 1442695040888963407ULL;
    double u1 = ((s_rng >> 11) + 0.5) / 9007199254740992.0;
    s_rng = s_rng * 6364136223846793005ULL + 1442695040888963407ULL;
    double u2 = ((s_rng >> 11) + 0.5) / 9007199254740992.0;
    return sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2) * sigma;
}

static const double TAGS[4][2] = { { -500, -500 }, { 5500, -500 }, { 5500, 5500 }, { -500, 5500 } };

/* stan prawdy i licznikow */
static double x, y, th;                 /* th CCW, rad */
static double cl, cr;                   /* zliczenia (ulamkowe - enkoder liczy calkowite) */
static long long t_us = 1000000;
static int tick, seq, drop = -1;
static char msg[8192];
static size_t msg_n;

static void out(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void out(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    msg_n += vsnprintf(msg + msg_n, sizeof msg - msg_n, fmt, ap);
    va_end(ap);
}

static void flush(void)
{
    if (msg_n && seq != drop) {
        printf("#%d\n%s", seq, msg);
    }
    seq++;
    msg_n = 0;
    msg[0] = 0;
}

/** Jeden takt: komenda drog kol [mm], prawda z poslizgiem, linie A/G/E. */
static void step(double ul, double ur, int tags_now)
{
    /* Kola obracaja sie o tyle, ile zadano wg kalibracji (PID na enkoderach);
     * po ziemi przejezdzaja proporcjonalnie do prawdziwej srednicy + poslizg. */
    double nl = ul / (M_PI * D_CFG / COUNTS), nr = ur / (M_PI * D_CFG / COUNTS);
    double gl = nl * M_PI * D_L_TRUE / COUNTS + rnd_n(sqrt(0.3 * fabs(ul)));
    double gr = nr * M_PI * D_R_TRUE / COUNTS + rnd_n(sqrt(0.3 * fabs(ur)));
    double ds = 0.5 * (gl + gr), dth = (gr - gl) / W_TRUE;
    x += ds * cos(th + 0.5 * dth);
    y += ds * sin(th + 0.5 * dth);
    th += dth;
    cl += nl;
    cr += nr;
    t_us += (long long)(DT * 1e6);
    tick++;

    if (tick % 3 != 0) {   /* ATTITUDE ~17 Hz */
        double head = fmod(B_TRUE_DEG - th * 180.0 / M_PI + rnd_n(3.0) + 3600.0, 360.0);
        double rate = -(dth / DT) * 180.0 / M_PI;   /* CW */
        out(FLORA_NAVLOG_FMT_A, 20000L, (unsigned)lround(head * 100) % 36000,
            (int)lround(rate * 100), FLORA_NAVLOG_ATT_HEADING_VALID | FLORA_NAVLOG_ATT_MAG_VALID,
            400u, 90u);
    }
    if (tags_now && tick % 10 == 0) {
        double c = cos(th), s = sin(th);
        double camx = x + 295 * c, camy = y + 295 * s;
        char g[160];
        int n = 0, len = 0;
        char items[140] = "";
        for (int i = 0; i < 4 && n < 4; i++) {
            double dx = TAGS[i][0] - camx, dy = TAGS[i][1] - camy;
            double bear = atan2(dy, dx) - th;   /* CCW */
            bear = atan2(sin(bear), cos(bear));
            if (fabs(bear) > 0.5) continue;     /* pole widzenia */
            double d = hypot(dx, dy) + rnd_n(20 + 0.02 * hypot(dx, dy));
            len += snprintf(items + len, sizeof items - len, FLORA_NAVLOG_FMT_G_ITEM,
                            (unsigned)i, (unsigned)lround(d),
                            (int)lround(-(bear + rnd_n(0.02)) * 18000.0 / M_PI), 90u);
            n++;
        }
        snprintf(g, sizeof g, FLORA_NAVLOG_FMT_G_HEAD "%s\n", (unsigned)n, items);
        out("%s", g);
    }
    out(FLORA_NAVLOG_FMT_E, t_us, (long long)llround(cl), (long long)llround(cr));
    if (tick % 10 == 0) {
        flush();   /* paczka co 200 ms jak w hub_rec */
    }
}

int main(int argc, char **argv)
{
    if (argc == 3 && strcmp(argv[1], "--drop") == 0) {
        drop = atoi(argv[2]);
    }
    const double mm = M_PI * D_CFG / COUNTS;
    out(FLORA_NAVLOG_FMT_V, FLORA_NAVLOG_VERSION);
    out(FLORA_NAVLOG_FMT_P, mm, mm, 390.0, 295.0, 0.0, 50.06, 19.94, 6.0);
    out(FLORA_NAVLOG_FMT_C);
    for (int i = 0; i < 4; i++) {
        out(FLORA_NAVLOG_FMT_M, (unsigned)i, TAGS[i][0], TAGS[i][1]);
    }
    out(FLORA_NAVLOG_FMT_E, t_us, 0LL, 0LL);

    for (int side = 0; side < 4; side++) {
        for (int i = 0; i < 100; i++) step(0, 0, 1);                 /* postoj 2 s, paliki */
        int n = (int)(SIDE / (SPEED * DT));
        for (int i = 0; i < n; i++) step(SPEED * DT, SPEED * DT, 0);
        int m = (int)(90.0 / (TURN_DPS * DT));
        double arc = 0.5 * 390.0 * TURN_DPS * M_PI / 180.0 * DT;    /* wg kalibracji */
        for (int i = 0; i < m; i++) step(-arc, arc, 0);
    }
    for (int i = 0; i < 100; i++) step(0, 0, 1);
    flush();
    fprintf(stderr, "prawda na koncu: (%.0f, %.0f) mm, %.1f st\n", x, y, th * 180.0 / M_PI);
    return 0;
}
