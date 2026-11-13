/*
 * Odtworzenie nagrania przejazdu (cmd/rec) tym samym flora_navcore co na
 * robocie. Bez zmian strojenia wynik jest taki jak na robocie; z opcjami
 * mozna porownac warianty Q/R, podloge kursu, martwa strefe zyro albo
 * inna kalibracje kol - na tym samym przejezdzie.
 *
 *   cc -O2 -Wall -I../include -o replay replay.c ../flora_navcore.c \
 *      ../flora_ekf.c ../flora_sun.c ../flora_sungate.c -lm
 *   ./replay nagranie.log                      # podsumowanie wariantow
 *   ./replay --csv traj.csv --end "0 0" nagranie.log
 *
 * Opcje strojenia (domyslnie jak w firmware):
 *   --head-floor <st>   podloga sigma kursu glowicy        (4)
 *   --head-every <s>    kurs glowicy do EKF co             (0.5)
 *   --deadband <st>     martwa strefa zyro vs kola         (2)
 *   --recover <n>       odrzucen kursu do wyjscia z blokady (10, 0 = wyl.)
 *   --k-wheel <mm2/mm>  szum drogi kola                    (0.5)
 *   --q-b <rad2/s>      bladzenie b                        (3e-6)
 *   --gate1d <chi2>, --gate2d <chi2>                       (6.63, 9.21)
 * Kalibracja (nadpisuje linie P z nagrania):
 *   --track <mm>        rozstaw kol
 *   --scale-l <k>, --scale-r <k>   mnozniki drogi kola (np. inny wheel_mm)
 * Wyjscie:
 *   --csv <plik>        t_s i dla kazdego wariantu x, y, th, spos
 *   --every <n>         co ktory takt do CSV (5 = 10 Hz)
 *   --end "<x> <y>"     prawdziwe polozenie konca [mm] -> blad zamkniecia
 */
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "flora_navcore.h"
#include "flora_navlog.h"

#define R2D(r) ((r) * 180.0f / (float)M_PI)

typedef struct {
    flora_navcore_tuning_t tun;
    bool   have_track;
    float  track_mm;
    float  scale_l, scale_r;
    const char *csv;
    int    every;
    bool   have_end;
    float  end_x, end_y;
} opts_t;

static void usage(void)
{
    fprintf(stderr, "uzycie: replay [opcje] nagranie.log  (opcje na gorze replay.c)\n");
    exit(2);
}

static double num(const char *s)
{
    char *end;
    double v = strtod(s, &end);
    if (end == s || *end) {
        fprintf(stderr, "zla liczba: %s\n", s);
        exit(2);
    }
    return v;
}

/** Kolejne pole po przecinku; NULL na koncu linii. */
static char *field(char **p)
{
    if (!*p) return NULL;
    char *s = *p;
    char *c = strchr(s, ',');
    if (c) { *c = 0; *p = c + 1; } else { *p = NULL; }
    return s;
}

int main(int argc, char **argv)
{
    opts_t o = { .tun = FLORA_NAVCORE_DEFAULT_TUNING(), .scale_l = 1.0f, .scale_r = 1.0f, .every = 5 };
    const char *path = NULL;
    for (int i = 1; i < argc; i++) {
        const char *a = argv[i];
        const char *v = i + 1 < argc ? argv[i + 1] : NULL;
        #define ARG(name) (strcmp(a, name) == 0 && v && ++i)
        if      (ARG("--head-floor")) o.tun.head_sigma_floor_deg = (float)num(v);
        else if (ARG("--head-every")) o.tun.head_every_s = (float)num(v);
        else if (ARG("--deadband"))   o.tun.ekf.gyro_deadband_rad = (float)num(v) * (float)M_PI / 180.0f;
        else if (ARG("--recover"))    o.tun.ekf.heading_recover_n = (uint16_t)num(v);
        else if (ARG("--k-wheel"))    o.tun.k_wheel = (float)num(v);
        else if (ARG("--q-b"))        o.tun.ekf.q_b = (float)num(v);
        else if (ARG("--gate1d"))     o.tun.ekf.gate_1d = (float)num(v);
        else if (ARG("--gate2d"))     o.tun.ekf.gate_2d = (float)num(v);
        else if (ARG("--track"))      { o.have_track = true; o.track_mm = (float)num(v); }
        else if (ARG("--scale-l"))    o.scale_l = (float)num(v);
        else if (ARG("--scale-r"))    o.scale_r = (float)num(v);
        else if (ARG("--csv"))        o.csv = v;
        else if (ARG("--every"))      o.every = (int)num(v);
        else if (ARG("--end")) {
            if (sscanf(v, "%f %f", &o.end_x, &o.end_y) != 2) usage();
            o.have_end = true;
        }
        else if (a[0] == '-')         usage();
        else                          path = a;
        #undef ARG
    }
    if (!path) usage();
    if (o.every < 1) o.every = 1;

    FILE *f = fopen(path, "r");
    if (!f) { perror(path); return 1; }
    FILE *csv = NULL;
    if (o.csv) {
        csv = fopen(o.csv, "w");
        if (!csv) { perror(o.csv); return 1; }
    }

    static flora_navcore_t core;
    bool core_ready = false;
    float mm_l = 0, mm_r = 0;
    flora_navcore_geom_t geom = { .track_mm = 390.0f };

    flora_navcore_att_t att;
    flora_navcore_sun_t sun;
    flora_navcore_det_t dets[FLORA_NAVCORE_MAX_DETS];
    bool have_att = false, have_sun = false;
    int n_dets = -1;
    int64_t unix_s = 0;

    bool have_prev = false;
    int64_t prev_t = 0, prev_l = 0, prev_r = 0, t0 = 0;
    long seq = -1, gaps = 0, lines = 0, bad = 0, ticks = 0;
    char line[512];

    if (csv) {
        fprintf(csv, "t_s");
        for (int i = 0; i < FLORA_NAVCORE_VARIANTS; i++) {
            static const char *names[] = { "full", "odom", "head", "head_sun", "odom_tags", "no_odom" };
            fprintf(csv, ",%s_x,%s_y,%s_th,%s_spos", names[i], names[i], names[i], names[i]);
        }
        fprintf(csv, "\n");
    }

    while (fgets(line, sizeof line, f)) {
        lines++;
        line[strcspn(line, "\r\n")] = 0;
        if (!line[0]) continue;
        char *p = line + 1;
        if (*p == ',') p++;

        switch (line[0]) {
        case '#': {
            long s = strtol(line + 1, NULL, 10);
            if (seq >= 0 && s != seq + 1) {
                /* Zgubiona paczka: przyrost enkoderow przez dziure bylby
                 * jednym wielkim krokiem - zaczynamy od nowej probki. */
                gaps++;
                have_prev = false;
            }
            seq = s;
            break;
        }
        case 'V':
            if (atoi(p) != FLORA_NAVLOG_VERSION) {
                fprintf(stderr, "nagranie w wersji %d, replay zna %d\n", atoi(p), FLORA_NAVLOG_VERSION);
                return 1;
            }
            break;
        case 'P': {
            float v[8];
            for (int i = 0; i < 8; i++) {
                char *s = field(&p);
                if (!s) { bad++; goto next; }
                v[i] = strtof(s, NULL);
            }
            mm_l = v[0] * o.scale_l;
            mm_r = v[1] * o.scale_r;
            geom = (flora_navcore_geom_t){ .track_mm = o.have_track ? o.track_mm : v[2],
                                           .cam_x_mm = v[3], .cam_y_mm = v[4],
                                           .lat_deg = v[5], .lon_deg = v[6], .decl_deg = v[7] };
            if (!core_ready) {
                flora_navcore_init(&core, &o.tun, &geom);
                core_ready = true;
            } else {
                flora_navcore_set_geom(&core, &geom);
            }
            break;
        }
        case 'C':
            core.n_tags = 0;
            break;
        case 'M': {
            if (core.n_tags >= FLORA_NAVCORE_MAX_TAGS) break;
            char *id = field(&p), *x = field(&p), *y = field(&p);
            if (!id || !x || !y) { bad++; break; }
            core.tags[core.n_tags++] = (flora_navcore_tag_t){
                (uint16_t)atoi(id), strtof(x, NULL), strtof(y, NULL) };
            break;
        }
        case 'R': {
            float x, y, th;
            if (sscanf(p, "%f,%f,%f", &x, &y, &th) != 3 || !core_ready) { bad++; break; }
            flora_navcore_reset_pose(&core, x, y, th);
            break;
        }
        case 'A': {
            long age; unsigned cdeg, flags, sig, q; int cdps;
            if (sscanf(p, "%ld,%u,%d,%u,%u,%u", &age, &cdeg, &cdps, &flags, &sig, &q) != 6) { bad++; break; }
            att = (flora_navcore_att_t){
                .heading_valid = (flags & FLORA_NAVLOG_ATT_HEADING_VALID) != 0,
                .mag_valid     = (flags & FLORA_NAVLOG_ATT_MAG_VALID) != 0,
                .heading_deg   = (uint16_t)cdeg / 100.0f,
                .rate_dps      = (int16_t)cdps / 100.0f,
                .sigma_deg     = (uint16_t)sig / 100.0f,
                .quality       = (uint8_t)q,
                .age_us        = (int32_t)age,
            };
            have_att = true;
            break;
        }
        case 'S': {
            long long us; unsigned az, valid; float l[4];
            if (sscanf(p, "%lld,%u,%f,%f,%f,%f,%u", &us, &az, &l[0], &l[1], &l[2], &l[3], &valid) != 7) {
                bad++; break;
            }
            sun.valid = valid != 0;
            sun.azimuth_deg = (uint16_t)az / 100.0f;
            memcpy(sun.lux, l, sizeof l);
            unix_s = us;
            have_sun = true;
            break;
        }
        case 'G': {
            char *s = field(&p);
            if (!s) { bad++; break; }
            int n = atoi(s);
            n_dets = 0;
            for (int i = 0; i < n; i++) {
                char *id = field(&p), *d = field(&p), *b = field(&p), *q = field(&p);
                if (!id || !d || !b || !q) { bad++; break; }
                if (n_dets < FLORA_NAVCORE_MAX_DETS) {
                    dets[n_dets++] = (flora_navcore_det_t){
                        (uint16_t)atoi(id), (uint16_t)atoi(d), (int16_t)atoi(b) / 100.0f, (uint8_t)atoi(q) };
                }
            }
            break;
        }
        case 'E': {
            long long t, l, r;
            if (sscanf(p, "%lld,%lld,%lld", &t, &l, &r) != 3) { bad++; break; }
            if (!core_ready) {
                fprintf(stderr, "linia E przed P - brak parametrow\n");
                return 1;
            }
            if (have_prev) {
                flora_navcore_in_t in = {
                    .t_us   = t,
                    .dl_mm  = (float)(l - prev_l) * mm_l,
                    .dr_mm  = (float)(r - prev_r) * mm_r,
                    .dt_s   = (float)(t - prev_t) / 1e6f,
                    .att    = have_att ? &att : NULL,
                    .sun    = have_sun ? &sun : NULL,
                    .unix_s = unix_s,
                    .dets   = n_dets >= 0 ? dets : NULL,
                    .n_dets = n_dets,
                };
                flora_navcore_step(&core, &in);
                ticks++;
                if (csv && ticks % o.every == 0) {
                    fprintf(csv, "%.3f", (t - t0) / 1e6);
                    for (int i = 0; i < FLORA_NAVCORE_VARIANTS; i++) {
                        const flora_ekf_t *e = &core.v[i].ekf;
                        fprintf(csv, ",%.1f,%.1f,%.2f,%.1f", e->x[0], e->x[1], R2D(e->x[2]),
                                flora_ekf_pos_sigma_max(e));
                    }
                    fprintf(csv, "\n");
                }
            } else if (t0 == 0) {
                t0 = t;
            }
            /* Robot nie czyta lacza w takcie bez poprzedniej probki, wiec A/S/G
             * przed pierwszym E sie nie pojawiaja - czyscimy i tak. */
            have_prev = true;
            prev_t = t; prev_l = l; prev_r = r;
            have_att = have_sun = false;
            n_dets = -1;
            break;
        }
        default:
            bad++;
        }
    next:;
    }
    fclose(f);
    if (csv) fclose(csv);

    if (!core_ready) {
        fprintf(stderr, "brak linii P - to nie jest nagranie hub_rec?\n");
        return 1;
    }
    printf("%ld linii, %ld taktow (%.1f s), luk w nagraniu %ld, zlych linii %ld, skokow enkodera %u\n",
           lines, ticks, (prev_t - t0) / 1e6, gaps, bad, core.n_jumps);
    printf("%-10s %8s %8s %7s %7s %6s %5s %5s %5s %5s%s\n", "wariant", "x_mm", "y_mm", "th_st",
           "spos", "head", "sun", "tag", "rej", "rec", o.have_end ? "  blad_mm" : "");
    for (int i = 0; i < FLORA_NAVCORE_VARIANTS; i++) {
        const flora_navcore_variant_t *v = &core.v[i];
        const flora_ekf_t *e = &v->ekf;
        printf("%-10s %8.0f %8.0f %7.1f %7.0f %6u %5u %5u %5u %5u", v->name, e->x[0], e->x[1],
               R2D(e->x[2]), flora_ekf_pos_sigma_max(e), v->n_head, v->n_sun, v->n_tag,
               e->n_rejected, e->n_recovered);
        if (o.have_end) {
            printf("  %7.0f", hypotf(e->x[0] - o.end_x, e->x[1] - o.end_y));
        }
        printf("\n");
    }
    return 0;
}
