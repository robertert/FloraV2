/*
 * Sprawdzenie flora_ekf i flora_sun na komputerze.
 *
 * Symulacja testu z pracy: kwadrat 5 x 5 m, obroty o 90 st w miejscu
 * w rogach, powrot do startu. Prawda jest generowana z bledami, ktorych
 * filtr nie zna:
 *   - rozstaw kol 400 mm, a filtr zaklada 390 (blad obrotu ~2,5%),
 *   - enkodery z bledem skali +1% (lewy) i -0,5% (prawy),
 *   - losowy poslizg kol, rosnacy z droga.
 * Trzy przebiegi na tej samej prawdzie: sama odometria, + kurs z glowicy,
 * + znaczniki na palikach. Wypisuje blad zamkniecia trasy kazdego z nich -
 * to te trzy liczby maja trafic na wykres w pracy.
 *
 *   cc -O2 -Wall -I../include -o test test.c ../flora_navcore.c ../flora_ekf.c ../flora_sun.c ../flora_sungate.c -lm
 *   ./test            # sprawdzenia, kod wyjscia 0 = OK
 *   ./test --csv      # trajektoria: t,x,y,x_ekf,y_ekf,spos (przebieg pelny)
 */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "flora_ekf.h"
#include "flora_navcore.h"
#include "flora_sun.h"
#include "flora_sungate.h"

#define D2R(d) ((d) * (float)M_PI / 180.0f)
#define R2D(r) ((r) * 180.0f / (float)M_PI)

/* ---- generator losowy: dwa niezalezne strumienie, zeby prawda byla
 *      identyczna we wszystkich trzech przebiegach ---- */
typedef struct { uint64_t s; } rng_t;

static double rnd_u(rng_t *r)
{
    r->s = r->s * 6364136223846793005ULL + 1442695040888963407ULL;
    return ((r->s >> 11) + 0.5) / 9007199254740992.0;
}

static float rnd_n(rng_t *r, float sigma)
{
    double u1 = rnd_u(r), u2 = rnd_u(r);
    return (float)(sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2)) * sigma;
}

/* ---- parametry symulacji ---- */
#define DT          0.02f
#define SIDE_MM     5000.0f
#define SPEED_MM_S  100.0f
#define TURN_DPS    30.0f
#define W_TRUE      400.0f
#define W_NOMINAL   390.0f
#define ENC_ERR_L   0.010f
#define ENC_ERR_R  -0.005f
#define K_SLIP      0.3f     /* prawdziwy poslizg, mm^2/mm */
#define B_TRUE      1.0f     /* kurs kompasowy osi X mapy, rad */
#define HEAD_SIGMA  D2R(4.0f)
#define CAM_X       150.0f
#define CAM_Y       0.0f
#define TAG_RANGE   6000.0f

static const float TAGS[4][2] = {
    { -500.0f, -500.0f }, { 5500.0f, -500.0f }, { 5500.0f, 5500.0f }, { -500.0f, 5500.0f },
};

enum { RUN_ODOM, RUN_HEADING, RUN_TAGS };

typedef struct {
    float x, y, th;
} pose_t;

typedef struct {
    float err_xy;    /* blad pozycji na koncu, mm */
    float err_th;    /* blad kata, st */
    float err_b;     /* blad b, st */
    float spos;      /* sigma pozycji z filtra, mm */
    flora_ekf_t ekf;
} result_t;

static int s_csv;

static void tag_stop(flora_ekf_t *ekf, const pose_t *t, rng_t *meas)
{
    float c = cosf(t->th), s = sinf(t->th);
    float camx = t->x + CAM_X * c - CAM_Y * s;
    float camy = t->y + CAM_X * s + CAM_Y * c;
    for (int frame = 0; frame < 3; frame++) {
        for (int i = 0; i < 4; i++) {
            float dx = TAGS[i][0] - camx, dy = TAGS[i][1] - camy;
            float d = sqrtf(dx * dx + dy * dy);
            if (d > TAG_RANGE) continue;
            float sd = 20.0f + 0.02f * d;
            float sb = D2R(1.5f);
            float dm = d + rnd_n(meas, sd);
            float bm = flora_ekf_wrap(atan2f(dy, dx) - t->th + rnd_n(meas, sb));
            flora_ekf_update_tag(ekf, TAGS[i][0], TAGS[i][1], CAM_X, CAM_Y, dm, bm, sd, sb);
        }
    }
}

/** Jeden krok: komenda na kola, prawda, odometria, pomiary. */
static void step(flora_ekf_t *ekf, pose_t *t, float cl, float cr, int run,
                 rng_t *truth, rng_t *meas, int *tick)
{
    /* prawdziwa droga po ziemi = komenda + poslizg */
    float gl = cl + rnd_n(truth, sqrtf(K_SLIP * fabsf(cl)));
    float gr = cr + rnd_n(truth, sqrtf(K_SLIP * fabsf(cr)));
    float ds = 0.5f * (gl + gr), dth = (gr - gl) / W_TRUE;
    t->x += ds * cosf(t->th + 0.5f * dth);
    t->y += ds * sinf(t->th + 0.5f * dth);
    t->th = flora_ekf_wrap(t->th + dth);

    /* enkoder widzi obrot kola, nie ziemie - stad blad skali, bez poslizgu */
    flora_ekf_predict_odom(ekf, cl * (1.0f + ENC_ERR_L), cr * (1.0f + ENC_ERR_R), DT);

    (*tick)++;
    if (run >= RUN_HEADING && *tick % 25 == 0) {
        float psi = B_TRUE - t->th + rnd_n(meas, HEAD_SIGMA);
        flora_ekf_update_heading(ekf, psi, D2R(5.0f));
    }
    if (s_csv && run == RUN_TAGS && *tick % 10 == 0) {
        printf("%.2f,%.1f,%.1f,%.1f,%.1f,%.1f\n", *tick * DT, t->x, t->y,
               ekf->x[0], ekf->x[1], flora_ekf_pos_sigma_max(ekf));
    }
}

static result_t run_square(int run)
{
    flora_ekf_cfg_t cfg = FLORA_EKF_DEFAULT_CFG();
    cfg.track_mm = W_NOMINAL;
    result_t res;
    flora_ekf_init(&res.ekf, &cfg);
    flora_ekf_t *ekf = &res.ekf;

    rng_t truth = { 12345 }, meas = { 777 };
    pose_t t = { 0 };
    int tick = 0;

    /* Kurs startowy, zeby b bylo znane jeszcze przed jazda. */
    if (run >= RUN_HEADING) {
        for (int i = 0; i < 5; i++) {
            flora_ekf_update_heading(ekf, B_TRUE - t.th + rnd_n(&meas, HEAD_SIGMA), D2R(5.0f));
        }
    }

    for (int side = 0; side < 4; side++) {
        if (run == RUN_TAGS) tag_stop(ekf, &t, &meas);

        int n = (int)(SIDE_MM / (SPEED_MM_S * DT));
        float d = SPEED_MM_S * DT;
        for (int i = 0; i < n; i++) step(ekf, &t, d, d, run, &truth, &meas, &tick);

        /* obrot o 90 st CCW w miejscu */
        int m = (int)(90.0f / (TURN_DPS * DT));
        float arc = 0.5f * W_TRUE * D2R(TURN_DPS) * DT;
        for (int i = 0; i < m; i++) step(ekf, &t, -arc, arc, run, &truth, &meas, &tick);
    }
    if (run == RUN_TAGS) tag_stop(ekf, &t, &meas);

    float ex = ekf->x[0] - t.x, ey = ekf->x[1] - t.y;
    res.err_xy = sqrtf(ex * ex + ey * ey);
    res.err_th = fabsf(R2D(flora_ekf_wrap(ekf->x[2] - t.th)));
    res.err_b  = fabsf(R2D(flora_ekf_wrap(ekf->x[3] - B_TRUE)));
    res.spos   = flora_ekf_pos_sigma_max(ekf);
    return res;
}

/* ---- pomocnicze ---- */
static int s_fail;

static void check(int ok, const char *what)
{
    printf("%s  %s\n", ok ? "OK  " : "FAIL", what);
    if (!ok) s_fail++;
}

static int p_is_sane(const flora_ekf_t *e)
{
    for (int i = 0; i < FLORA_EKF_N; i++) {
        if (!(e->P[i][i] >= 0.0f) || !isfinite(e->P[i][i])) return 0;
        for (int j = 0; j < FLORA_EKF_N; j++) {
            if (fabsf(e->P[i][j] - e->P[j][i]) > 1e-3f * (fabsf(e->P[i][j]) + 1.0f)) return 0;
            /* |Pij| <= sqrt(Pii Pjj) - konieczne dla dodatniej okreslonosci */
            if (fabsf(e->P[i][j]) > 1.001f * sqrtf(e->P[i][i] * e->P[j][j]) + 1e-6f) return 0;
        }
    }
    return 1;
}

/** Dni od 1970-01-01 (algorytm Howarda Hinnanta). */
static int64_t days_from_civil(int y, unsigned m, unsigned d)
{
    y -= m <= 2;
    const int era = (y >= 0 ? y : y - 399) / 400;
    const unsigned yoe = (unsigned)(y - era * 400);
    const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return (int64_t)era * 146097 + (int64_t)doe - 719468;
}

/** Szuka gornowania slonca w danym dniu, krok 30 s. */
static void sun_noon(int y, unsigned m, unsigned d, double lat, double lon,
                     double *el, double *az, double *minute)
{
    int64_t t0 = days_from_civil(y, m, d) * 86400;
    *el = -90.0;
    for (int64_t t = t0; t < t0 + 86400; t += 30) {
        flora_sun_pos_t p = flora_sun_position(t, lat, lon);
        if (p.elevation_deg > *el) {
            *el = p.elevation_deg;
            *az = p.azimuth_deg;
            *minute = (double)(t - t0) / 60.0;
        }
    }
}

int main(int argc, char **argv)
{
    s_csv = argc > 1 && strcmp(argv[1], "--csv") == 0;
    if (s_csv) {
        printf("t,x,y,x_ekf,y_ekf,spos\n");
        run_square(RUN_TAGS);
        return 0;
    }

    /* ---- kwadrat 5 x 5 m ---- */
    result_t a = run_square(RUN_ODOM);
    result_t b = run_square(RUN_HEADING);
    result_t c = run_square(RUN_TAGS);

    printf("Kwadrat 5 x 5 m, blad zamkniecia trasy:\n");
    printf("  sama odometria   %7.0f mm  (kat %5.1f st, sigma z filtra %5.0f mm)\n", a.err_xy, a.err_th, a.spos);
    printf("  + kurs z glowicy %7.0f mm  (kat %5.1f st, sigma z filtra %5.0f mm)\n", b.err_xy, b.err_th, b.spos);
    printf("  + znaczniki      %7.0f mm  (kat %5.1f st, sigma z filtra %5.0f mm)\n", c.err_xy, c.err_th, c.spos);
    printf("  znaczniki: przyjete %u, odrzucone %u; b: blad %.2f st\n\n",
           c.ekf.n_tag, c.ekf.n_rejected, c.err_b);

    check(a.err_xy > b.err_xy && b.err_xy > c.err_xy, "blad maleje: odometria > + kurs > + znaczniki");
    check(b.err_th < 3.0f, "z kursem glowicy blad kata ponizej 3 st");
    check(c.err_b < 3.0f, "b zbiega do prawdziwego offsetu (< 3 st)");
    check(c.err_xy < 3.0f * c.spos + 50.0f, "blad pozycji miesci sie w 3 sigma filtra");
    check(p_is_sane(&a.ekf) && p_is_sane(&b.ekf) && p_is_sane(&c.ekf), "P symetryczna i dodatnio okreslona");

    /* ---- bramka: falszywy znacznik 3 m dalej ---- */
    flora_ekf_t e = c.ekf;
    uint32_t rej = e.n_rejected;
    float c0 = cosf(e.x[2]), s0 = sinf(e.x[2]);
    float camx = e.x[0] + CAM_X * c0, camy = e.x[1] + CAM_X * s0;
    float dx = TAGS[0][0] - camx, dy = TAGS[0][1] - camy;
    float d = sqrtf(dx * dx + dy * dy);
    bool took = flora_ekf_update_tag(&e, TAGS[0][0], TAGS[0][1], CAM_X, CAM_Y, d + 3000.0f,
                                     atan2f(dy, dx) - e.x[2], 20.0f + 0.02f * d, D2R(1.5f));
    check(!took && e.n_rejected == rej + 1, "falszywy znacznik odrzucony przez bramke Mahalanobisa");

    /* ---- zawijanie kata przez +-pi ---- */
    flora_ekf_cfg_t cfg = FLORA_EKF_DEFAULT_CFG();
    flora_ekf_init(&e, &cfg);
    e.x[3] = 0.0f;
    e.P[3][3] = 1e-6f;
    flora_ekf_set_pose(&e, 0, 0, D2R(178.0f), 10.0f, D2R(5.0f), false);
    /* prawdziwy kat -178 st, czyli 4 st dalej przez pi; psi = b - th */
    for (int i = 0; i < 20; i++) flora_ekf_update_heading(&e, -D2R(-178.0f), D2R(1.0f));
    check(fabsf(R2D(flora_ekf_wrap(e.x[2] - D2R(-178.0f)))) < 0.5f && fabsf(e.x[2]) > 3.0f,
          "kat przechodzi przez +-pi bez skoku przez zero");
    flora_ekf_predict_odom(&e, -100.0f, 100.0f, DT);
    check(e.x[2] > -(float)M_PI && e.x[2] <= (float)M_PI, "kat po predykcji w (-pi, pi]");

    /* ---- reset pozy z innym katem: kurs nie moze zostac odciety ---- */
    {
        flora_ekf_t r;
        flora_ekf_init(&r, &cfg);
        rng_t m = { 99 };
        float th = 0.0f;   /* prawdziwy kat robota */
        for (int i = 0; i < 20; i++) flora_ekf_update_heading(&r, B_TRUE - th + rnd_n(&m, HEAD_SIGMA), D2R(5.0f));
        /* Robot przeniesiony recznie i obrocony o 90 st; operator podaje to theta. */
        th = D2R(90.0f);
        flora_ekf_set_pose(&r, 1000.0f, 0.0f, th, 20.0f, D2R(2.0f), true);
        uint32_t acc0 = r.n_heading;
        for (int i = 0; i < 10; i++) flora_ekf_update_heading(&r, B_TRUE - th + rnd_n(&m, HEAD_SIGMA), D2R(5.0f));
        check(r.n_heading - acc0 >= 9 && fabsf(R2D(flora_ekf_wrap(r.x[2] - th))) < 3.0f,
              "pose_reset o 90 st: kurs dalej przyjmowany, theta zostaje");
        check(p_is_sane(&r), "pose_reset: P poprawna");
    }

    /* ---- poslizg: kola widza obrot o 90 st, robot stoi ---- */
    {
        flora_ekf_t r;
        flora_ekf_init(&r, &cfg);
        rng_t m = { 5 };
        for (int i = 0; i < 20; i++) flora_ekf_update_heading(&r, B_TRUE + rnd_n(&m, HEAD_SIGMA), D2R(5.0f));
        int n = (int)(90.0f / (TURN_DPS * DT));
        float arc = 0.5f * W_NOMINAL * D2R(TURN_DPS) * DT;
        for (int i = 0; i < n; i++) flora_ekf_predict_odom(&r, -arc, arc, DT);
        uint32_t rej0 = r.n_rejected;
        /* 20 pomiarow kursu (10 s przy 0,5 s), prawdziwe theta = 0 */
        for (int i = 0; i < 20; i++) flora_ekf_update_heading(&r, B_TRUE + rnd_n(&m, HEAD_SIGMA), D2R(5.0f));
        printf("  poslizg 90 st: odrzucone %u, wyjscia z blokady %u, blad kata %.1f st\n",
               r.n_rejected - rej0, r.n_recovered, fabsf(R2D(r.x[2])));
        check(r.n_recovered >= 1 && fabsf(R2D(r.x[2])) < 5.0f,
              "poslizg 90 st: kompas po blokadzie przywraca kat (< 5 st)");
        check(p_is_sane(&r), "poslizg: P poprawna");
    }

    /* ---- poslizg + zyroskop: kurs przyjety od razu, bez blokady ---- */
    {
        flora_ekf_t r;
        flora_ekf_init(&r, &cfg);
        rng_t m = { 6 };
        for (int i = 0; i < 20; i++) flora_ekf_update_heading(&r, B_TRUE + rnd_n(&m, HEAD_SIGMA), D2R(5.0f));
        int n = (int)(90.0f / (TURN_DPS * DT));
        float arc = 0.5f * W_NOMINAL * D2R(TURN_DPS) * DT;
        for (int i = 0; i < n; i++) {
            flora_ekf_predict_odom(&r, -arc, arc, DT);
            flora_ekf_observe_gyro(&r, 0.0f);   /* zyroskop: robot stoi */
        }
        uint32_t rej0 = r.n_rejected;
        bool took = flora_ekf_update_heading(&r, B_TRUE + rnd_n(&m, HEAD_SIGMA), D2R(5.0f));
        for (int i = 0; i < 5; i++) flora_ekf_update_heading(&r, B_TRUE + rnd_n(&m, HEAD_SIGMA), D2R(5.0f));
        printf("  poslizg 90 st z zyroskopem: pierwszy kurs %s, blad kata %.1f st\n",
               took ? "przyjety" : "odrzucony", fabsf(R2D(r.x[2])));
        check(took && r.n_rejected == rej0 && fabsf(R2D(r.x[2])) < 5.0f,
              "poslizg 90 st + zyro: pierwszy kurs przyjety, kat poprawiony");
        check(p_is_sane(&r), "poslizg + zyro: P poprawna");
    }

    /* ---- zwykly skret z zyroskopem opoznionym o 60 ms: bez sztucznej niepewnosci ---- */
    {
        flora_ekf_t r, ref;
        flora_ekf_init(&r, &cfg);
        for (int i = 0; i < 20; i++) flora_ekf_update_heading(&r, B_TRUE, D2R(5.0f));
        ref = r;
        int n = (int)(90.0f / (TURN_DPS * DT));
        float arc = 0.5f * W_NOMINAL * D2R(TURN_DPS) * DT;
        const int lag = 3;   /* 3 takty = 60 ms */
        for (int i = 0; i < n + lag; i++) {
            float dl = i < n ? -arc : 0.0f, dr = i < n ? arc : 0.0f;
            flora_ekf_predict_odom(&r, dl, dr, DT);
            flora_ekf_predict_odom(&ref, dl, dr, DT);
            float g = (i >= lag && i < n + lag) ? 2.0f * arc / W_NOMINAL : 0.0f;
            flora_ekf_observe_gyro(&r, g * 1.01f);   /* + 1% bledu skali zyro */
        }
        float extra = R2D(sqrtf(r.P[2][2])) - R2D(sqrtf(ref.P[2][2]));
        printf("  zwykly skret 90 st, zyro opoznione 60 ms: dodatkowa sigma theta %.2f st\n", extra);
        check(extra < 0.1f, "zwykly skret: opoznienie zyro miesci sie w martwej strefie");
    }

    /* ---- poza z NVS po restarcie (flora_navcore_restore_pose) ---- */
    {
        static flora_navcore_t nc;
        flora_navcore_tuning_t tun = FLORA_NAVCORE_DEFAULT_TUNING();
        flora_navcore_geom_t geom = { .track_mm = 390.0f, .lat_deg = 50.06, .lon_deg = 19.94, .decl_deg = 6.0f };
        flora_navcore_init(&nc, &tun, &geom);
        flora_navcore_restore_pose(&nc, 1000.0f, 500.0f, 90.0f, 30.0f, 300.0f, 10.0f, 10.0f);
        int same = 1;
        for (int i = 0; i < FLORA_NAVCORE_VARIANTS; i++) {
            const flora_ekf_t *v = &nc.v[i].ekf;
            same &= fabsf(v->x[0] - 1000.0f) < 1e-3f && fabsf(v->x[1] - 500.0f) < 1e-3f &&
                    fabsf(R2D(v->x[2]) - 90.0f) < 1e-3f && fabsf(R2D(v->x[3]) - 30.0f) < 1e-3f &&
                    fabsf(sqrtf(v->P[0][0]) - 300.0f) < 0.01f && p_is_sane(v);
        }
        check(same, "odtworzenie pozy: wszystkie warianty, z b i zadana niepewnoscia");

        /* Kurs zgodny z zapisanym b (psi = b - theta = -60 = 300 st):
         * przyjety od razu, bez odrzucen i bez wyjscia z blokady. */
        flora_navcore_att_t att = { .heading_valid = true, .mag_valid = true,
                                    .heading_deg = 300.0f, .sigma_deg = 3.0f, .age_us = -1 };
        int64_t t = 1000000;
        for (int i = 0; i < 100; i++) {
            flora_navcore_in_t in = { .t_us = t, .dt_s = 0.02f, .att = &att };
            flora_navcore_step(&nc, &in);
            t += 20000;
        }
        const flora_ekf_t *f = &nc.v[0].ekf;
        check(nc.v[0].n_head > 0 && f->n_rejected == 0 && fabsf(R2D(f->x[2]) - 90.0f) < 2.0f,
              "po odtworzeniu kurs przyjety od razu, theta zostaje");
    }

    /* ---- slonce (Krakow) ---- */
    const double lat = 50.06, lon = 19.94;
    double el, az, minute;
    sun_noon(2026, 3, 20, lat, lon, &el, &az, &minute);
    printf("\n20.03.2026 Krakow: gorowanie %02d:%02d UTC, elewacja %.2f st, azymut %.1f st\n",
           (int)minute / 60, (int)minute % 60, el, az);
    check(fabs(el - (90.0 - lat)) < 0.6, "rownonoc: elewacja w poludnie ~ 90 - szerokosc");
    check(fabs(az - 180.0) < 3.0, "poludnie sloneczne: azymut ~180 st");
    check(fabs(minute - (10 * 60 + 48)) < 5.0, "gorowanie ok. 10:48 UTC (dlugosc + rownanie czasu)");
    sun_noon(2026, 6, 21, lat, lon, &el, &az, &minute);
    printf("21.06.2026 Krakow: elewacja w poludnie %.2f st\n", el);
    check(fabs(el - (90.0 - lat + 23.44)) < 0.6, "przesilenie letnie: elewacja ~ 90 - szer. + 23,44");
    flora_sun_pos_t morning = flora_sun_position(days_from_civil(2026, 6, 21) * 86400 + 4 * 3600, lat, lon);
    check(morning.azimuth_deg > 45.0 && morning.azimuth_deg < 90.0,
          "06:00 czasu lokalnego latem: slonce na polnocnym wschodzie");

    /* ---- model bez odometrii ---- */
    flora_ekf_init(&e, &cfg);
    for (int i = 0; i < 500; i++) flora_ekf_predict_static(&e, DT, 1.0e4f, 0.27f);
    float sp = flora_ekf_pos_sigma_max(&e);
    check(e.x[0] == 0.0f && fabsf(sp - 316.2f) < 5.0f,
          "bez odometrii: poza stoi, sigma rosnie jak pierwiastek z czasu (10 s -> 316 mm)");

    /* ---- bramki kompasu slonecznego ---- */
    printf("\nKompas sloneczny (21.06.2026 12:00 CEST, Krakow):\n");
    flora_sungate_cfg_t gcfg = FLORA_SUNGATE_DEFAULT_CFG();
    gcfg.lat_deg = lat;
    gcfg.lon_deg = lon;
    gcfg.declination_deg = 6.0f;
    flora_sungate_t g;
    flora_sungate_out_t go;
    const int64_t noon = days_from_civil(2026, 6, 21) * 86400 + 10 * 3600;
    const float sun_az = (float)flora_sun_position(noon, lat, lon).azimuth_deg;
    const float psi_true = D2R(57.0f);   /* prawdziwy kurs magnetyczny glowicy */

    /* Wejscie zgodne z rzeczywistoscia: robot stoi, slonce swieci. */
    flora_sungate_in_t in = {
        .lux = { 80000.0f, 60000.0f, 20000.0f, 30000.0f },
        .azimuth_meas_deg = sun_az - 6.0f - R2D(psi_true),
        .unix_s = noon,
        .th_ref_rad = 0.0f,
        .head_mag_ok = true,
        .head_psi_rad = psi_true + D2R(3.0f),   /* magnetometr myli sie o 3 st */
    };

    flora_sungate_init(&g, &gcfg);
    flora_sungate_result_t r1 = flora_sungate_eval(&g, &in, &go);
    flora_sungate_result_t r2 = flora_sungate_eval(&g, &in, &go);
    flora_sungate_result_t r3 = flora_sungate_eval(&g, &in, &go);
    check(r1 == FLORA_SUNGATE_UNSTABLE && r2 == FLORA_SUNGATE_UNSTABLE && r3 == FLORA_SUNGATE_OK,
          "slonce: przyjete dopiero po 3 zgodnych pomiarach");
    check(fabsf(R2D(flora_ekf_wrap(go.psi_rad - psi_true))) < 0.1f,
          "slonce: kurs z NOAA i krzyza zgodny z prawdziwym");
    check(fabsf(R2D(go.sigma_rad) - 15.0f) < 0.01f && fabsf(go.vs_mag_deg + 3.0f) < 0.1f,
          "slonce przy magnetometrze: sigma 15 st, roznica do magnetometru widoczna");

    /* Robot sie obraca miedzy pomiarami - stalosc liczona w ukladzie mapy. */
    flora_sungate_init(&g, &gcfg);
    flora_sungate_result_t rr = FLORA_SUNGATE_NONE;
    for (int i = 0; i < 4; i++) {
        float th = D2R(20.0f * i);
        flora_sungate_in_t t = in;
        t.th_ref_rad = th;
        t.head_psi_rad = psi_true - th;
        t.azimuth_meas_deg = sun_az - 6.0f - R2D(psi_true - th);
        rr = flora_sungate_eval(&g, &t, &go);
    }
    check(rr == FLORA_SUNGATE_OK, "slonce w trakcie obrotu robota: nadal przyjmowane");

    /* Lampa we wnetrzu: setki luksow. */
    flora_sungate_in_t lamp = in;
    for (int i = 0; i < 4; i++) lamp.lux[i] = 300.0f + 200.0f * i;
    flora_sungate_init(&g, &gcfg);
    for (int i = 0; i < 5; i++) rr = flora_sungate_eval(&g, &lamp, &go);
    check(rr == FLORA_SUNGATE_DIM, "lampa we wnetrzu (~900 lx): odrzucona jako nie-slonce");

    /* Mocny reflektor (25 tys. lx) z innego kierunku niz slonce. */
    flora_sungate_in_t spot = in;
    spot.lux[0] = 25000.0f;
    spot.azimuth_meas_deg = in.azimuth_meas_deg + 70.0f;
    flora_sungate_init(&g, &gcfg);
    for (int i = 0; i < 5; i++) rr = flora_sungate_eval(&g, &spot, &go);
    check(rr == FLORA_SUNGATE_MAG_CONFLICT, "reflektor 25 klx z innej strony: konflikt z magnetometrem");

    /* Odbicia: kierunek skacze o kilkadziesiat stopni. */
    flora_sungate_init(&g, &gcfg);
    for (int i = 0; i < 6; i++) {
        flora_sungate_in_t t = in;
        t.azimuth_meas_deg += (i % 2 ? 35.0f : -35.0f);
        rr = flora_sungate_eval(&g, &t, &go);
    }
    check(rr == FLORA_SUNGATE_UNSTABLE, "skaczacy kierunek (odbicia): odrzucony jako niestabilny");

    /* Noc i brak zegara. */
    flora_sungate_in_t night = in;
    night.unix_s = days_from_civil(2026, 6, 21) * 86400 + 22 * 3600;
    flora_sungate_init(&g, &gcfg);
    check(flora_sungate_eval(&g, &night, &go) == FLORA_SUNGATE_LOW, "noc (24:00 CEST): slonce pod horyzontem");
    flora_sungate_in_t noclock = in;
    noclock.unix_s = 0;
    check(flora_sungate_eval(&g, &noclock, &go) == FLORA_SUNGATE_LOW, "brak czasu UTC: pomijane");

    /* Bez magnetometru slonce jest jedynym kursem - przyjete ostrozniej. */
    flora_sungate_in_t nomag = in;
    nomag.head_mag_ok = false;
    flora_sungate_init(&g, &gcfg);
    for (int i = 0; i < 3; i++) rr = flora_sungate_eval(&g, &nomag, &go);
    check(rr == FLORA_SUNGATE_OK && fabsf(R2D(go.sigma_rad) - 20.0f) < 0.01f,
          "bez magnetometru: przyjete z sigma 20 st");

    /* Waga w EKF: przy kursie z glowicy slonce prawie nie rusza kata. */
    flora_ekf_init(&e, &cfg);
    for (int i = 0; i < 20; i++) flora_ekf_update_heading(&e, psi_true, D2R(4.0f));
    /* Liczy sie przewidywany kurs b - theta (na swiezym filtrze theta jest
     * pewne, wiec poprawka idzie w b - ale efekt dla kursu jest ten sam). */
    float psi_before = e.x[3] - e.x[2];
    flora_ekf_update_heading(&e, psi_true + D2R(15.0f), D2R(15.0f));
    float moved_deg = fabsf(R2D(flora_ekf_wrap(e.x[3] - e.x[2] - psi_before)));
    printf("  slonce 15 st obok magnetometru przesuwa kurs o %.2f st\n", moved_deg);
    check(moved_deg < 1.0f, "magnetometr dominuje: pojedynczy pomiar slonca przesuwa kurs < 1 st");

    printf("%s\n", s_fail ? "WYNIK: BLEDY" : "WYNIK: OK");
    return s_fail ? 1 : 0;
}
