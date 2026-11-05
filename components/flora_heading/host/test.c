/*
 * Sprawdzenie flora_magcal, flora_hkf i flora_magdist na komputerze.
 *
 *   cc -O2 -Wall -I../include -o test test.c ../flora_magcal.c ../flora_hkf.c ../flora_magdist.c -lm
 *   ./test            # sprawdzenia, kod wyjscia 0 = OK
 *   ./test --csv      # chmura punktow przed i po kalibracji: x,y,x_kal,y_kal (wykres do pracy)
 */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "flora_hkf.h"
#include "flora_magcal.h"
#include "flora_magdist.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define D2R(d) ((d) * (float)M_PI / 180.0f)
#define R2D(r) ((r) * 180.0f / (float)M_PI)

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

static int s_fail;

static void check(int ok, const char *what)
{
    printf("%s  %s\n", ok ? "OK  " : "FAIL", what);
    if (!ok) s_fail++;
}

static float diff180(float a, float b)
{
    float d = fmodf(a - b + 540.0f, 360.0f);
    if (d < 0.0f) d += 360.0f;
    return d - 180.0f;
}

/* ---- model znieksztalcen ----
 * Pole poziome 19 uT (Krakow), inklinacja 66 st -> pionowa ~43 uT.
 * Hard iron (12, -7) uT, soft iron: splaszczenie 1,25 pod katem 30 st. */
#define H_TRUE   19.0f
#define V_TRUE   43.0f
#define HI_X     12.0f
#define HI_Y     -7.0f

static void distort(float hx, float hy, float *mx, float *my)
{
    const float a = D2R(30.0f), k = 1.25f;
    float c = cosf(a), s = sinf(a);
    /* S = R diag(k, 1/k) R' */
    float s00 = k * c * c + s * s / k, s01 = (k - 1.0f / k) * c * s, s11 = k * s * s + c * c / k;
    *mx = s00 * hx + s01 * hy + HI_X;
    *my = s01 * hx + s11 * hy + HI_Y;
}

/** Kurs z pola w osiach czujnika: atan2(y, x), jak w head_heading_tilt_compensated. */
static float heading_of(float x, float y)
{
    float h = R2D(atan2f(y, x));
    return h < 0 ? h + 360.0f : h;
}

/** Pole poziome widziane przy kursie glowicy psi (CW od N): czujnik obrocony o psi. */
static void field_at(float psi_deg, float *hx, float *hy)
{
    /* Kurs psi = atan2(my, mx), wiec pole w osiach czujnika ma kat psi. */
    *hx = H_TRUE * cosf(D2R(psi_deg));
    *hy = H_TRUE * sinf(D2R(psi_deg));
}

static flora_magcal_collector_t s_col;

int main(int argc, char **argv)
{
    int csv = argc > 1 && strcmp(argv[1], "--csv") == 0;
    rng_t rng = { 4242 };

    /* ================= KALIBRACJA ================= */
    flora_magcal_begin(&s_col);
    for (int i = 0; i < 400; i++) {
        float psi = 720.0f * i / 400.0f;   /* dwa obroty */
        float hx, hy, mx, my;
        field_at(psi, &hx, &hy);
        distort(hx, hy, &mx, &my);
        flora_magcal_add(&s_col, mx + rnd_n(&rng, 0.3f), my + rnd_n(&rng, 0.3f), V_TRUE);
    }
    flora_magcal_t cal;
    flora_magcal_identity(&cal);
    flora_magcal_report_t rep = flora_magcal_fit(&s_col, &cal);

    if (csv) {
        printf("x,y,x_kal,y_kal\n");
        for (int i = 0; i < s_col.n; i++) {
            float cx, cy;
            flora_magcal_apply(&cal, s_col.x[i], s_col.y[i], &cx, &cy);
            printf("%.2f,%.2f,%.2f,%.2f\n", s_col.x[i], s_col.y[i], cx, cy);
        }
        return 0;
    }

    printf("Kalibracja: %s, %u probek, %u/12 sektorow, osie %.2f, rozrzut %.2f%%\n",
           flora_magcal_status_name(rep.status), rep.n, rep.sectors, rep.axis_ratio, rep.rms_pct);
    printf("  hard iron (%.2f, %.2f) uT [prawda (%.1f, %.1f)], pole %.2f uT, inklinacja %.1f st\n",
           cal.ox, cal.oy, HI_X, HI_Y, cal.h_ref_ut, cal.dip_ref_deg);
    check(rep.status == FLORA_MAGCAL_OK, "dwa obroty z szumem: kalibracja udana");
    check(fabsf(cal.ox - HI_X) < 0.3f && fabsf(cal.oy - HI_Y) < 0.3f, "hard iron odzyskany (< 0,3 uT)");
    check(fabsf(rep.axis_ratio - 1.5625f) < 0.05f, "splaszczenie odzyskane (1,25^2 = 1,56)");
    check(fabsf(cal.dip_ref_deg - R2D(atan2f(V_TRUE, cal.h_ref_ut))) < 0.1f, "inklinacja odniesienia policzona");

    /* Blad kursu przed i po: soft iron obraca kat, wiec porownujemy
     * zmiane kursu wzgledem pierwszego punktu (staly offset kata przechodzi
     * do b w EKF bryly i nie szkodzi). */
    float worst_raw = 0, worst_cal = 0;
    float raw0 = 0, cal0 = 0;
    for (int i = 0; i <= 360; i += 5) {
        float hx, hy, mx, my, cx, cy;
        field_at((float)i, &hx, &hy);
        distort(hx, hy, &mx, &my);
        flora_magcal_apply(&cal, mx, my, &cx, &cy);
        float raw = heading_of(mx, my), kal = heading_of(cx, cy);
        if (i == 0) { raw0 = raw; cal0 = kal; continue; }
        float er = fabsf(diff180(raw - raw0, (float)i));
        float ec = fabsf(diff180(kal - cal0, (float)i));
        if (er > worst_raw) worst_raw = er;
        if (ec > worst_cal) worst_cal = ec;
    }
    printf("  najgorszy blad kursu: bez kalibracji %.1f st, po kalibracji %.2f st\n", worst_raw, worst_cal);
    check(worst_raw > 20.0f && worst_cal < 1.0f, "kalibracja zbija blad kursu z kilkudziesieciu st do < 1 st");

    /* Pol obrotu - za malo, zeby wyznaczyc elipse. */
    flora_magcal_begin(&s_col);
    for (int i = 0; i < 300; i++) {
        float hx, hy, mx, my;
        field_at(180.0f * i / 300.0f, &hx, &hy);
        distort(hx, hy, &mx, &my);
        flora_magcal_add(&s_col, mx + rnd_n(&rng, 0.3f), my + rnd_n(&rng, 0.3f), V_TRUE);
    }
    flora_magcal_t untouched = cal;
    rep = flora_magcal_fit(&s_col, &untouched);
    check(rep.status != FLORA_MAGCAL_OK && memcmp(&untouched, &cal, sizeof cal) == 0,
          "pol obrotu: odrzucone, poprzednia kalibracja nietknieta");

    flora_magcal_begin(&s_col);
    for (int i = 0; i < 50; i++) flora_magcal_add(&s_col, (float)i, (float)i, 0);
    check(flora_magcal_fit(&s_col, &untouched).status == FLORA_MAGCAL_TOO_FEW, "za malo probek: odrzucone");

    /* ================= KALMAN KURSU ================= */
    printf("\nKalman kursu:\n");
    flora_hkf_cfg_t kcfg = FLORA_HKF_DEFAULT_CFG();
    flora_hkf_t kf;
    const float dt = 1.0f / 60.0f, bias_true = 0.5f;
    float psi_true = 350.0f;
    flora_hkf_init(&kf, &kcfg, psi_true, 4.0f);
    for (int i = 0; i < 60 * 120; i++) {   /* 2 min, obrot 10 st/s w kolko */
        float omega = 10.0f;
        psi_true = fmodf(psi_true + omega * dt, 360.0f);
        flora_hkf_predict(&kf, omega + bias_true + rnd_n(&rng, 0.2f), dt);
        if (i % 6 == 0) flora_hkf_update(&kf, psi_true + rnd_n(&rng, 3.0f), 4.0f);
    }
    printf("  dryf: oszacowany %.3f st/s (prawda %.2f), sigma kursu %.2f st, blad %.2f st\n",
           kf.bz, bias_true, flora_hkf_sigma(&kf), fabsf(diff180(kf.psi, psi_true)));
    check(fabsf(kf.bz - bias_true) < 0.05f, "dryf zyroskopu zbiega (0,5 st/s, blad < 0,05)");
    check(fabsf(diff180(kf.psi, psi_true)) < 3.0f, "kurs przez 0/360 bez skokow, blad < 3 st");
    check(flora_hkf_sigma(&kf) < 3.0f && flora_hkf_sigma(&kf) >= kcfg.sigma_floor,
          "z magnetometrem sigma kursu 2-3 st (nie ponizej progu)");

    /* Bez magnetometru sigma rosnie. */
    float s0 = flora_hkf_sigma(&kf);
    for (int i = 0; i < 60 * 60; i++) flora_hkf_predict(&kf, 0.0f, dt);
    float s1 = flora_hkf_sigma(&kf);
    printf("  bez magnetometru przez 60 s: sigma %.2f -> %.2f st\n", s0, s1);
    check(s1 > 1.5f * s0, "bez magnetometru sigma kursu rosnie");

    /* Skok magnetometru o 40 st (stal obok trasy) - odrzucony. */
    flora_hkf_init(&kf, &kcfg, 100.0f, 2.0f);
    for (int i = 0; i < 100; i++) {
        flora_hkf_predict(&kf, 0.0f, dt);
        if (i % 6 == 0) flora_hkf_update(&kf, 100.0f + rnd_n(&rng, 2.0f), 4.0f);
    }
    uint32_t rej0 = kf.n_rejected;
    bool took = flora_hkf_update(&kf, 140.0f, 4.0f);
    check(!took && kf.n_rejected == rej0 + 1 && fabsf(diff180(kf.psi, 100.0f)) < 2.0f,
          "skok magnetometru o 40 st: odrzucony, kurs nietkniety");

    /* ================= ZAKLOCENIA ================= */
    printf("\nWykrywanie zaklocen:\n");
    flora_magdist_cfg_t dcfg = FLORA_MAGDIST_DEFAULT_CFG();
    flora_magdist_t md;
    flora_magdist_init(&md, &dcfg);
    flora_magdist_set_ref(&md, true, 19.0f, 66.0f);
    bool dist = false;
    for (int i = 0; i < 10; i++) dist = flora_magdist_step(&md, 100, 19.2f, 66.3f, 47.0f, true);
    check(!dist, "pole jak przy kalibracji: czysto");
    dist = flora_magdist_step(&md, 100, 19.0f * 1.2f, 66.0f, 47.0f, true);
    check(dist && md.cause == FLORA_MAGDIST_CAUSE_FIELD, "pole poziome +20%: zaklocenie");
    dist = flora_magdist_step(&md, 100, 19.0f, 66.0f, 47.0f, true);
    check(dist, "zaraz po powrocie: flaga trzyma (histereza)");
    for (int i = 0; i < 20; i++) dist = flora_magdist_step(&md, 100, 19.0f, 66.0f, 47.0f, true);
    check(!dist, "po 2 s spokoju: czysto");
    dist = flora_magdist_step(&md, 100, 19.0f, 75.0f, 47.0f, true);
    check(dist && md.cause == FLORA_MAGDIST_CAUSE_DIP, "inklinacja +9 st: zaklocenie");

    flora_magdist_init(&md, &dcfg);
    flora_magdist_set_ref(&md, true, 19.0f, 66.0f);
    for (int i = 0; i < 15; i++) dist = flora_magdist_step(&md, 100, 19.0f, 66.0f, 47.0f, false);
    check(!dist, "1,5 s niezgodnosci z zyro: jeszcze nie");
    for (int i = 0; i < 6; i++) dist = flora_magdist_step(&md, 100, 19.0f, 66.0f, 47.0f, false);
    check(dist && md.cause == FLORA_MAGDIST_CAUSE_GYRO,
          "ponad 2 s niezgodnosci z zyro (pole w normie, ale obrocone): zaklocenie");

    flora_magdist_init(&md, &dcfg);
    dist = flora_magdist_step(&md, 100, 0, 0, 120.0f, true);
    check(dist && md.cause == FLORA_MAGDIST_CAUSE_RANGE, "bez kalibracji: 120 uT (magnes) wykryte");

    printf("%s\n", s_fail ? "WYNIK: BLEDY" : "WYNIK: OK");
    return s_fail ? 1 : 0;
}
