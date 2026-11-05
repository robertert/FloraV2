#include <math.h>
#include <string.h>

#include "flora_magcal.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define MIN_SAMPLES     150
#define SECTORS         12
#define MIN_SECTORS     10
#define MAX_RMS_PCT     5.0f
#define MAX_AXIS_RATIO  1.6f

void flora_magcal_identity(flora_magcal_t *cal)
{
    memset(cal, 0, sizeof(*cal));
    cal->w[0][0] = 1.0f;
    cal->w[1][1] = 1.0f;
}

void flora_magcal_begin(flora_magcal_collector_t *c)
{
    c->n = 0;
    c->v_sum = 0.0;
}

bool flora_magcal_add(flora_magcal_collector_t *c, float x, float y, float v)
{
    if (c->n >= FLORA_MAGCAL_MAX_SAMPLES) {
        return false;
    }
    c->x[c->n] = x;
    c->y[c->n] = y;
    c->n++;
    c->v_sum += v;
    return true;
}

void flora_magcal_apply(const flora_magcal_t *cal, float x, float y, float *out_x, float *out_y)
{
    float dx = x - cal->ox, dy = y - cal->oy;
    *out_x = cal->w[0][0] * dx + cal->w[0][1] * dy;
    *out_y = cal->w[1][0] * dx + cal->w[1][1] * dy;
}

const char *flora_magcal_status_name(flora_magcal_status_t s)
{
    switch (s) {
        case FLORA_MAGCAL_OK:         return "ok";
        case FLORA_MAGCAL_TOO_FEW:    return "za malo probek";
        case FLORA_MAGCAL_COVERAGE:   return "nie obrocono pelnego kola";
        case FLORA_MAGCAL_FIT:        return "punkty nie tworza elipsy";
        case FLORA_MAGCAL_RESIDUAL:   return "za duzy rozrzut";
        case FLORA_MAGCAL_AXIS_RATIO: return "elipsa zbyt splaszczona";
        default:                      return "?";
    }
}

/** Eliminacja Gaussa z wyborem elementu glownego. false gdy osobliwy. */
static bool solve5(double A[5][5], double b[5], double x[5])
{
    for (int c = 0; c < 5; c++) {
        int p = c;
        for (int r = c + 1; r < 5; r++) {
            if (fabs(A[r][c]) > fabs(A[p][c])) p = r;
        }
        if (fabs(A[p][c]) < 1e-12) return false;
        if (p != c) {
            for (int k = 0; k < 5; k++) { double t = A[c][k]; A[c][k] = A[p][k]; A[p][k] = t; }
            double t = b[c]; b[c] = b[p]; b[p] = t;
        }
        for (int r = c + 1; r < 5; r++) {
            double f = A[r][c] / A[c][c];
            for (int k = c; k < 5; k++) A[r][k] -= f * A[c][k];
            b[r] -= f * b[c];
        }
    }
    for (int r = 4; r >= 0; r--) {
        double s = b[r];
        for (int k = r + 1; k < 5; k++) s -= A[r][k] * x[k];
        x[r] = s / A[r][r];
    }
    return true;
}

flora_magcal_report_t flora_magcal_fit(const flora_magcal_collector_t *c, flora_magcal_t *cal)
{
    flora_magcal_report_t rep = { .status = FLORA_MAGCAL_TOO_FEW, .n = (uint16_t)c->n };
    if (c->n < MIN_SAMPLES) {
        return rep;
    }

    /* Normalizacja: srednia 0, rozrzut ~1 - inaczej x^2 rzedu 2500 obok
     * wyrazow rzedu 1 i uklad jest zle uwarunkowany. */
    double mx = 0, my = 0;
    for (int i = 0; i < c->n; i++) { mx += c->x[i]; my += c->y[i]; }
    mx /= c->n; my /= c->n;
    double s = 0;
    for (int i = 0; i < c->n; i++) {
        double dx = c->x[i] - mx, dy = c->y[i] - my;
        s += dx * dx + dy * dy;
    }
    s = sqrt(s / c->n);
    if (s < 1e-6) {
        rep.status = FLORA_MAGCAL_FIT;
        return rep;
    }

    /* Rownania normalne dla [A B C D E] przy prawej stronie 1. */
    double N[5][5] = {{0}}, r[5] = {0};
    for (int i = 0; i < c->n; i++) {
        double u = (c->x[i] - mx) / s, v = (c->y[i] - my) / s;
        double row[5] = { u * u, u * v, v * v, u, v };
        for (int a = 0; a < 5; a++) {
            r[a] += row[a];
            for (int b = 0; b < 5; b++) N[a][b] += row[a] * row[b];
        }
    }
    double p[5];
    if (!solve5(N, r, p)) {
        rep.status = FLORA_MAGCAL_FIT;
        return rep;
    }
    double A = p[0], B = p[1], C = p[2], D = p[3], E = p[4];

    /* Elipsa wymaga dodatnio okreslonej czesci kwadratowej. */
    if (A <= 0 || C <= 0 || 4 * A * C - B * B <= 0) {
        rep.status = FLORA_MAGCAL_FIT;
        return rep;
    }
    /* Srodek: gradient zero, [2A B; B 2C] c = -[D E]. */
    double det = 4 * A * C - B * B;
    double u0 = (-2 * C * D + B * E) / det;
    double v0 = (-2 * A * E + B * D) / det;
    /* Po przesunieciu: x' M x' = k, k = 1 + c' M c. */
    double k = 1.0 + A * u0 * u0 + B * u0 * v0 + C * v0 * v0;
    if (k <= 0) {
        rep.status = FLORA_MAGCAL_FIT;
        return rep;
    }
    /* Q = M / (k s^2) w jednostkach uT: elipsa x' Q x' = 1. */
    double q00 = A / (k * s * s), q01 = 0.5 * B / (k * s * s), q11 = C / (k * s * s);

    /* Wartosci wlasne symetrycznej 2x2. */
    double tr = q00 + q11, dd = sqrt(0.25 * (q00 - q11) * (q00 - q11) + q01 * q01);
    double l1 = 0.5 * tr + dd, l2 = 0.5 * tr - dd;   /* l1 >= l2 > 0 */
    if (l2 <= 0) {
        rep.status = FLORA_MAGCAL_FIT;
        return rep;
    }
    rep.axis_ratio = (float)sqrt(l1 / l2);

    /* Wektor wlasny dla l1. */
    double ex, ey;
    if (fabs(q01) > 1e-15) { ex = l1 - q11; ey = q01; }
    else if (q00 >= q11)   { ex = 1; ey = 0; }
    else                   { ex = 0; ey = 1; }
    double en = sqrt(ex * ex + ey * ey);
    ex /= en; ey /= en;

    /* W = R * sqrt(Q), R = sqrt(a b) - okrag o polu elipsy, polosie a = 1/sqrt(l). */
    double R = 1.0 / sqrt(sqrt(l1 * l2));
    double s1 = sqrt(l1) * R, s2 = sqrt(l2) * R;
    flora_magcal_t out;
    out.ox = (float)(mx + s * u0);
    out.oy = (float)(my + s * v0);
    out.w[0][0] = (float)(s1 * ex * ex + s2 * ey * ey);
    out.w[0][1] = (float)((s1 - s2) * ex * ey);
    out.w[1][0] = out.w[0][1];
    out.w[1][1] = (float)(s1 * ey * ey + s2 * ex * ex);
    out.h_ref_ut = (float)R;
    out.dip_ref_deg = (float)(atan2(c->v_sum / c->n, R) * 180.0 / M_PI);

    /* Rozrzut i pokrycie liczone juz po korekcji. */
    bool seen[SECTORS] = { false };
    double e2 = 0;
    for (int i = 0; i < c->n; i++) {
        float cx, cy;
        flora_magcal_apply(&out, c->x[i], c->y[i], &cx, &cy);
        double rr = sqrt((double)cx * cx + (double)cy * cy);
        e2 += (rr - R) * (rr - R);
        double ang = atan2(cy, cx) + M_PI;   /* 0..2pi */
        int sec = (int)(ang / (2 * M_PI) * SECTORS);
        if (sec >= SECTORS) sec = SECTORS - 1;
        seen[sec] = true;
    }
    rep.rms_pct = (float)(100.0 * sqrt(e2 / c->n) / R);
    for (int i = 0; i < SECTORS; i++) rep.sectors += seen[i];

    if (rep.sectors < MIN_SECTORS)       rep.status = FLORA_MAGCAL_COVERAGE;
    else if (rep.rms_pct > MAX_RMS_PCT)  rep.status = FLORA_MAGCAL_RESIDUAL;
    else if (rep.axis_ratio > MAX_AXIS_RATIO) rep.status = FLORA_MAGCAL_AXIS_RATIO;
    else {
        rep.status = FLORA_MAGCAL_OK;
        *cal = out;
    }
    return rep;
}
