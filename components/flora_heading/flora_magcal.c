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
#define ITERATIONS      6
/** Fizyczna granica: zero AK09916 do 300 uT + otoczenie. Wiecej = zly fit. */
#define MAX_OFFSET_UT   500.0

#define DEG2RAD (M_PI / 180.0)

void flora_magcal_identity(flora_magcal_t *cal)
{
    memset(cal, 0, sizeof(*cal));
    cal->w[0][0] = 1.0f;
    cal->w[1][1] = 1.0f;
}

void flora_magcal_begin(flora_magcal_collector_t *c)
{
    c->n = 0;
}

bool flora_magcal_add(flora_magcal_collector_t *c, const float m[3],
                      float pitch_deg, float roll_deg)
{
    if (c->n >= FLORA_MAGCAL_MAX_SAMPLES) {
        return false;
    }
    c->x[c->n] = m[0];
    c->y[c->n] = m[1];
    c->z[c->n] = m[2];
    c->pitch[c->n] = pitch_deg;
    c->roll[c->n]  = roll_deg;
    c->n++;
    return true;
}

void flora_magcal_level(const float m[3], float pitch_deg, float roll_deg, float out[3])
{
    float p = pitch_deg * (float)DEG2RAD, r = roll_deg * (float)DEG2RAD;
    float sp = sinf(p), cp = cosf(p), sr = sinf(r), cr = cosf(r);
    out[0] =  m[0] * cp + m[1] * sr * sp + m[2] * cr * sp;
    out[1] =  m[1] * cr - m[2] * sr;
    out[2] = -m[0] * sp + m[1] * sr * cp + m[2] * cr * cp;
}

void flora_magcal_apply(const flora_magcal_t *cal, const float m[3],
                        float pitch_deg, float roll_deg, flora_magcal_out_t *out)
{
    /* Kolejnosc jak w fizyce: hard iron i soft iron siedza na plytce,
     * wiec korekta w osiach plytki, a poziomowanie dopiero na koncu. */
    float dx = m[0] - cal->ox, dy = m[1] - cal->oy, dz = m[2] - cal->oz;
    float e[3] = {
        cal->w[0][0] * dx + cal->w[0][1] * dy,
        cal->w[1][0] * dx + cal->w[1][1] * dy,
        dz,
    };
    float L[3];
    flora_magcal_level(e, pitch_deg, roll_deg, L);
    float hdg = atan2f(L[1], L[0]) * (float)(180.0 / M_PI);
    out->heading_deg = hdg < 0.0f ? hdg + 360.0f : hdg;
    out->h_ut        = sqrtf(L[0] * L[0] + L[1] * L[1]);
    out->dip_deg     = atan2f(L[2], out->h_ut) * (float)(180.0 / M_PI);
    out->total_ut    = sqrtf(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]);
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

/** Biezacy stan iteracji dopasowania. */
typedef struct {
    double o[3];           /**< hard iron */
    double si[2][2];       /**< W^-1: soft iron "w przod" (pole -> pomiar) */
    double v;              /**< pionowa skladowa pola Ziemi [uT], dodatnia */
} model_t;

/** Kierunek "w gore" w osiach plytki z przechylu (trzeci wiersz poziomowania). */
static void up_vector(float pitch_deg, float roll_deg, double u[3])
{
    double p = pitch_deg * DEG2RAD, r = roll_deg * DEG2RAD;
    u[0] = -sin(p);
    u[1] =  sin(r) * cos(p);
    u[2] =  cos(r) * cos(p);
}

/**
 * Probka przygotowana do dopasowania elipsy: pomiar minus hard iron minus
 * przeciek pionowej czesci pola Ziemi (-v * u, juz przez soft iron).
 * Zostaje soft iron * pozioma czesc pola Ziemi - punkty na elipsie.
 * q[2]: pomiar Z minus przeciek - srednio ~0, reszta to blad oz.
 */
static void sample(const flora_magcal_collector_t *c, int i, const model_t *md, double q[3])
{
    double u[3];
    up_vector(c->pitch[i], c->roll[i], u);
    double ex = -md->v * u[0], ey = -md->v * u[1], ez = -md->v * u[2];
    q[0] = c->x[i] - md->o[0] - (md->si[0][0] * ex + md->si[0][1] * ey);
    q[1] = c->y[i] - md->o[1] - (md->si[1][0] * ex + md->si[1][1] * ey);
    q[2] = c->z[i] - md->o[2] - ez;
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

/** Elipsa w poziomie dla danego o. */
typedef struct {
    double cx, cy;      /**< srodek (resztkowy hard iron w poziomie) */
    double w00, w01, w11;
    double R;           /**< promien okregu o polu elipsy */
    double axis_ratio;
    double mean_lz;
} ellipse_t;

static bool fit_ellipse(const flora_magcal_collector_t *c, const model_t *o, ellipse_t *e)
{
    /* Normalizacja: srednia 0, rozrzut ~1 - inaczej x^2 rzedu 2500 obok
     * wyrazow rzedu 1 i uklad jest zle uwarunkowany. */
    double mx = 0, my = 0, mz = 0, L[3];
    for (int i = 0; i < c->n; i++) {
        sample(c, i, o, L);
        mx += L[0]; my += L[1]; mz += L[2];
    }
    mx /= c->n; my /= c->n; mz /= c->n;
    double s = 0;
    for (int i = 0; i < c->n; i++) {
        sample(c, i, o, L);
        double dx = L[0] - mx, dy = L[1] - my;
        s += dx * dx + dy * dy;
    }
    s = sqrt(s / c->n);
    if (s < 1e-6) {
        return false;
    }

    /* Rownania normalne dla [A B C D E] przy prawej stronie 1. */
    double N[5][5] = {{0}}, r[5] = {0};
    for (int i = 0; i < c->n; i++) {
        sample(c, i, o, L);
        double u = (L[0] - mx) / s, v = (L[1] - my) / s;
        double row[5] = { u * u, u * v, v * v, u, v };
        for (int a = 0; a < 5; a++) {
            r[a] += row[a];
            for (int b = 0; b < 5; b++) N[a][b] += row[a] * row[b];
        }
    }
    double p[5];
    if (!solve5(N, r, p)) {
        return false;
    }
    double A = p[0], B = p[1], C = p[2], D = p[3], E = p[4];

    /* Elipsa wymaga dodatnio okreslonej czesci kwadratowej. */
    double det = 4 * A * C - B * B;
    if (A <= 0 || C <= 0 || det <= 0) {
        return false;
    }
    /* Srodek: gradient zero, [2A B; B 2C] c = -[D E]. */
    double u0 = (-2 * C * D + B * E) / det;
    double v0 = (-2 * A * E + B * D) / det;
    /* Po przesunieciu: x' M x' = k, k = 1 + c' M c. */
    double k = 1.0 + A * u0 * u0 + B * u0 * v0 + C * v0 * v0;
    if (k <= 0) {
        return false;
    }
    /* Q = M / (k s^2) w uT: elipsa x' Q x' = 1. */
    double q00 = A / (k * s * s), q01 = 0.5 * B / (k * s * s), q11 = C / (k * s * s);

    /* Wartosci wlasne symetrycznej 2x2. */
    double tr = q00 + q11, dd = sqrt(0.25 * (q00 - q11) * (q00 - q11) + q01 * q01);
    double l1 = 0.5 * tr + dd, l2 = 0.5 * tr - dd;   /* l1 >= l2 > 0 */
    if (l2 <= 0) {
        return false;
    }
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
    e->cx  = mx + s * u0;
    e->cy  = my + s * v0;
    e->w00 = s1 * ex * ex + s2 * ey * ey;
    e->w01 = (s1 - s2) * ex * ey;
    e->w11 = s1 * ey * ey + s2 * ex * ex;
    e->R   = R;
    e->axis_ratio = sqrt(l1 / l2);
    e->mean_lz = mz;
    return true;
}

flora_magcal_report_t flora_magcal_fit(const flora_magcal_collector_t *c, float dip_site_deg,
                                       flora_magcal_t *cal)
{
    flora_magcal_report_t rep = { .status = FLORA_MAGCAL_TOO_FEW, .n = (uint16_t)c->n };
    if (c->n < MIN_SAMPLES) {
        return rep;
    }

    /* Iteracja. Model pomiaru (wszystko w osiach plytki):
     *   m = o + S * (Ez_poziome + Ez_pionowe),  S = W^-1 tylko w x, y
     * Ez_pionowe = -v * u: kierunek z akcelerometru (u - "w gore"),
     * wielkosc v = R * tan(inklinacji miejsca). Po odjeciu o i przecieku
     * pionowego punkty leza na elipsie S * Ez_poziome:
     *   srodek elipsy  - poprawka ox, oy,
     *   ksztalt        - nowe W (i S do nastepnego kroku),
     *   srednia reszta w Z - poprawka oz (pozioma czesc pola ma w Z
     *                    tylko maly skladnik od przechylu, srednio ~0). */
    const double tan_dip = tan(dip_site_deg * DEG2RAD);
    model_t md = { .o = { 0, 0, 0 }, .si = { { 1, 0 }, { 0, 1 } }, .v = 0 };
    ellipse_t e;
    for (int it = 0; it < ITERATIONS; it++) {
        if (!fit_ellipse(c, &md, &e)) {
            rep.status = FLORA_MAGCAL_FIT;
            return rep;
        }
        md.o[0] += e.cx;
        md.o[1] += e.cy;
        md.o[2] += e.mean_lz;
        double det = e.w00 * e.w11 - e.w01 * e.w01;
        if (det <= 0) {
            rep.status = FLORA_MAGCAL_FIT;
            return rep;
        }
        md.si[0][0] =  e.w11 / det;
        md.si[0][1] = md.si[1][0] = -e.w01 / det;
        md.si[1][1] =  e.w00 / det;
        md.v = e.R * tan_dip;
    }
    if (!fit_ellipse(c, &md, &e) ||
        fabs(md.o[0]) > MAX_OFFSET_UT || fabs(md.o[1]) > MAX_OFFSET_UT ||
        fabs(md.o[2]) > MAX_OFFSET_UT) {
        rep.status = FLORA_MAGCAL_FIT;
        return rep;
    }
    rep.axis_ratio = (float)e.axis_ratio;

    flora_magcal_t out;
    /* Resztki po ostatniej iteracji sa rzedu ulamka uT - dokladamy je,
     * zeby wynik byl spojny z ostatnim dopasowaniem. */
    out.ox = (float)(md.o[0] + e.cx);
    out.oy = (float)(md.o[1] + e.cy);
    out.oz = (float)(md.o[2] + e.mean_lz);
    out.w[0][0] = (float)e.w00;
    out.w[0][1] = (float)e.w01;
    out.w[1][0] = (float)e.w01;
    out.w[1][1] = (float)e.w11;
    out.h_ref_ut = (float)e.R;
    out.dip_ref_deg = 0.0f;

    /* Rozrzut, pokrycie i inklinacja odniesienia - juz na pelnej korekcji. */
    bool seen[SECTORS] = { false };
    double e2 = 0, dip_sum = 0;
    for (int i = 0; i < c->n; i++) {
        float m[3] = { c->x[i], c->y[i], c->z[i] };
        flora_magcal_out_t r;
        flora_magcal_apply(&out, m, c->pitch[i], c->roll[i], &r);
        e2 += (r.h_ut - e.R) * (r.h_ut - e.R);
        dip_sum += r.dip_deg;
        int sec = (int)(r.heading_deg / 360.0f * SECTORS);
        if (sec >= SECTORS) sec = SECTORS - 1;
        if (sec < 0) sec = 0;
        seen[sec] = true;
    }
    out.dip_ref_deg = (float)(dip_sum / c->n);
    rep.rms_pct = (float)(100.0 * sqrt(e2 / c->n) / e.R);
    for (int i = 0; i < SECTORS; i++) rep.sectors += seen[i];

    if (rep.sectors < MIN_SECTORS)            rep.status = FLORA_MAGCAL_COVERAGE;
    else if (rep.rms_pct > MAX_RMS_PCT)       rep.status = FLORA_MAGCAL_RESIDUAL;
    else if (rep.axis_ratio > MAX_AXIS_RATIO) rep.status = FLORA_MAGCAL_AXIS_RATIO;
    else {
        rep.status = FLORA_MAGCAL_OK;
        *cal = out;
    }
    return rep;
}
