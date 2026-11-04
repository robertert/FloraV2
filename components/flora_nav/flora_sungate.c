#include <math.h>
#include <string.h>

#include "flora_ekf.h"
#include "flora_sun.h"
#include "flora_sungate.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define D2R(d) ((d) * (float)M_PI / 180.0f)
#define R2D(r) ((r) * 180.0f / (float)M_PI)

void flora_sungate_init(flora_sungate_t *g, const flora_sungate_cfg_t *cfg)
{
    memset(g, 0, sizeof(*g));
    g->cfg = *cfg;
}

void flora_sungate_reset(flora_sungate_t *g)
{
    g->n = 0;
}

const char *flora_sungate_name(flora_sungate_result_t r)
{
    switch (r) {
        case FLORA_SUNGATE_OK:           return "ok";
        case FLORA_SUNGATE_DIM:          return "za malo swiatla (nie slonce?)";
        case FLORA_SUNGATE_LOW:          return "brak czasu UTC albo slonce nisko";
        case FLORA_SUNGATE_UNSTABLE:     return "kolejne pomiary niezgodne";
        case FLORA_SUNGATE_MAG_CONFLICT: return "niezgodny z magnetometrem";
        default:                         return "?";
    }
}

flora_sungate_result_t flora_sungate_eval(flora_sungate_t *g, const flora_sungate_in_t *in,
                                          flora_sungate_out_t *out)
{
    const flora_sungate_cfg_t *c = &g->cfg;
    memset(out, 0, sizeof(*out));

    /* 1. Swiatlo. */
    float lux_max = 0.0f;
    for (int i = 0; i < 4; i++) {
        if (in->lux[i] > lux_max) lux_max = in->lux[i];
    }
    if (lux_max < c->min_lux) {
        g->n = 0;   /* po lampie potrzeba od nowa kilku zgodnych pomiarow */
        return FLORA_SUNGATE_DIM;
    }

    /* 2. Niebo. */
    if (in->unix_s <= 0) {
        g->n = 0;
        return FLORA_SUNGATE_LOW;
    }
    flora_sun_pos_t p = flora_sun_position(in->unix_s, c->lat_deg, c->lon_deg);
    if (p.elevation_deg < c->min_elev_deg) {
        g->n = 0;
        return FLORA_SUNGATE_LOW;
    }

    /* Krzyz mierzy azymut slonca wzgledem glowicy (CW), NOAA daje azymut
     * wzgledem polnocy geograficznej:
     *   kurs_geo = A_noaa - a_zmierzony,  kurs_mag = kurs_geo - deklinacja */
    float psi = flora_ekf_wrap(D2R((float)p.azimuth_deg - in->azimuth_meas_deg - c->declination_deg));

    /* 3. Stalosc: psi + theta = kurs osi mapy, niezalezny od obrotu robota. */
    float b_impl = flora_ekf_wrap(psi + in->th_ref_rad);
    if (g->n < FLORA_SUNGATE_STABLE_N) {
        g->b[g->n++] = b_impl;
    } else {
        memmove(&g->b[0], &g->b[1], (FLORA_SUNGATE_STABLE_N - 1) * sizeof(float));
        g->b[FLORA_SUNGATE_STABLE_N - 1] = b_impl;
    }
    if (g->n < FLORA_SUNGATE_STABLE_N) {
        return FLORA_SUNGATE_UNSTABLE;
    }
    for (int i = 0; i < FLORA_SUNGATE_STABLE_N - 1; i++) {
        if (fabsf(flora_ekf_wrap(g->b[i] - b_impl)) > D2R(c->stable_deg)) {
            return FLORA_SUNGATE_UNSTABLE;
        }
    }

    /* 4. Zgodnosc z magnetometrem - on ma pierwszenstwo. */
    float sigma = D2R(c->sigma_nomag_deg);
    if (in->head_mag_ok) {
        float diff = flora_ekf_wrap(psi - in->head_psi_rad);
        out->vs_mag_deg = R2D(diff);
        if (fabsf(diff) > D2R(c->mag_max_deg)) {
            return FLORA_SUNGATE_MAG_CONFLICT;
        }
        sigma = D2R(c->sigma_deg);
    }

    out->psi_rad   = psi;
    out->sigma_rad = sigma;
    return FLORA_SUNGATE_OK;
}
