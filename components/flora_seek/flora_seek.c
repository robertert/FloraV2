/**
 * @file flora_seek.c
 * @brief Ucieczka z cienia, opis w flora_seek.h.
 */
#include <math.h>
#include <string.h>

#include "flora_seek.h"

#define DEG2RAD 0.017453292f

float flora_seek_contrast(const float lux[4])
{
    float lo = lux[0], hi = lux[0];
    for (int i = 1; i < 4; i++) {
        if (lux[i] < lo) lo = lux[i];
        if (lux[i] > hi) hi = lux[i];
    }
    const float sum = hi + lo;
    return sum > 0.0f ? (hi - lo) / sum : 0.0f;
}

void flora_seek_init(flora_seek_t *s, const flora_seek_cfg_t *cfg)
{
    memset(s, 0, sizeof *s);
    s->cfg = *cfg;
    s->state = FLORA_SEEK_WATCH;
}

void flora_seek_set_cfg(flora_seek_t *s, const flora_seek_cfg_t *cfg)
{
    s->cfg = *cfg;
}

void flora_seek_restart(flora_seek_t *s, uint32_t now_s)
{
    s->state = FLORA_SEEK_WATCH;
    s->dwell_since_s = now_s;
    s->in_shade = false;
}

void flora_seek_force(flora_seek_t *s)
{
    if (s->state == FLORA_SEEK_WATCH) {
        s->force = true;
    }
}

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

/** Granice obszaru pomniejszone o margines; false, gdy nic nie zostaje. */
static bool inner_box(const flora_seek_cfg_t *c, float *x0, float *y0, float *x1, float *y1)
{
    if (!c->area_valid) {
        return false;
    }
    *x0 = c->xmin + c->margin_mm;
    *x1 = c->xmax - c->margin_mm;
    *y0 = c->ymin + c->margin_mm;
    *y1 = c->ymax - c->margin_mm;
    return *x1 > *x0 && *y1 > *y0;
}

/**
 * Punkt probny numer i: na promieniu ze srodka (ox, oy), przyciety do
 * obszaru. false, gdy po przycieciu zostaje mniej niz min_len_mm.
 */
static bool probe_point(const flora_seek_t *s, int i, float *px, float *py)
{
    const flora_seek_cfg_t *c = &s->cfg;
    float x0, y0, x1, y1;
    if (!inner_box(c, &x0, &y0, &x1, &y1)) {
        return false;
    }
    const float ang = (s->base_deg + (i - 0.5f * (c->max_probes - 1)) * c->step_deg) * DEG2RAD;
    const float dx = cosf(ang), dy = sinf(ang);

    /* Najwieksze t w [0, len], dla ktorego punkt zostaje w obszarze. */
    float t = c->probe_len_mm;
    if (dx > 1e-6f)       t = fminf(t, (x1 - s->ox) / dx);
    else if (dx < -1e-6f) t = fminf(t, (x0 - s->ox) / dx);
    if (dy > 1e-6f)       t = fminf(t, (y1 - s->oy) / dy);
    else if (dy < -1e-6f) t = fminf(t, (y0 - s->oy) / dy);
    if (t < c->min_len_mm) {
        return false;
    }
    *px = s->ox + t * dx;
    *py = s->oy + t * dy;
    return true;
}

/** Wybiera kolejny osiagalny punkt; false, gdy lista sie skonczyla. */
static bool next_probe(flora_seek_t *s, flora_seek_out_t *o)
{
    while (s->next < s->cfg.max_probes && s->next < FLORA_SEEK_MAX_PROBES) {
        const int i = s->next++;
        float px, py;
        if (probe_point(s, i, &px, &py)) {
            s->tx = px;
            s->ty = py;
            s->probe++;
            s->state = FLORA_SEEK_MOVING;
            o->cmd = FLORA_SEEK_CMD_GOTO;
            o->gx = px;
            o->gy = py;
            return true;
        }
    }
    return false;
}

static void finish(flora_seek_t *s, flora_seek_out_t *o, flora_seek_result_t res, uint32_t now)
{
    const flora_seek_cfg_t *c = &s->cfg;
    s->state = FLORA_SEEK_WATCH;
    s->dwell_since_s = now;
    s->in_shade = false;
    o->result = res;
    if (res == FLORA_SEEK_RES_IMPROVED) {
        s->backoff_cur_s = 0;
        s->backoff_on = false;
    } else if (res == FLORA_SEEK_RES_NO_GAIN || res == FLORA_SEEK_RES_BLOCKED) {
        s->backoff_cur_s = s->backoff_cur_s ? s->backoff_cur_s * 2 : c->backoff_s;
        if (s->backoff_cur_s > c->backoff_max_s) {
            s->backoff_cur_s = c->backoff_max_s;
        }
        s->backoff_until_s = now + s->backoff_cur_s;
        s->backoff_on = true;
        /* Niedawne maksimum juz nie obowiazuje - tu po prostu jest ciemniej. */
        if (s->have_lux) {
            s->ref = s->lux_f;
        }
    }
}

/** Koniec listy punktow: zostac, wrocic do najlepszego albo uznac porazke. */
static void wrap_up(flora_seek_t *s, flora_seek_out_t *o, uint32_t now)
{
    const bool gained = s->blux >= s->cfg.gain_ratio * s->lux0;
    if (s->moved_ok == 0) {
        /* Zaden przejazd nie doszedl: robot stoi (prawie) na starcie. */
        finish(s, o, s->moved_fail ? FLORA_SEEK_RES_BLOCKED : FLORA_SEEK_RES_NO_GAIN, now);
        return;
    }
    if (s->best_is_here) {
        finish(s, o, gained ? FLORA_SEEK_RES_IMPROVED : FLORA_SEEK_RES_NO_GAIN, now);
        return;
    }
    s->tx = s->bx;
    s->ty = s->by;
    s->state = FLORA_SEEK_RETURN;
    o->cmd = FLORA_SEEK_CMD_GOTO;
    o->gx = s->bx;
    o->gy = s->by;
}

flora_seek_out_t flora_seek_update(flora_seek_t *s, const flora_seek_in_t *in)
{
    const flora_seek_cfg_t *c = &s->cfg;
    const uint32_t now = in->now_s;
    flora_seek_out_t o = { .cmd = FLORA_SEEK_CMD_NONE, .result = FLORA_SEEK_RES_NONE };

    if (!s->started) {
        s->started = true;
        s->last_s = now;
        s->dwell_since_s = now;
    }
    const float dt = (float)(uint32_t)(now - s->last_s);
    s->last_s = now;

    /* --- filtry jasnosci --- */
    if (in->lux_valid && isfinite(in->lux)) {
        if (!s->have_lux) {
            s->lux_f = in->lux;
            s->con_f = in->contrast;
            s->ref = in->lux;
            s->have_lux = true;
        } else {
            const float a = c->lux_tau_s > 0.0f ? dt / (c->lux_tau_s + dt) : 1.0f;
            s->lux_f += a * (in->lux - s->lux_f);
            s->con_f += a * (in->contrast - s->con_f);
        }
    }
    if (s->have_lux) {
        if (c->ref_tau_s > 0.0f && dt > 0.0f) {
            s->ref *= expf(-dt / c->ref_tau_s);
        }
        if (s->lux_f > s->ref) {
            s->ref = s->lux_f;
        }
    }

    const bool day = in->elev_known ? in->sun_elev_deg > c->min_elev_deg
                                    : (s->have_lux && s->lux_f > c->min_lux);
    const bool direct = s->have_lux && s->con_f >= c->contrast_min;
    const bool shade = s->have_lux && day &&
                       (!direct || s->lux_f < c->shade_ratio * s->ref);

    switch (s->state) {
    case FLORA_SEEK_WATCH: {
        if (shade) {
            if (!s->in_shade) {
                s->in_shade = true;
                s->shade_since_s = now;
            }
        } else {
            s->in_shade = false;
        }
        if (s->backoff_on && (int32_t)(now - s->backoff_until_s) >= 0) {
            s->backoff_on = false;
        }
        const bool due = s->in_shade &&
                         (uint32_t)(now - s->shade_since_s) >= c->shade_hold_s &&
                         (uint32_t)(now - s->dwell_since_s) >= c->min_dwell_s &&
                         !s->backoff_on;
        const bool ready = (due || s->force) && s->have_lux && !in->hold;
        float x0, y0, x1, y1;
        if (ready && inner_box(c, &x0, &y0, &x1, &y1)) {
            s->force = false;
            /* Start spoza obszaru: luk wokol najblizszego punktu w srodku. */
            s->ox = clampf(in->x_mm, x0, x1);
            s->oy = clampf(in->y_mm, y0, y1);
            s->lux0 = s->lux_f;
            s->base_deg = in->sun_dir_valid ? in->sun_map_deg + 180.0f : 0.0f;
            s->next = 0;
            s->probe = 0;
            s->moved_ok = s->moved_fail = 0;
            /* Najlepszy na razie jest start - ale tylko w obszarze: spoza
             * niego wraca sie do najblizszego punktu w srodku, nie na zewnatrz. */
            s->bx = s->ox;
            s->by = s->oy;
            s->blux = s->lux0;
            s->best_is_here = s->ox == in->x_mm && s->oy == in->y_mm;
            if (!next_probe(s, &o)) {
                /* Obszar za maly na jakikolwiek odcinek. */
                finish(s, &o, FLORA_SEEK_RES_NO_GAIN, now);
            }
        }
        break;
    }

    case FLORA_SEEK_MOVING:
    case FLORA_SEEK_RETURN:
        if (in->hold) {
            if (in->nav == FLORA_SEEK_NAV_BUSY) {
                o.cmd = FLORA_SEEK_CMD_STOP;
            }
            finish(s, &o, FLORA_SEEK_RES_ABORTED, now);
            break;
        }
        if (in->nav == FLORA_SEEK_NAV_BUSY) {
            break;
        }
        if (s->state == FLORA_SEEK_RETURN) {
            /* Dojechal albo nie - szukanie skonczone, ocena wg tego, co bylo. */
            finish(s, &o, in->nav == FLORA_SEEK_NAV_ARRIVED &&
                          s->blux >= c->gain_ratio * s->lux0
                          ? FLORA_SEEK_RES_IMPROVED : FLORA_SEEK_RES_NO_GAIN, now);
            break;
        }
        if (in->nav == FLORA_SEEK_NAV_ARRIVED) {
            s->moved_ok++;
            s->best_is_here = false;
            s->state = FLORA_SEEK_MEASURE;
            s->meas_since_s = now;
            s->meas_sum = s->meas_con = 0.0f;
            s->meas_n = 0;
        } else {
            /* Przeszkoda albo brak postepu: ten punkt przepada. Robot mogl
             * odjechac kawalek, wiec najlepszy punkt nie jest juz "tutaj". */
            s->moved_fail++;
            s->best_is_here = false;
            if (!next_probe(s, &o)) {
                wrap_up(s, &o, now);
            }
        }
        break;

    case FLORA_SEEK_MEASURE:
        if (in->hold) {
            finish(s, &o, FLORA_SEEK_RES_ABORTED, now);
            break;
        }
        if (in->lux_valid && isfinite(in->lux)) {
            s->meas_sum += in->lux;
            s->meas_con += in->contrast;
            s->meas_n++;
        }
        if ((uint32_t)(now - s->meas_since_s) < c->settle_s) {
            break;
        }
        if (s->meas_n > 0) {
            const float avg = s->meas_sum / (float)s->meas_n;
            const float con = s->meas_con / (float)s->meas_n;
            /* Filtr wolny nie nadazyl za przejazdem - podstawiamy pomiar. */
            s->lux_f = avg;
            s->con_f = con;
            if (avg > s->blux) {
                s->blux = avg;
                s->bx = in->x_mm;
                s->by = in->y_mm;
                s->best_is_here = true;
            } else {
                s->best_is_here = false;
            }
            if (s->best_is_here && con >= c->contrast_min && avg >= c->gain_ratio * s->lux0) {
                finish(s, &o, FLORA_SEEK_RES_IMPROVED, now);
                break;
            }
        }
        if (!next_probe(s, &o)) {
            wrap_up(s, &o, now);
        }
        break;
    }

    o.state = s->state;
    o.shade = shade;
    o.day = day;
    o.lux_filt = s->have_lux ? s->lux_f : NAN;
    o.lux_ref = s->have_lux ? s->ref : NAN;
    o.shade_s = s->in_shade ? (uint32_t)(now - s->shade_since_s) : 0;
    o.probe = s->probe;
    if (s->state == FLORA_SEEK_WATCH) {
        uint32_t w = 0;
        const uint32_t dw = (uint32_t)(now - s->dwell_since_s);
        if (dw < c->min_dwell_s) {
            w = c->min_dwell_s - dw;
        }
        if (s->backoff_on) {
            const uint32_t b = (uint32_t)(s->backoff_until_s - now);
            if (b > w) w = b;
        }
        o.wait_s = w;
    }
    s->last = o;
    return o;
}
