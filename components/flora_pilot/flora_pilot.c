/**
 * @file flora_pilot.c
 * @brief Jazda do punktu, opis w flora_pilot.h.
 */
#include <math.h>
#include <string.h>

#include "flora_pilot.h"

#define DEG2RAD 0.017453292f
#define RAD2DEG 57.29578f

/** Najkrotsza roznica katow a - b, -180..180. */
static float diff180(float a, float b)
{
    float d = fmodf(a - b + 540.0f, 360.0f);
    if (d < 0.0f) d += 360.0f;
    return d - 180.0f;
}

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

void flora_pilot_init(flora_pilot_t *p, const flora_pilot_cfg_t *cfg)
{
    memset(p, 0, sizeof *p);
    p->cfg = *cfg;
    p->state = FLORA_PILOT_IDLE;
}

void flora_pilot_set_cfg(flora_pilot_t *p, const flora_pilot_cfg_t *cfg)
{
    p->cfg = *cfg;
}

static void begin(flora_pilot_t *p, flora_pilot_state_t st)
{
    p->state = st;
    p->fail = FLORA_PILOT_FAIL_NONE;
    p->t = 0.0f;
    p->t_progress = 0.0f;
    p->best = -1.0f;      /* pierwsza miara ustawi */
}

void flora_pilot_goto(flora_pilot_t *p, float x_mm, float y_mm)
{
    p->gx = x_mm;
    p->gy = y_mm;
    p->has_point = true;
    p->has_final = false;
    begin(p, FLORA_PILOT_TURN);
}

void flora_pilot_goto_pose(flora_pilot_t *p, float x_mm, float y_mm, float th_deg)
{
    flora_pilot_goto(p, x_mm, y_mm);
    p->gth = th_deg;
    p->has_final = true;
}

void flora_pilot_turn(flora_pilot_t *p, float th_deg)
{
    p->gth = th_deg;
    p->has_point = false;
    p->has_final = true;
    begin(p, FLORA_PILOT_FINAL);
}

void flora_pilot_cancel(flora_pilot_t *p)
{
    if (flora_pilot_busy(p)) {
        p->state = FLORA_PILOT_FAILED;
        p->fail = FLORA_PILOT_FAIL_CANCELLED;
    }
}

bool flora_pilot_busy(const flora_pilot_t *p)
{
    return p->state == FLORA_PILOT_TURN || p->state == FLORA_PILOT_DRIVE ||
           p->state == FLORA_PILOT_FINAL;
}

/** Predkosc obrotu w miejscu dla bledu kursu, st/s ze znakiem. */
static float turn_rate(const flora_pilot_cfg_t *c, float err_deg)
{
    float w = c->k_heading * err_deg;
    const float a = fabsf(w);
    if (a > c->w_max_dps) {
        w = w > 0.0f ? c->w_max_dps : -c->w_max_dps;
    } else if (a < c->w_min_dps) {
        w = err_deg >= 0.0f ? c->w_min_dps : -c->w_min_dps;
    }
    return w;
}

/** Postep liczony jako spadek miary (odleglosc w mm albo blad w st x10). */
static void note_progress(flora_pilot_t *p, float measure)
{
    if (p->best < 0.0f || measure < p->best - p->cfg.progress_mm) {
        p->best = measure;
        p->t_progress = p->t;
    }
}

/** Nowy etap: licznik postepu od nowa, bo zmienia sie miara. */
static void next_stage(flora_pilot_t *p, flora_pilot_state_t st)
{
    p->state = st;
    p->best = -1.0f;
    p->t_progress = p->t;
}

flora_pilot_out_t flora_pilot_update(flora_pilot_t *p, float x_mm, float y_mm,
                                     float th_deg, float dt_s)
{
    const flora_pilot_cfg_t *c = &p->cfg;
    flora_pilot_out_t o = { .state = p->state, .fail = p->fail };

    if (!flora_pilot_busy(p)) {
        p->last = o;
        return o;
    }
    if (dt_s > 0.0f) {
        p->t += dt_s;
    }

    float v = 0.0f, w_dps = 0.0f;
    const float dx = p->gx - x_mm, dy = p->gy - y_mm;
    const float dist = p->has_point ? sqrtf(dx * dx + dy * dy) : 0.0f;

    if (p->state == FLORA_PILOT_TURN || p->state == FLORA_PILOT_DRIVE) {
        o.dist_mm = dist;
        if (dist <= c->pos_tol_mm) {
            if (p->has_final) {
                next_stage(p, FLORA_PILOT_FINAL);
            } else {
                p->state = FLORA_PILOT_DONE;
            }
        } else {
            const float err = diff180(atan2f(dy, dx) * RAD2DEG, th_deg);
            o.err_deg = err;
            if (p->state == FLORA_PILOT_TURN) {
                if (fabsf(err) <= c->turn_out_deg) {
                    next_stage(p, FLORA_PILOT_DRIVE);
                } else {
                    w_dps = turn_rate(c, err);
                    note_progress(p, fabsf(err) * 10.0f);
                }
            }
            if (p->state == FLORA_PILOT_DRIVE) {
                if (fabsf(err) > c->turn_in_deg) {
                    next_stage(p, FLORA_PILOT_TURN);
                    w_dps = turn_rate(c, err);
                } else {
                    v = c->v_max;
                    if (dist < c->slow_radius_mm && c->slow_radius_mm > 0.0f) {
                        v = c->v_min + (c->v_max - c->v_min) * dist / c->slow_radius_mm;
                    }
                    w_dps = clampf(c->k_heading * err, -c->w_max_dps, c->w_max_dps);
                    note_progress(p, dist);
                }
            }
        }
    }

    if (p->state == FLORA_PILOT_FINAL) {
        const float err = diff180(p->gth, th_deg);
        o.err_deg = err;
        o.dist_mm = dist;
        if (fabsf(err) <= c->th_tol_deg) {
            p->state = FLORA_PILOT_DONE;
        } else {
            v = 0.0f;
            w_dps = turn_rate(c, err);
            note_progress(p, fabsf(err) * 10.0f);
        }
    }

    if (flora_pilot_busy(p)) {
        if (p->t - p->t_progress > c->no_progress_s) {
            p->state = FLORA_PILOT_FAILED;
            p->fail = FLORA_PILOT_FAIL_NO_PROGRESS;
        } else if (c->timeout_s > 0.0f && p->t > c->timeout_s) {
            p->state = FLORA_PILOT_FAILED;
            p->fail = FLORA_PILOT_FAIL_TIMEOUT;
        }
    }

    if (flora_pilot_busy(p)) {
        /* w dodatnie = CCW = prawe kolo szybciej. */
        const float dv = w_dps * DEG2RAD * 0.5f * c->track_mm;
        o.v_l = v - dv;
        o.v_r = v + dv;
    }
    o.state = p->state;
    o.fail = p->fail;
    p->last = o;
    return o;
}
