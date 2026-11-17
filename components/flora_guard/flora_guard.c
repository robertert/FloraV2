/**
 * @file flora_guard.c
 * @brief Straznik napedu, opis w flora_guard.h.
 */
#include <math.h>
#include <string.h>

#include "flora_guard.h"

#define DEG2RAD 0.017453292f

enum { DIR_FWD, DIR_REV, DIR_TURN };

void flora_guard_init(flora_guard_t *g, const flora_guard_cfg_t *cfg)
{
    memset(g, 0, sizeof *g);
    g->cfg = *cfg;
    g->last.allow_fwd = g->last.allow_rev = g->last.allow_turn = true;
    g->last.autonomy_ok = true;
    g->last.power_limit = 1.0f;
    g->last.front_mm = g->last.rear_mm = -1.0f;
}

void flora_guard_set_cfg(flora_guard_t *g, const flora_guard_cfg_t *cfg)
{
    g->cfg = *cfg;
}

/**
 * Najblizsza przeszkoda z kamery w korytarzu jazdy.
 * @param[out] blind  true, gdy w stozku na wprost jest sektor "nie wiem"
 * @return odleglosc wzdluz osi jazdy [mm] albo -1, gdy korytarz czysty
 */
static float cam_front(const flora_guard_cfg_t *c, const uint8_t *sectors, bool *blind)
{
    const float step = c->fov_deg / FLORA_GUARD_SECTORS;
    float nearest = -1.0f;
    *blind = false;
    for (int i = 0; i < FLORA_GUARD_SECTORS; i++) {
        const float ang = (i + 0.5f) * step - 0.5f * c->fov_deg;   /* lewo ujemne */
        const uint8_t v = sectors[i];
        if (v == FLORA_GUARD_SECTOR_UNKNOWN) {
            if (fabsf(ang) <= c->unknown_cone_deg) {
                *blind = true;
            }
            continue;
        }
        if (v >= FLORA_GUARD_SECTOR_CLEAR) {
            continue;
        }
        const float r = v * 10.0f;
        const float along = r * cosf(ang * DEG2RAD);
        const float side  = fabsf(r * sinf(ang * DEG2RAD));
        if (side <= c->corridor_half_mm && (nearest < 0.0f || along < nearest)) {
            nearest = along;
        }
    }
    return nearest;
}

static float nearer(float a, float b)
{
    if (a < 0.0f) return b;
    if (b < 0.0f) return a;
    return a < b ? a : b;
}

/** Blokada z podtrzymaniem: surowa blokada teraz albo w ostatnich release_ms. */
static bool held(flora_guard_t *g, int dir, bool raw, uint32_t now)
{
    if (raw) {
        g->blk_seen[dir] = true;
        g->blk_ms[dir] = now;
        return true;
    }
    if (g->blk_seen[dir]) {
        if ((uint32_t)(now - g->blk_ms[dir]) < g->cfg.release_ms) {
            return true;
        }
        g->blk_seen[dir] = false;
    }
    return false;
}

flora_guard_out_t flora_guard_update(flora_guard_t *g, const flora_guard_in_t *in)
{
    const flora_guard_cfg_t *c = &g->cfg;
    flora_guard_out_t o = {
        .autonomy_ok = true,
        .power_limit = 1.0f,
        .front_mm    = -1.0f,
        .rear_mm     = -1.0f,
    };
    bool blk_fwd = false, blk_rev = false, blk_all = false;

    if (c->mask & FLORA_GUARD_CAM) {
        const bool fresh = in->obs_valid && in->obs_age_ms <= c->obs_max_age_ms;
        bool blind = true;
        float d = -1.0f;
        if (fresh) {
            d = cam_front(c, in->sectors, &blind);
        }
        o.front_mm = nearer(o.front_mm, d);
        if (blind || (d >= 0.0f && d < c->stop_front_mm)) {
            blk_fwd = true;
            o.reasons |= FLORA_GUARD_CAM;
        }
    }

    if (c->mask & FLORA_GUARD_SONAR) {
        float f = -1.0f;
        const float fl = in->sonar_mm[FLORA_GUARD_SONAR_FRONT_L];
        const float fr = in->sonar_mm[FLORA_GUARD_SONAR_FRONT_R];
        const float re = in->sonar_mm[FLORA_GUARD_SONAR_REAR];
        if (!isnan(fl)) f = nearer(f, fl);
        if (!isnan(fr)) f = nearer(f, fr);
        o.front_mm = nearer(o.front_mm, f);
        if (f >= 0.0f && f < c->stop_front_mm) {
            blk_fwd = true;
            o.reasons |= FLORA_GUARD_SONAR;
        }
        if (!isnan(re)) {
            o.rear_mm = re;
            if (re < c->stop_rear_mm) {
                blk_rev = true;
                o.reasons |= FLORA_GUARD_SONAR;
            }
        }
    }

    const bool att_fresh = in->att_valid && in->att_age_ms <= c->att_max_age_ms;

    if (c->mask & FLORA_GUARD_TILT) {
        /* Bez swiezego ATTITUDE przechylu nie znamy - stan zostaje, a brak
         * danych zglasza STALE. Zdjecie blokady tylko na swiezym pomiarze. */
        if (att_fresh) {
            const float t = fmaxf(fabsf(in->pitch_deg), fabsf(in->roll_deg));
            if (t > c->tilt_stop_deg) {
                g->tilt_active = true;
            } else if (t < c->tilt_clear_deg) {
                g->tilt_active = false;
            }
        }
        if (g->tilt_active) {
            blk_all = true;
            o.autonomy_ok = false;
            o.reasons |= FLORA_GUARD_TILT;
        }
    } else {
        g->tilt_active = false;
    }

    if (c->mask & FLORA_GUARD_VOLTAGE) {
        if (in->bus_v_valid) {
            if (in->bus_v < c->v_stop) {
                if (!g->v_low) {
                    g->v_low = true;
                    g->v_low_since_ms = in->now_ms;
                }
                if ((uint32_t)(in->now_ms - g->v_low_since_ms) >= c->v_hold_ms) {
                    g->v_active = true;
                }
            } else {
                g->v_low = false;
                if (in->bus_v > c->v_clear) {
                    g->v_active = false;
                }
            }
        }
        if (g->v_active) {
            blk_all = true;
            o.autonomy_ok = false;
            o.reasons |= FLORA_GUARD_VOLTAGE;
        }
    } else {
        g->v_active = false;
        g->v_low = false;
    }

    if ((c->mask & FLORA_GUARD_STALE) && !att_fresh) {
        o.autonomy_ok = false;
        o.power_limit = c->crawl_limit;
        o.reasons |= FLORA_GUARD_STALE;
    }

    o.allow_fwd  = !held(g, DIR_FWD,  blk_fwd || blk_all, in->now_ms);
    o.allow_rev  = !held(g, DIR_REV,  blk_rev || blk_all, in->now_ms);
    o.allow_turn = !held(g, DIR_TURN, blk_all, in->now_ms);

    g->last = o;
    return o;
}

float flora_guard_permit(const flora_guard_out_t *out, float left, float right)
{
    if (left == 0.0f && right == 0.0f) {
        return 1.0f;
    }
    const float v = 0.5f * (left + right);    /* postep */
    const float w = 0.5f * (right - left);    /* obrot */
    bool ok;
    /* Obrot w miejscu: postep znikomy wobec roznicy kol. Luk z wyraznym
     * postepem to jazda - liczy sie kierunek postepu. */
    if (fabsf(v) <= 0.2f * fabsf(w)) {
        ok = out->allow_turn;
    } else if (v > 0.0f) {
        ok = out->allow_fwd;
    } else {
        ok = out->allow_rev;
    }
    if (!ok) {
        return 0.0f;
    }
    float k = out->power_limit;
    if (k > 1.0f) k = 1.0f;
    if (k < 0.0f) k = 0.0f;
    return k;
}
