/**
 * @file flora_energy.c
 * @brief Polityka energii, opis w flora_energy.h.
 */
#include <math.h>
#include <string.h>

#include "flora_energy.h"

void flora_energy_init(flora_energy_t *e, const flora_energy_cfg_t *cfg)
{
    memset(e, 0, sizeof *e);
    e->cfg = *cfg;
    e->mode = FLORA_ENERGY_NORMAL;
    e->v_filt = NAN;
}

void flora_energy_set_cfg(flora_energy_t *e, const flora_energy_cfg_t *cfg)
{
    e->cfg = *cfg;
}

flora_energy_out_t flora_energy_update(flora_energy_t *e, const flora_energy_in_t *in)
{
    const flora_energy_cfg_t *c = &e->cfg;

    if (in->v_valid && isfinite(in->bus_v) && !in->load_high) {
        if (!e->have_v) {
            e->v_filt = in->bus_v;
            e->have_v = true;
        } else {
            float a = c->tau_s > 0.0f && in->dt_s > 0.0f ? in->dt_s / (c->tau_s + in->dt_s) : 1.0f;
            e->v_filt += a * (in->bus_v - e->v_filt);
        }
    }

    /* Bez pomiaru tryb zostaje: brak INA226 nie moze uspic robota na dobre
     * ani odblokowac jazdy na rozladowanej baterii. */
    if (e->have_v) {
        const float v = e->v_filt;
        switch (e->mode) {
        case FLORA_ENERGY_NORMAL:
            if (v < c->v_survival) {
                e->mode = FLORA_ENERGY_SURVIVAL;
            } else if (v < c->v_save) {
                e->mode = FLORA_ENERGY_SAVE;
            }
            break;
        case FLORA_ENERGY_SAVE:
            if (v < c->v_survival) {
                e->mode = FLORA_ENERGY_SURVIVAL;
            } else if (v > c->v_save + c->v_hyst) {
                e->mode = FLORA_ENERGY_NORMAL;
            }
            break;
        case FLORA_ENERGY_SURVIVAL:
            if (v > c->v_save + c->v_hyst) {
                e->mode = FLORA_ENERGY_NORMAL;
            } else if (v > c->v_survival + c->v_hyst) {
                e->mode = FLORA_ENERGY_SAVE;
            }
            break;
        }
    }

    flora_energy_out_t o = {
        .mode   = e->mode,
        .v_filt = e->have_v ? e->v_filt : NAN,
    };
    const bool night = in->night_known && in->night;
    switch (e->mode) {
    case FLORA_ENERGY_NORMAL:
        o.allow_drive = true;
        o.allow_water = true;
        o.mast_on = true;
        o.sleep_s = night ? c->sleep_night_s : c->sleep_day_s;
        break;
    case FLORA_ENERGY_SAVE:
        o.allow_drive = false;
        o.allow_water = true;
        o.mast_on = true;
        o.sleep_s = c->sleep_save_s;
        break;
    case FLORA_ENERGY_SURVIVAL:
        o.allow_drive = false;
        o.allow_water = false;
        o.mast_on = false;
        o.sleep_s = c->sleep_survival_s;
        break;
    }
    e->last = o;
    return o;
}
