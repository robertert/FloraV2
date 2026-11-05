#include <math.h>
#include <string.h>

#include "flora_magdist.h"

void flora_magdist_init(flora_magdist_t *d, const flora_magdist_cfg_t *cfg)
{
    memset(d, 0, sizeof(*d));
    d->cfg = *cfg;
}

void flora_magdist_set_ref(flora_magdist_t *d, bool have, float h_ref_ut, float dip_ref_deg)
{
    d->have_ref = have && h_ref_ut > 0.0f;
    d->h_ref    = h_ref_ut;
    d->dip_ref  = dip_ref_deg;
}

bool flora_magdist_step(flora_magdist_t *d, uint32_t dt_ms, float h_ut, float dip_deg,
                        float total_ut, bool gyro_agrees)
{
    uint8_t cause = FLORA_MAGDIST_CAUSE_NONE;

    if (gyro_agrees) {
        d->reject_ms_run = 0;
    } else {
        d->reject_ms_run += dt_ms;
    }

    if (d->have_ref) {
        if (fabsf(h_ut / d->h_ref - 1.0f) > d->cfg.h_tol) {
            cause = FLORA_MAGDIST_CAUSE_FIELD;
        } else if (fabsf(dip_deg - d->dip_ref) > d->cfg.dip_tol_deg) {
            cause = FLORA_MAGDIST_CAUSE_DIP;
        }
    } else if (total_ut < 15.0f || total_ut > 90.0f) {
        /* Szerzej niz pole ziemskie (25-65 uT), bo bez kalibracji robot
         * sam przesuwa wartosc - ale magnes albo przewod pod pradem
         * wypada daleko poza ten zakres. */
        cause = FLORA_MAGDIST_CAUSE_RANGE;
    }
    if (cause == FLORA_MAGDIST_CAUSE_NONE && d->reject_ms_run > d->cfg.reject_ms) {
        cause = FLORA_MAGDIST_CAUSE_GYRO;
    }

    if (cause != FLORA_MAGDIST_CAUSE_NONE) {
        d->disturbed = true;
        d->cause     = cause;
        d->good_ms   = 0;
    } else if (d->disturbed) {
        d->good_ms += dt_ms;
        if (d->good_ms >= d->cfg.clear_ms) {
            d->disturbed = false;
            d->cause     = FLORA_MAGDIST_CAUSE_NONE;
        }
    }
    return d->disturbed;
}
