/**
 * @file flora_water.c
 * @brief Polityka podlewania, opis w flora_water.h.
 */
#include <math.h>
#include <string.h>

#include "flora_water.h"

#define DAY_S 86400u

void flora_water_init(flora_water_t *w, const flora_water_cfg_t *cfg)
{
    memset(w, 0, sizeof *w);
    w->cfg = *cfg;
    w->state = FLORA_WATER_NO_SENSOR;
    w->median = NAN;
}

void flora_water_set_cfg(flora_water_t *w, const flora_water_cfg_t *cfg)
{
    w->cfg = *cfg;
}

void flora_water_reset_fault(flora_water_t *w)
{
    if (w->state == FLORA_WATER_FAULT) {
        w->state = FLORA_WATER_OK;
        w->fault = FLORA_WATER_FAULT_NONE;
        w->no_rise = 0;
        w->remaining_ml = 0.0f;
    }
}

static void push_sample(flora_water_t *w, float pct)
{
    w->win[w->win_head] = pct;
    w->win_head = (w->win_head + 1) % FLORA_WATER_MEDIAN_N;
    if (w->win_n < FLORA_WATER_MEDIAN_N) {
        w->win_n++;
    }
    if (w->win_n < FLORA_WATER_MEDIAN_N) {
        return;
    }
    /* Okno jest male - sortowanie przez wstawianie na kopii. */
    float s[FLORA_WATER_MEDIAN_N];
    memcpy(s, w->win, sizeof s);
    for (int i = 1; i < FLORA_WATER_MEDIAN_N; i++) {
        float v = s[i];
        int k = i - 1;
        while (k >= 0 && s[k] > v) {
            s[k + 1] = s[k];
            k--;
        }
        s[k + 1] = v;
    }
    w->median = s[FLORA_WATER_MEDIAN_N / 2];
    w->have_median = true;
}

static void enter(flora_water_t *w, flora_water_state_t st, uint32_t now_s)
{
    w->state = st;
    w->t_state_s = now_s;
}

static void fault(flora_water_t *w, flora_water_fault_t f)
{
    w->state = FLORA_WATER_FAULT;
    w->fault = f;
    w->remaining_ml = 0.0f;
}

flora_water_out_t flora_water_update(flora_water_t *w, const flora_water_in_t *in)
{
    const flora_water_cfg_t *c = &w->cfg;
    const uint32_t now = in->now_s;
    float request = 0.0f;

    if (in->soil_valid && isfinite(in->soil_pct)) {
        push_sample(w, in->soil_pct);
        w->last_valid_s = now;
        w->seen_valid = true;
    }
    const bool sensor_ok = w->have_median && w->seen_valid &&
                           (uint32_t)(now - w->last_valid_s) <= c->sensor_timeout_s;

    /* Okno dobowe otwiera pierwsza dawka i zamyka sie po 24 h. */
    if (w->day_open && (uint32_t)(now - w->day_start_s) >= DAY_S) {
        w->day_open = false;
        w->day_ml = 0.0f;
    }

    /* To, co poszlo, liczymy zawsze - takze gdy stan sie w miedzyczasie
     * zmienil (np. zabraklo wody w polowie porcji). */
    if (in->delivered_ml > 0.0f) {
        if (!w->day_open) {
            w->day_open = true;
            w->day_start_s = now;
        }
        w->day_ml += in->delivered_ml;
        w->remaining_ml -= in->delivered_ml;
        if (w->state == FLORA_WATER_DOSING) {
            w->t_state_s = now;      /* pompka podaje - limit czasu od nowa */
        }
    }

    if (sensor_ok) {
        if (w->median < c->dry_pct) {
            w->need = true;
        } else if (w->median > c->wet_pct) {
            w->need = false;
        }
    }

    switch (w->state) {
    case FLORA_WATER_FAULT:
        break;

    case FLORA_WATER_DOSING:
        if (in->pump_busy) {
            if ((uint32_t)(now - w->t_state_s) > c->pump_timeout_s) {
                fault(w, FLORA_WATER_FAULT_PUMP);
            }
            break;
        }
        if (w->remaining_ml <= 0.5f) {
            w->remaining_ml = 0.0f;
            enter(w, FLORA_WATER_SOAK, now);
            break;
        }
        if (!in->water_present) {
            /* Reszta dawki przepada: po dolaniu wody decyzja zapada od nowa. */
            w->remaining_ml = 0.0f;
            enter(w, FLORA_WATER_NO_WATER, now);
            break;
        }
        if (!in->allowed) {
            /* Robot ruszyl w polowie dawki - czekamy, to nie wina pompki. */
            w->t_state_s = now;
            break;
        }
        if ((uint32_t)(now - w->t_state_s) > c->pump_timeout_s) {
            /* Pompka nie rusza mimo zadan (odmowa albo brak kalibracji). */
            fault(w, FLORA_WATER_FAULT_PUMP);
            break;
        }
        request = w->remaining_ml;
        break;

    case FLORA_WATER_SOAK:
        if ((uint32_t)(now - w->t_state_s) < c->soak_s) {
            break;
        }
        /* Po wsiakaniu: czy dawka cokolwiek zmienila? */
        if (sensor_ok) {
            if (w->median >= w->pct_before + c->rise_min_pct) {
                w->no_rise = 0;
            } else if (++w->no_rise >= c->max_doses_no_rise) {
                fault(w, FLORA_WATER_FAULT_NO_RISE);
                break;
            }
        }
        enter(w, FLORA_WATER_OK, now);
        /* fallthrough - od razu ocena, czy potrzebna kolejna dawka */

    default:
        if (!sensor_ok) {
            w->state = FLORA_WATER_NO_SENSOR;
            break;
        }
        if (!w->need) {
            w->state = FLORA_WATER_OK;
            w->no_rise = 0;
            break;
        }
        if (!in->water_present) {
            w->state = FLORA_WATER_NO_WATER;
            break;
        }
        if (w->day_ml + 0.5f >= c->daily_max_ml) {
            w->state = FLORA_WATER_LIMIT;
            break;
        }
        if (!in->allowed || in->pump_busy) {
            w->state = FLORA_WATER_DRY;
            break;
        }
        w->pct_before = w->median;
        w->remaining_ml = fminf(c->dose_ml, c->daily_max_ml - w->day_ml);
        enter(w, FLORA_WATER_DOSING, now);
        request = w->remaining_ml;
        break;
    }

    if (request > 0.0f && in->max_chunk_ml > 0.0f && request > in->max_chunk_ml) {
        request = in->max_chunk_ml;
    }

    flora_water_out_t o = {
        .state        = w->state,
        .fault        = w->fault,
        .request_ml   = request,
        .median_pct   = w->have_median ? w->median : NAN,
        .day_ml       = w->day_ml,
        .remaining_ml = w->remaining_ml > 0.0f ? w->remaining_ml : 0.0f,
    };
    w->last = o;
    return o;
}
