#include <math.h>
#include <string.h>

#include "flora_pid.h"

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

void flora_pid_init(flora_pid_t *pid, const flora_pid_cfg_t *cfg)
{
    memset(pid, 0, sizeof(*pid));
    pid->cfg = *cfg;
}

void flora_pid_set_cfg(flora_pid_t *pid, const flora_pid_cfg_t *cfg)
{
    pid->cfg = *cfg;
    /* Nowy limit moze byc ciasniejszy od biezacej calki. */
    pid->integ = clampf(pid->integ, -cfg->i_max, cfg->i_max);
}

void flora_pid_reset(flora_pid_t *pid)
{
    pid->integ  = 0.0f;
    pid->d_filt = 0.0f;
    pid->primed = false;
    memset(&pid->last, 0, sizeof(pid->last));
}

float flora_pid_update(flora_pid_t *pid, float setpoint, float measured, float dt_s)
{
    const flora_pid_cfg_t *c = &pid->cfg;
    float e = setpoint - measured;

    float ff = c->kff * setpoint;
    if (fabsf(setpoint) > c->ks_deadband) {
        ff += setpoint > 0.0f ? c->ks : -c->ks;
    }

    float p = c->kp * e;

    float d = 0.0f;
    if (dt_s > 0.0f && pid->primed) {
        float raw = -(measured - pid->prev_meas) / dt_s;
        pid->d_filt += c->d_alpha * (raw - pid->d_filt);
        d = c->kd * pid->d_filt;
    }
    pid->prev_meas = measured;
    pid->primed    = true;

    /* Calka: najpierw kandydat, potem decyzja, czy go przyjac. */
    float integ = pid->integ;
    if (dt_s > 0.0f && c->ki != 0.0f) {
        float cand = clampf(integ + c->ki * e * dt_s, -c->i_max, c->i_max);
        float u    = ff + p + cand + d;
        bool  push_high = (u > c->out_max) && (e > 0.0f);
        bool  push_low  = (u < c->out_min) && (e < 0.0f);
        if (!push_high && !push_low) {
            integ = cand;
        }
    }
    pid->integ = integ;

    float out = clampf(ff + p + integ + d, c->out_min, c->out_max);
    pid->last = (flora_pid_terms_t){ .ff = ff, .p = p, .i = integ, .d = d, .out = out };
    return out;
}
