#include <math.h>
#include <string.h>

#include "flora_hkf.h"

static float wrap360(float d)
{
    d = fmodf(d, 360.0f);
    return d < 0.0f ? d + 360.0f : d;
}

/** Najkrotsza roznica a - b w zakresie -180..180. */
static float diff180(float a, float b)
{
    float d = fmodf(a - b + 540.0f, 360.0f);
    if (d < 0.0f) d += 360.0f;
    return d - 180.0f;
}

void flora_hkf_init(flora_hkf_t *kf, const flora_hkf_cfg_t *cfg, float psi0_deg, float sigma0_deg)
{
    memset(kf, 0, sizeof(*kf));
    kf->cfg = *cfg;
    kf->psi = wrap360(psi0_deg);
    kf->P[0][0] = sigma0_deg * sigma0_deg;
    kf->P[1][1] = cfg->sigma_b0 * cfg->sigma_b0;
}

void flora_hkf_reset_heading(flora_hkf_t *kf, float psi_deg, float sigma_deg)
{
    kf->psi = wrap360(psi_deg);
    kf->P[0][0] = sigma_deg * sigma_deg;
    kf->P[0][1] = kf->P[1][0] = 0.0f;
    kf->reject_run = 0;
}

void flora_hkf_predict(flora_hkf_t *kf, float omega_dps, float dt_s)
{
    if (dt_s <= 0.0f) {
        return;
    }
    kf->psi = wrap360(kf->psi + (omega_dps - kf->bz) * dt_s);

    /* F = [1 -dt; 0 1] */
    float p00 = kf->P[0][0], p01 = kf->P[0][1], p11 = kf->P[1][1];
    float n00 = p00 - 2.0f * dt_s * p01 + dt_s * dt_s * p11 + kf->cfg.q_psi * dt_s;
    float n01 = p01 - dt_s * p11;
    float n11 = p11 + kf->cfg.q_bias * dt_s;
    kf->P[0][0] = n00;
    kf->P[0][1] = kf->P[1][0] = n01;
    kf->P[1][1] = n11;
}

bool flora_hkf_would_accept(const flora_hkf_t *kf, float z_deg, float sigma_deg)
{
    float y = diff180(z_deg, kf->psi);
    float S = kf->P[0][0] + sigma_deg * sigma_deg;
    return S > 0.0f && y * y / S <= kf->cfg.gate;
}

bool flora_hkf_update(flora_hkf_t *kf, float z_deg, float sigma_deg)
{
    float R = sigma_deg * sigma_deg;
    float y = diff180(z_deg, kf->psi);
    float S = kf->P[0][0] + R;
    if (S <= 0.0f || y * y / S > kf->cfg.gate) {
        kf->n_rejected++;
        kf->reject_run++;
        return false;
    }
    float k0 = kf->P[0][0] / S, k1 = kf->P[1][0] / S;
    kf->psi = wrap360(kf->psi + k0 * y);
    kf->bz += k1 * y;

    /* Joseph: P = (I - K H) P (I - K H)' + K R K', H = [1 0] */
    float p00 = kf->P[0][0], p01 = kf->P[0][1], p11 = kf->P[1][1];
    float a = 1.0f - k0;
    float n00 = a * a * p00 + k0 * k0 * R;
    float n01 = a * (p01 - k1 * p00) + k0 * k1 * R;
    float n11 = p11 - 2.0f * k1 * p01 + k1 * k1 * p00 + k1 * k1 * R;
    kf->P[0][0] = n00;
    kf->P[0][1] = kf->P[1][0] = n01;
    kf->P[1][1] = n11;

    kf->n_accepted++;
    kf->reject_run = 0;
    return true;
}

bool flora_hkf_update_bias(flora_hkf_t *kf, float z_dps, float sigma_dps)
{
    /* H = [0 1]: mierzymy wprost b_z. */
    float R = sigma_dps * sigma_dps;
    float y = z_dps - kf->bz;
    float S = kf->P[1][1] + R;
    /* Bez bramki: po skoku temperatury dryf zmienia sie szybciej, niz
     * zaklada q_bias, i bramka odrzucalaby postoj w nieskonczonosc. Ze
     * robot naprawde stoi, pilnuje wolajacy (enkodery + prog obrotu). */
    if (S <= 0.0f) {
        return false;
    }
    float k0 = kf->P[0][1] / S, k1 = kf->P[1][1] / S;
    kf->psi = wrap360(kf->psi + k0 * y);
    kf->bz += k1 * y;

    /* Joseph: A = I - K H = [1 -k0; 0 1-k1] */
    float p00 = kf->P[0][0], p01 = kf->P[0][1], p11 = kf->P[1][1];
    float b = 1.0f - k1;
    float n00 = p00 - 2.0f * k0 * p01 + k0 * k0 * p11 + k0 * k0 * R;
    float n01 = b * (p01 - k0 * p11) + k0 * k1 * R;
    float n11 = b * b * p11 + k1 * k1 * R;
    kf->P[0][0] = n00;
    kf->P[0][1] = kf->P[1][0] = n01;
    kf->P[1][1] = n11;
    return true;
}

float flora_hkf_sigma(const flora_hkf_t *kf)
{
    float p = kf->P[0][0] > 0.0f ? kf->P[0][0] : 0.0f;
    return sqrtf(p + kf->cfg.sigma_floor * kf->cfg.sigma_floor);
}
