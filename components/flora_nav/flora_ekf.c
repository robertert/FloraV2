#include <math.h>
#include <string.h>

#include "flora_ekf.h"

#define N FLORA_EKF_N

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

float flora_ekf_wrap(float a)
{
    a = fmodf(a + (float)M_PI, 2.0f * (float)M_PI);
    if (a <= 0.0f) {
        a += 2.0f * (float)M_PI;
    }
    return a - (float)M_PI;
}

/** Wymusza symetrie P - w float bledy zaokraglen ja powoli rozjezdzaja. */
static void symmetrize(float P[N][N])
{
    for (int i = 0; i < N; i++) {
        for (int j = i + 1; j < N; j++) {
            float m = 0.5f * (P[i][j] + P[j][i]);
            P[i][j] = P[j][i] = m;
        }
    }
}

void flora_ekf_init(flora_ekf_t *ekf, const flora_ekf_cfg_t *cfg)
{
    memset(ekf, 0, sizeof(*ekf));
    ekf->cfg = *cfg;
    ekf->P[FLORA_EKF_B][FLORA_EKF_B] = (float)(M_PI * M_PI);
}

void flora_ekf_set_pose(flora_ekf_t *ekf, float x_mm, float y_mm, float th_rad,
                        float sigma_xy_mm, float sigma_th_rad, bool forget_b)
{
    ekf->x[FLORA_EKF_X]  = x_mm;
    ekf->x[FLORA_EKF_Y]  = y_mm;
    ekf->x[FLORA_EKF_TH] = flora_ekf_wrap(th_rad);
    ekf->heading_reject_run = 0;
    ekf->gyro_mis = ekf->gyro_applied = 0.0f;

    float pbb = forget_b ? (float)(M_PI * M_PI) : ekf->P[FLORA_EKF_B][FLORA_EKF_B];
    memset(ekf->P, 0, sizeof(ekf->P));
    ekf->P[FLORA_EKF_X][FLORA_EKF_X]   = sigma_xy_mm * sigma_xy_mm;
    ekf->P[FLORA_EKF_Y][FLORA_EKF_Y]   = sigma_xy_mm * sigma_xy_mm;
    ekf->P[FLORA_EKF_TH][FLORA_EKF_TH] = sigma_th_rad * sigma_th_rad;
    ekf->P[FLORA_EKF_B][FLORA_EKF_B]   = pbb;
}

void flora_ekf_predict_odom(flora_ekf_t *ekf, float ds_l, float ds_r, float dt_s)
{
    const float W   = ekf->cfg.track_mm;
    const float ds  = 0.5f * (ds_l + ds_r);
    const float dth = (ds_r - ds_l) / W;
    const float th  = ekf->x[FLORA_EKF_TH];
    const float m   = th + 0.5f * dth;   /* kierunek srodka luku */
    const float c   = cosf(m), s = sinf(m);

    ekf->x[FLORA_EKF_X]  += ds * c;
    ekf->x[FLORA_EKF_Y]  += ds * s;
    ekf->x[FLORA_EKF_TH]  = flora_ekf_wrap(th + dth);
    ekf->last_dth = dth;

    /* F = df/dx: tylko pozycja zalezy od kata. */
    float F[N][N] = {
        { 1, 0, -ds * s, 0 },
        { 0, 1,  ds * c, 0 },
        { 0, 0,  1,      0 },
        { 0, 0,  0,      1 },
    };
    /* G = df/d(ds_l, ds_r). */
    const float h = ds / (2.0f * W);
    float G[N][2] = {
        { 0.5f * c + h * s, 0.5f * c - h * s },
        { 0.5f * s - h * c, 0.5f * s + h * c },
        { -1.0f / W,        1.0f / W         },
        { 0.0f,             0.0f             },
    };
    const float ql = ekf->cfg.k_l * fabsf(ds_l);
    const float qr = ekf->cfg.k_r * fabsf(ds_r);

    /* P = F P F' + G Q G' */
    float FP[N][N] = {0};
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++)
            for (int k = 0; k < N; k++)
                FP[i][j] += F[i][k] * ekf->P[k][j];
    float Pn[N][N] = {0};
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            for (int k = 0; k < N; k++) {
                Pn[i][j] += FP[i][k] * F[j][k];
            }
            Pn[i][j] += G[i][0] * ql * G[j][0] + G[i][1] * qr * G[j][1];
        }
    }
    if (dt_s > 0.0f) {
        Pn[FLORA_EKF_B][FLORA_EKF_B] += ekf->cfg.q_b * dt_s;
    }
    memcpy(ekf->P, Pn, sizeof(Pn));
    symmetrize(ekf->P);
}

void flora_ekf_predict_static(flora_ekf_t *ekf, float dt_s, float q_xy, float q_th)
{
    ekf->last_dth = 0.0f;
    if (dt_s <= 0.0f) {
        return;
    }
    ekf->P[FLORA_EKF_X][FLORA_EKF_X]   += q_xy * dt_s;
    ekf->P[FLORA_EKF_Y][FLORA_EKF_Y]   += q_xy * dt_s;
    ekf->P[FLORA_EKF_TH][FLORA_EKF_TH] += q_th * dt_s;
    ekf->P[FLORA_EKF_B][FLORA_EKF_B]   += ekf->cfg.q_b * dt_s;
}

/**
 * Wspolny krok aktualizacji dla pomiaru wymiaru m (1 albo 2).
 *
 * y - innowacja (juz zawinieta tam, gdzie to kat), H - Jacobian m x N,
 * R - kowariancja pomiaru. Zwraca false, gdy pomiar nie przeszedl bramki.
 * Kowariancja w formie Josepha: wolniejsza, ale zostaje dodatnio okreslona
 * mimo bledow zaokraglen w float.
 */
static bool kf_update(flora_ekf_t *ekf, int m, const float *y,
                      const float H[][N], const float R[][2], float gate)
{
    /* PHt = P H' (N x m), S = H P H' + R (m x m) */
    float PHt[N][2] = {{0}};
    for (int i = 0; i < N; i++)
        for (int j = 0; j < m; j++)
            for (int k = 0; k < N; k++)
                PHt[i][j] += ekf->P[i][k] * H[j][k];

    float S[2][2] = {{0}};
    for (int i = 0; i < m; i++)
        for (int j = 0; j < m; j++) {
            for (int k = 0; k < N; k++) {
                S[i][j] += H[i][k] * PHt[k][j];
            }
            S[i][j] += R[i][j];
        }

    float Si[2][2];
    if (m == 1) {
        if (S[0][0] <= 0.0f) return false;
        Si[0][0] = 1.0f / S[0][0];
    } else {
        float det = S[0][0] * S[1][1] - S[0][1] * S[1][0];
        if (det <= 0.0f) return false;
        Si[0][0] =  S[1][1] / det;
        Si[0][1] = -S[0][1] / det;
        Si[1][0] = -S[1][0] / det;
        Si[1][1] =  S[0][0] / det;
    }

    /* Odleglosc Mahalanobisa y' S^-1 y */
    float d2 = 0.0f;
    for (int i = 0; i < m; i++)
        for (int j = 0; j < m; j++)
            d2 += y[i] * Si[i][j] * y[j];
    if (d2 > gate) {
        ekf->n_rejected++;
        return false;
    }

    /* K = P H' S^-1 (N x m) */
    float K[N][2] = {{0}};
    for (int i = 0; i < N; i++)
        for (int j = 0; j < m; j++)
            for (int k = 0; k < m; k++)
                K[i][j] += PHt[i][k] * Si[k][j];

    for (int i = 0; i < N; i++)
        for (int j = 0; j < m; j++)
            ekf->x[i] += K[i][j] * y[j];
    ekf->x[FLORA_EKF_TH] = flora_ekf_wrap(ekf->x[FLORA_EKF_TH]);
    ekf->x[FLORA_EKF_B]  = flora_ekf_wrap(ekf->x[FLORA_EKF_B]);

    /* A = I - K H */
    float A[N][N];
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++) {
            float kh = 0.0f;
            for (int k = 0; k < m; k++) kh += K[i][k] * H[k][j];
            A[i][j] = (i == j ? 1.0f : 0.0f) - kh;
        }
    /* P = A P A' + K R K' */
    float AP[N][N] = {{0}};
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++)
            for (int k = 0; k < N; k++)
                AP[i][j] += A[i][k] * ekf->P[k][j];
    float Pn[N][N] = {{0}};
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++) {
            for (int k = 0; k < N; k++) Pn[i][j] += AP[i][k] * A[j][k];
            for (int a = 0; a < m; a++)
                for (int b = 0; b < m; b++)
                    Pn[i][j] += K[i][a] * R[a][b] * K[j][b];
        }
    memcpy(ekf->P, Pn, sizeof(Pn));
    symmetrize(ekf->P);
    return true;
}

bool flora_ekf_update_heading(flora_ekf_t *ekf, float psi_rad, float sigma_rad)
{
    float pred = ekf->x[FLORA_EKF_B] - ekf->x[FLORA_EKF_TH];
    float y[1] = { flora_ekf_wrap(psi_rad - pred) };
    const float H[1][N] = { { 0.0f, 0.0f, -1.0f, 1.0f } };
    const float R[1][2] = { { sigma_rad * sigma_rad, 0.0f } };
    bool ok = kf_update(ekf, 1, y, H, R, ekf->cfg.gate_1d);

    /* Wyjscie z blokady: kurs uparcie mowi co innego niz odometria, wiec
     * to theta jest niepewne bardziej, niz filtr sadzi (poslizg, uderzenie).
     * Po powiekszeniu P innowacja miesci sie w 1 sigma i bramka przepuszcza. */
    if (!ok && ekf->cfg.heading_recover_n &&
        ++ekf->heading_reject_run >= ekf->cfg.heading_recover_n) {
        ekf->P[FLORA_EKF_TH][FLORA_EKF_TH] += y[0] * y[0];
        ekf->heading_reject_run = 0;
        ekf->n_recovered++;
        ok = kf_update(ekf, 1, y, H, R, ekf->cfg.gate_1d);
    }
    if (ok) {
        ekf->n_heading++;
        ekf->heading_reject_run = 0;
        /* Kurs bezwzgledny wlasnie poprawil theta - niezgodnosc liczymy od nowa. */
        ekf->gyro_mis = ekf->gyro_applied = 0.0f;
    }
    return ok;
}

void flora_ekf_observe_gyro(flora_ekf_t *ekf, float dth_gyro_rad)
{
    ekf->gyro_mis += ekf->last_dth - dth_gyro_rad;
    float excess = fabsf(ekf->gyro_mis) - ekf->cfg.gyro_deadband_rad;
    if (excess <= 0.0f) {
        return;
    }
    /* Dokladamy tylko przyrost: laczna dodana wariancja = excess^2,
     * czyli tyle, ile wynosi kwadrat bledu systematycznego. */
    float var = excess * excess;
    if (var > ekf->gyro_applied) {
        ekf->P[FLORA_EKF_TH][FLORA_EKF_TH] += var - ekf->gyro_applied;
        ekf->gyro_applied = var;
    }
}

bool flora_ekf_update_tag(flora_ekf_t *ekf, float tag_x_mm, float tag_y_mm,
                          float cam_x_mm, float cam_y_mm,
                          float d_mm, float beta_rad,
                          float sigma_d_mm, float sigma_beta_rad)
{
    const float px = ekf->x[FLORA_EKF_X], py = ekf->x[FLORA_EKF_Y];
    const float th = ekf->x[FLORA_EKF_TH];
    const float c = cosf(th), s = sinf(th);

    /* Kamera w ukladzie mapy i jej ramie wzgledem srodka osi kol. */
    const float ox = cam_x_mm * c - cam_y_mm * s;
    const float oy = cam_x_mm * s + cam_y_mm * c;
    const float dx = tag_x_mm - (px + ox);
    const float dy = tag_y_mm - (py + oy);
    const float r2 = dx * dx + dy * dy;
    const float r  = sqrtf(r2);
    if (r < 1.0f) {
        return false;   /* znacznik w kamerze - geometria bez sensu */
    }

    float y[2] = {
        d_mm - r,
        flora_ekf_wrap(beta_rad - (atan2f(dy, dx) - th)),
    };
    /* d(dx)/dth = oy, d(dy)/dth = -ox (kamera obraca sie z robotem). */
    const float H[2][N] = {
        { -dx / r,  -dy / r,  (dx * oy - dy * ox) / r,       0.0f },
        {  dy / r2, -dx / r2, (-dx * ox - dy * oy) / r2 - 1.0f, 0.0f },
    };
    const float R[2][2] = {
        { sigma_d_mm * sigma_d_mm, 0.0f },
        { 0.0f, sigma_beta_rad * sigma_beta_rad },
    };
    bool ok = kf_update(ekf, 2, y, H, R, ekf->cfg.gate_2d);
    if (ok) ekf->n_tag++;
    return ok;
}

float flora_ekf_pos_sigma_max(const flora_ekf_t *ekf)
{
    float a = ekf->P[0][0], b = ekf->P[0][1], c = ekf->P[1][1];
    float half = 0.5f * (a - c);
    float lmax = 0.5f * (a + c) + sqrtf(half * half + b * b);
    return lmax > 0.0f ? sqrtf(lmax) : 0.0f;
}
