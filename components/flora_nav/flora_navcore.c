#include <math.h>
#include <string.h>

#include "flora_navcore.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define D2R(d) ((d) * (float)M_PI / 180.0f)
#define R2D(r) ((r) * 180.0f / (float)M_PI)

#define ODOM FLORA_NAVCORE_SRC_ODOM
#define HEAD FLORA_NAVCORE_SRC_HEAD
#define SUN  FLORA_NAVCORE_SRC_SUN
#define TAGS FLORA_NAVCORE_SRC_TAGS

/* Dopisanie kombinacji = jedna linijka (i FLORA_NAVCORE_VARIANTS).
 * Wariant 0 to glowny wynik; nazwa trafia do tematu MQTT tele/pose_<nazwa>,
 * wiec bez spacji i ukosnikow. */
static const struct {
    const char *name;
    uint8_t     src;
} VARIANTS[FLORA_NAVCORE_VARIANTS] = {
    { "full",      ODOM | HEAD | SUN | TAGS },
    { "odom",      ODOM },
    { "head",      ODOM | HEAD },
    { "head_sun",  ODOM | HEAD | SUN },
    { "odom_tags", ODOM | TAGS },
    { "no_odom",   HEAD | SUN | TAGS },
};

void flora_navcore_init(flora_navcore_t *c, const flora_navcore_tuning_t *tun,
                        const flora_navcore_geom_t *geom)
{
    memset(c, 0, sizeof(*c));
    c->tun  = *tun;
    c->geom = *geom;

    flora_ekf_cfg_t cfg = tun->ekf;
    cfg.track_mm = geom->track_mm;
    cfg.k_l = cfg.k_r = tun->k_wheel;
    for (int i = 0; i < FLORA_NAVCORE_VARIANTS; i++) {
        c->v[i].name = VARIANTS[i].name;
        c->v[i].src  = VARIANTS[i].src;
        flora_ekf_init(&c->v[i].ekf, &cfg);
    }
    flora_sungate_init(&c->gate, &tun->sun);
    flora_navcore_set_geom(c, geom);
    c->sun_gate   = FLORA_SUNGATE_NONE;
    c->att_age_ms = -1;
}

void flora_navcore_set_geom(flora_navcore_t *c, const flora_navcore_geom_t *geom)
{
    c->geom = *geom;
    for (int i = 0; i < FLORA_NAVCORE_VARIANTS; i++) {
        c->v[i].ekf.cfg.track_mm = geom->track_mm;
    }
    c->gate.cfg.lat_deg         = geom->lat_deg;
    c->gate.cfg.lon_deg         = geom->lon_deg;
    c->gate.cfg.declination_deg = geom->decl_deg;
    flora_sungate_reset(&c->gate);   /* stare pomiary liczone przy innym miejscu */
}

void flora_navcore_reset_pose(flora_navcore_t *c, float x_mm, float y_mm, float th_deg)
{
    /* Mala, niezerowa niepewnosc: robot postawiony "na oko" w znanym miejscu.
     * b liczone od nowa: nowe theta ze starym b dawaloby kurs poza bramka. */
    for (int i = 0; i < FLORA_NAVCORE_VARIANTS; i++) {
        flora_ekf_set_pose(&c->v[i].ekf, x_mm, y_mm, D2R(th_deg), 20.0f, D2R(2.0f), true);
    }
    flora_sungate_reset(&c->gate);   /* stare pomiary slonca liczone przy innym theta */
}

const flora_navcore_tag_t *flora_navcore_tag(const flora_navcore_t *c, uint16_t id)
{
    for (int i = 0; i < c->n_tags; i++) {
        if (c->tags[i].id == id) {
            return &c->tags[i];
        }
    }
    return NULL;
}

/** Pomiar kursu przygotowany dla wariantow w biezacym takcie. */
typedef struct {
    bool  valid;
    float psi, sigma;
} meas_t;

/**
 * Zapasowa niepewnosc kursu z `quality`, gdy glowica nie podala sigma.
 * 90 (z magnetometrem) -> 4 st, 50 (sam zyroskop) -> 16 st.
 */
static float sigma_from_quality(uint8_t quality)
{
    float q = quality > 100 ? 100.0f : (float)quality;
    float deg = 4.0f + (90.0f - q) * 0.3f;
    if (deg < 3.0f)  deg = 3.0f;
    if (deg > 45.0f) deg = 45.0f;
    return D2R(deg);
}

static bool due(bool *taken, int64_t *last_us, int64_t now_us, float every_s)
{
    if (*taken && now_us - *last_us < (int64_t)(every_s * 1e6f)) {
        return false;
    }
    *taken   = true;
    *last_us = now_us;
    return true;
}

static meas_t take_heading(flora_navcore_t *c, int64_t now_us, const flora_navcore_att_t *att)
{
    meas_t m = { 0 };
    if (!att || !att->heading_valid) {
        return m;
    }
    float psi_deg = att->heading_deg;

    /* Kurs przesuwamy o obrot od chwili pomiaru do teraz. */
    c->att_age_ms = -1;
    if (att->age_us >= 0 && att->age_us < 500000) {
        c->att_age_ms = att->age_us / 1000;
        psi_deg += att->rate_dps * (att->age_us / 1e6f);
    }
    float psi = D2R(psi_deg);

    /* Kazda probka odswieza odniesienie dla bramki slonca, niezaleznie
     * od tego, czy trafi do EKF. */
    c->head_psi = psi;
    c->head_mag = att->mag_valid;
    c->head_us  = now_us;

    if (!due(&c->head_taken, &c->last_head_us, now_us, c->tun.head_every_s)) {
        return m;
    }
    m.valid = true;
    m.psi   = psi;
    m.sigma = att->sigma_deg > 0.0f ? D2R(att->sigma_deg) : sigma_from_quality(att->quality);
    /* Podloga: bledy magnetometru sa skorelowane w czasie, a EKF bierze
     * kolejne pomiary jako niezalezne. */
    if (m.sigma < D2R(c->tun.head_sigma_floor_deg)) {
        m.sigma = D2R(c->tun.head_sigma_floor_deg);
    }
    return m;
}

static meas_t take_sun(flora_navcore_t *c, int64_t now_us, const flora_navcore_sun_t *sun,
                       int64_t unix_s)
{
    meas_t m = { 0 };
    if (!sun || !sun->valid) {
        return m;
    }
    c->n_sun_seen++;

    flora_sungate_in_t in = {
        .azimuth_meas_deg = sun->azimuth_deg,
        .unix_s           = unix_s > 0 ? unix_s : 0,
        .th_ref_rad       = c->v[0].ekf.x[FLORA_EKF_TH],
        .head_mag_ok      = c->head_mag &&
                            now_us - c->head_us < (int64_t)(c->tun.sun_mag_max_age_s * 1e6f),
        .head_psi_rad     = c->head_psi,
    };
    memcpy(in.lux, sun->lux, sizeof in.lux);
    /* ZALOZENIE DO SPRAWDZENIA: kanal N krzyza i os odniesienia kursu IMU
     * patrza w te sama strone glowicy. Jesli nie, wyjdzie staly konflikt
     * z magnetometrem (sun_vs_mag_deg w tele/nav_sun) do odjecia tutaj. */
    flora_sungate_out_t out;
    flora_sungate_result_t g = flora_sungate_eval(&c->gate, &in, &out);
    if (out.vs_mag_deg != 0.0f) {
        c->sun_diff_deg = out.vs_mag_deg;
    }
    c->sun_gate = g;
    if (g != FLORA_SUNGATE_OK) {
        c->n_sun_gated++;
        return m;
    }
    if (!due(&c->sun_taken, &c->last_sun_us, now_us, c->tun.sun_every_s)) {
        return m;
    }
    m.valid = true;
    m.psi   = out.psi_rad;
    m.sigma = out.sigma_rad;
    return m;
}

static void apply_tags(flora_navcore_t *c, flora_navcore_variant_t *v,
                       const flora_navcore_det_t *dets, int n)
{
    for (int i = 0; i < n; i++) {
        const flora_navcore_det_t *t = &dets[i];
        const flora_navcore_tag_t *pos = flora_navcore_tag(c, t->id);
        if (!pos || t->distance_mm == 0) {
            continue;   /* nieznany palik albo brak kalibracji ogniskowej */
        }
        /* Gorsza detekcja = szerszy rozrzut: quality 100 -> x1, 50 -> x2. */
        float q = t->quality < 10 ? 10.0f : (float)t->quality;
        float k = 1.0f + (100.0f - q) / 50.0f;
        float d = (float)t->distance_mm;
        /* flora_tags: kat dodatni W PRAWO; EKF: CCW, czyli w lewo. */
        float beta = -D2R(t->bearing_deg);
        if (flora_ekf_update_tag(&v->ekf, pos->x_mm, pos->y_mm,
                                 c->geom.cam_x_mm, c->geom.cam_y_mm,
                                 d, beta, (20.0f + 0.02f * d) * k, D2R(1.5f) * k)) {
            v->n_tag++;
        }
    }
}

bool flora_navcore_step(flora_navcore_t *c, const flora_navcore_in_t *in)
{
    const float dl = in->dl_mm, dr = in->dr_mm, dt = in->dt_s;
    if (fabsf(dl) > c->tun.max_step_mm || fabsf(dr) > c->tun.max_step_mm) {
        c->n_jumps++;
        return false;
    }
    c->n_steps++;

    c->moved[c->moved_i] = fabsf(dl) + fabsf(dr);
    c->moved_i = (c->moved_i + 1) % FLORA_NAVCORE_STILL_TICKS;
    float recent = 0.0f;
    for (int i = 0; i < FLORA_NAVCORE_STILL_TICKS; i++) recent += c->moved[i];
    c->still = recent < c->tun.still_mm;

    /* Zyroskop: obrot w ukladzie mapy. rate to tempo kursu kompasowego
     * (CW), theta jest CCW - stad minus. */
    if (in->att) {
        c->att       = *in->att;
        c->att_rx_us = in->t_us;
        c->have_att  = true;
    }
    c->gyro_ok = c->have_att && c->att.heading_valid &&
                 in->t_us - c->att_rx_us < (int64_t)(c->tun.gyro_max_age_s * 1e6f);
    float dth_gyro = c->gyro_ok ? -D2R(c->att.rate_dps) * dt : 0.0f;
    if (c->gyro_ok && dt > 0.0f) {
        float dth_odo = (dr - dl) / c->geom.track_mm;
        float mis_dps = R2D(dth_odo - dth_gyro) / dt;
        c->gyro_vs_odo_dps += 0.05f * (mis_dps - c->gyro_vs_odo_dps);   /* ~0,4 s */
    }

    /* Pomiary raz na takt, wspolne dla wszystkich wariantow. */
    meas_t head = take_heading(c, in->t_us, in->att);
    meas_t sun  = take_sun(c, in->t_us, in->sun, in->unix_s);
    /* Znaczniki tylko na postoju (D-034). */
    int n_dets = (in->dets && c->still) ? in->n_dets : 0;
    if (n_dets > FLORA_NAVCORE_MAX_DETS) n_dets = FLORA_NAVCORE_MAX_DETS;

    for (int i = 0; i < FLORA_NAVCORE_VARIANTS; i++) {
        flora_navcore_variant_t *v = &c->v[i];
        if (v->src & ODOM) {
            flora_ekf_predict_odom(&v->ekf, dl, dr, dt);
            /* Zyroskop jest zrodlem glowicy - wariant "odom" zostaje czysty. */
            if ((v->src & HEAD) && c->gyro_ok) {
                flora_ekf_observe_gyro(&v->ekf, dth_gyro);
            }
        } else {
            flora_ekf_predict_static(&v->ekf, dt, c->tun.static_q_xy, c->tun.static_q_th);
        }
        if ((v->src & HEAD) && head.valid &&
            flora_ekf_update_heading(&v->ekf, head.psi, head.sigma)) {
            v->n_head++;
        }
        if ((v->src & SUN) && sun.valid &&
            flora_ekf_update_heading(&v->ekf, sun.psi, sun.sigma)) {
            v->n_sun++;
        }
        if ((v->src & TAGS) && n_dets > 0) {
            apply_tags(c, v, in->dets, n_dets);
        }
    }
    return true;
}
