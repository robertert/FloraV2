/**
 * @file flora_navcore.h
 * @brief Jeden takt nawigacji: warianty EKF, bramki kursu, slonca i tagow.
 *
 * Wydzielone z hub/main/hub_nav.c, zeby TEN SAM kod liczyl na robocie
 * i na komputerze nad nagraniem przejazdu (host/replay.c). Bez ESP-IDF:
 * zegar, enkodery, lacze z glowica i MQTT zostaja w hub_nav.c, ktory
 * tylko przeklada je na flora_navcore_in_t. Rekorder (hub_rec) zapisuje
 * dokladnie te wejscia - format w flora_navlog.h.
 *
 * Konwencje jak w flora_ekf.h: theta CCW, kursy kompasowe CW.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "flora_ekf.h"
#include "flora_sungate.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FLORA_NAVCORE_SRC_ODOM  (1u << 0)
#define FLORA_NAVCORE_SRC_HEAD  (1u << 1)   /**< kurs i zyroskop z glowicy */
#define FLORA_NAVCORE_SRC_SUN   (1u << 2)
#define FLORA_NAVCORE_SRC_TAGS  (1u << 3)

#define FLORA_NAVCORE_VARIANTS    6
#define FLORA_NAVCORE_MAX_TAGS    16
#define FLORA_NAVCORE_MAX_DETS    4
#define FLORA_NAVCORE_STILL_TICKS 25   /**< 0,5 s przy 50 Hz */

/** Geometria i miejsce - z hub_params na robocie, z linii P w nagraniu. */
typedef struct {
    float  track_mm;
    float  cam_x_mm, cam_y_mm;
    double lat_deg, lon_deg;
    float  decl_deg;
} flora_navcore_geom_t;

/** Strojenie - na komputerze mozna je zmieniac i porownywac na nagraniu. */
typedef struct {
    flora_ekf_cfg_t     ekf;      /**< track_mm nadpisywane z geometrii */
    flora_sungate_cfg_t sun;      /**< lat/lon/decl nadpisywane z geometrii */
    float k_wheel;                /**< wariancja drogi kola [mm^2/mm] */
    float head_every_s;           /**< kurs z glowicy do EKF najwyzej co tyle */
    float head_sigma_floor_deg;   /**< podloga niepewnosci kursu glowicy */
    float sun_every_s;
    float sun_mag_max_age_s;      /**< kurs glowicy starszy nie jest odniesieniem slonca */
    float gyro_max_age_s;         /**< rate_z starsze nie opisuje taktu */
    float still_mm;               /**< ponizej tej drogi w oknie STILL_TICKS = postoj */
    float max_step_mm;            /**< wiecej w jednym takcie = skok licznika */
    float static_q_xy;            /**< model bez odometrii [mm^2/s] */
    float static_q_th;            /**< [rad^2/s] */
} flora_navcore_tuning_t;

#define FLORA_NAVCORE_DEFAULT_TUNING()                        \
    {                                                         \
        .ekf  = FLORA_EKF_DEFAULT_CFG(),                      \
        .sun  = FLORA_SUNGATE_DEFAULT_CFG(),                  \
        .k_wheel              = 0.5f,                         \
        .head_every_s         = 0.5f,                         \
        .head_sigma_floor_deg = 4.0f,                         \
        .sun_every_s          = 5.0f,                         \
        .sun_mag_max_age_s    = 2.0f,                         \
        .gyro_max_age_s       = 0.15f,                        \
        .still_mm             = 1.0f,                         \
        .max_step_mm          = 100.0f,                       \
        .static_q_xy          = 1.0e4f,                       \
        .static_q_th          = 0.5236f * 0.5236f,            \
    }

/** ATTITUDE z glowicy, juz rozpakowane. */
typedef struct {
    bool    heading_valid;
    bool    mag_valid;
    float   heading_deg;   /**< kompasowy, CW */
    float   rate_dps;      /**< tempo kursu, CW */
    float   sigma_deg;     /**< 0 = glowica nie podala, wtedy z quality */
    uint8_t quality;
    int32_t age_us;        /**< wiek probki w chwili odbioru; < 0 nieznany */
} flora_navcore_att_t;

/** Krzyz VEML. */
typedef struct {
    bool  valid;
    float azimuth_deg;     /**< slonce wzgledem glowicy, CW */
    float lux[4];          /**< N, E, S, W */
} flora_navcore_sun_t;

/** Wykrycie znacznika z kamery. */
typedef struct {
    uint16_t id;
    uint16_t distance_mm;  /**< 0 = brak ogniskowej */
    float    bearing_deg;  /**< dodatni W PRAWO (jak flora_tags) */
    uint8_t  quality;
} flora_navcore_det_t;

typedef struct {
    uint16_t id;
    float    x_mm, y_mm;
} flora_navcore_tag_t;

/** Wejscia jednego taktu (50 Hz). NULL / 0 = w tym takcie nie przyszlo. */
typedef struct {
    int64_t t_us;                        /**< zegar bryly */
    float   dl_mm, dr_mm, dt_s;          /**< przyrost drog kol */
    const flora_navcore_att_t *att;
    const flora_navcore_sun_t *sun;
    int64_t unix_s;                      /**< UTC; <= 0 nieznany (tylko slonce) */
    const flora_navcore_det_t *dets;
    int     n_dets;
} flora_navcore_in_t;

typedef struct {
    const char *name;
    uint8_t     src;
    flora_ekf_t ekf;
    uint32_t    n_head, n_sun, n_tag;
} flora_navcore_variant_t;

typedef struct {
    flora_navcore_tuning_t  tun;
    flora_navcore_geom_t    geom;
    flora_navcore_variant_t v[FLORA_NAVCORE_VARIANTS];

    flora_navcore_tag_t tags[FLORA_NAVCORE_MAX_TAGS];
    int                 n_tags;

    flora_sungate_t gate;

    /* postoj */
    float moved[FLORA_NAVCORE_STILL_TICKS];
    int   moved_i;
    bool  still;

    /* ostatnie ATTITUDE */
    bool                have_att;
    flora_navcore_att_t att;
    int64_t             att_rx_us;
    bool                gyro_ok;
    float               gyro_vs_odo_dps;
    int32_t             att_age_ms;    /**< ostatnio uzyty wiek; -1 nieznany */

    /* kurs glowicy jako odniesienie dla slonca */
    float   head_psi;
    bool    head_mag;
    int64_t head_us;

    bool    head_taken, sun_taken;
    int64_t last_head_us, last_sun_us;

    int      sun_gate;                 /**< flora_sungate_result_t, NONE na starcie */
    float    sun_diff_deg;
    uint32_t n_sun_seen, n_sun_gated;
    uint32_t n_steps, n_jumps;
} flora_navcore_t;

void flora_navcore_init(flora_navcore_t *c, const flora_navcore_tuning_t *tun,
                        const flora_navcore_geom_t *geom);

/** Nowa geometria (po kalibracji): rozstaw w EKF, miejsce w bramkach slonca. */
void flora_navcore_set_geom(flora_navcore_t *c, const flora_navcore_geom_t *geom);

/**
 * Jeden takt. @return false gdy pominiety (skok licznika enkodera) -
 * wtedy nic sie nie zmienia poza licznikiem n_jumps.
 */
bool flora_navcore_step(flora_navcore_t *c, const flora_navcore_in_t *in);

/** Poza wszystkich wariantow; b liczone od nowa (patrz flora_ekf_set_pose). */
void flora_navcore_reset_pose(flora_navcore_t *c, float x_mm, float y_mm, float th_deg);

const flora_navcore_tag_t *flora_navcore_tag(const flora_navcore_t *c, uint16_t id);

#ifdef __cplusplus
}
#endif
