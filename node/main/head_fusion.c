#include <inttypes.h>
#include <string.h>

#include "head_fusion.h"
#include "head_cmd.h"
#include "head_math.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "flora_head_board.h"
#include "flora_hkf.h"
#include "flora_magcal.h"
#include "flora_magdist.h"
#include "flora_msgs.h"
#include "flora_store.h"

static const char *TAG = "fusion";

/* Tempo petli. 60 Hz daje dokladnie 20 Hz ATTITUDE po podziale na 3,
 * a blad calkowania zyroskopu to 0,5 stopnia na probke przy obrocie
 * 30 st/s - pomijalny. */
#define LOOP_HZ        60
#define LOOP_MS        (1000 / LOOP_HZ)
#define MAG_DIVIDER    6     /**< magnetometr 10 Hz; korekta dryfu nie musi byc szybsza */
#define MAG_PERIOD_MS  (MAG_DIVIDER * 1000 / LOOP_HZ)   /* 100, nie 6 * 16 = 96 */
#define ATT_DIVIDER    3     /**< ATTITUDE 20 Hz, zgodnie z kontraktem */
#define INFO_DIVIDER   LOOP_HZ   /**< MAG_INFO 1 Hz */

/* Po tylu nieudanych odczytach z rzedu uznajemy IMU za martwy. Jeden
 * zgubiony odczyt to normalna kolizja na magistrali, nie awaria. */
#define FAULT_THRESHOLD 10

/* Szum kursu z magnetometru po kalibracji. Przy przechyle wiekszym niz
 * TILT_LIMIT korekta przechylu traci dokladnosc - pomiar wazy wtedy mniej. */
#define MAG_SIGMA_DEG        4.0f
#define MAG_SIGMA_TILTED_DEG 8.0f

/* Bez magnetometru kurs jest tylko wzgledny wobec startu: niepewnosc
 * kierunku bezwzglednego jest pelna. Bryla dostaje to w sigma. */
#define NOMAG_SIGMA0_DEG     180.0f

/* Przechyl, powyzej ktorego kurs przestaje byc wiarygodny i wedruje
 * flaga TILTED. Korekta dziala dalej, ale blad rosnie nieliniowo. */
#define TILT_LIMIT_DEG 35.0f

/* Pochylenie i przechyl: filtr komplementarny (head_tilt_fuse). */
#define TILT_TAU_S       1.0f    /**< korekta z akcelerometru; drgania masztu sa szybsze */
#define TILT_MAX_G_DEV   0.08f   /**< dalej od 1 g akcelerometr nie poprawia */
#define TILT_MAX_GAP_S   0.5f    /**< po dluzszej przerwie start od samego akcelerometru */
/* Gdy wypadkowa odbiega od 1 g, pochylenie niesie sam zyroskop - magnetometr
 * dostaje wtedy wieksza sigma, jak przy duzym przechyle. */
#define MAG_MAX_G_DEV    0.05f

/* --- ZUPT: dryf zyroskopu na postoju --- */
#define ZUPT_SAMPLES       60     /**< 1 s przy 60 Hz */
#define ZUPT_MAX_RATE_DPS  2.0f   /**< glowica nie moze sie obracac (np. recznie) */
#define ZUPT_MAX_G_DEV     0.03f
#define ZUPT_SIGMA_DPS     0.05f  /**< niepewnosc sredniej z 1 s */

/* --- kalibracja offsetu --- */
#define CAL_SAMPLES        120    /**< 2 s przy 60 Hz */
#define CAL_MAX_RATE_DPS   2.0f   /**< powyzej tego uznajemy ruch */
#define CAL_MAX_G_DEV      0.08f  /**< odstepstwo wypadkowej od 1 g */
#define CAL_MAX_ATTEMPTS   3

/* Bufor kalibracji magnetometru (~10 kB) - statycznie, poza stosem zadania. */
static flora_magcal_collector_t s_collect;

static struct {
    flora_icm20948_handle_t imu;
    flora_link_handle_t     down;
    flora_store_t           store;

    head_fusion_state_t state;
    uint32_t  tick;
    int64_t   last_us;

    /* kurs: Kalman [psi, dryf] */
    flora_hkf_t kf;
    float     mag_heading;
    bool      mag_valid;      /**< jest swiezy, niezaklocony pomiar */
    bool      mag_seen;       /**< magnetometr w ogole odpowiada */
    head_attitude_t att;        /**< po offsecie poziomu - tego uzywa reszta */
    flora_icm20948_data_t last; /**< ostatni udany odczyt, do podgladu w logu */
    head_attitude_t tilt_raw;   /**< wyjscie filtra, bez offsetu */
    bool      tilt_init;
    int64_t   tilt_last_us;

    /* ZUPT */
    int       zupt_n;
    double    zupt_sum;
    uint32_t  zupt_count;

    /* magnetometr: kalibracja i zaklocenia */
    flora_magcal_t  mag_cal;
    bool            mag_cal_ok;
    flora_magdist_t dist;
    float     mag_h, mag_dip, mag_total;
    bool      collecting;
    int64_t   collect_end_us;

    /* poziom akcelerometru: przekrzywienie plytki na maszcie */
    float     lvl_off_pitch, lvl_off_roll;
    bool      lvl_measuring;
    uint8_t   lvl_step;
    int       lvl_n;
    double    lvl_sum[3];
    float     lvl_peak_rate;
    bool      lvl_have_first;
    float     lvl_first_pitch, lvl_first_roll;

    uint8_t   att_divider;
    uint32_t  fail_streak;
    uint32_t  errors;
    bool      heading_valid;

    /* kalibracja zyro */
    int       cal_n;
    int       cal_attempts;
    double    cal_sum[3];
    float     cal_peak_rate;
    int64_t   state_entered_us;
} s_f;

const char *head_fusion_state_name(head_fusion_state_t s)
{
    switch (s) {
        case HEAD_FUSION_INIT:      return "INIT";
        case HEAD_FUSION_BIAS_LOAD: return "BIAS_LOAD";
        case HEAD_FUSION_BIAS_CAL:  return "BIAS_CAL";
        case HEAD_FUSION_ALIGN:     return "ALIGN";
        case HEAD_FUSION_RUN:       return "RUN";
        case HEAD_FUSION_FAULT:     return "FAULT";
        default:                    return "?";
    }
}

static void go(head_fusion_state_t next)
{
    if (s_f.state == next) {
        return;
    }
    ESP_LOGI(TAG, "%s -> %s", head_fusion_state_name(s_f.state),
             head_fusion_state_name(next));
    s_f.state = next;
    s_f.state_entered_us = esp_timer_get_time();
}

static int64_t in_state_ms(void)
{
    return (esp_timer_get_time() - s_f.state_entered_us) / 1000;
}

/* ================= OFFSET ZYROSKOPU ================= */

static bool bias_load(void)
{
    if (!s_f.store) {
        return false;
    }
    bool ok = false;
    flora_store_get_bool(s_f.store, FLORA_HEAD_NVS_GYRO_OK, &ok, false);
    if (!ok) {
        return false;
    }
    float b[3] = { 0 };
    flora_store_get_float(s_f.store, FLORA_HEAD_NVS_GYRO_X, &b[0], 0.0f);
    flora_store_get_float(s_f.store, FLORA_HEAD_NVS_GYRO_Y, &b[1], 0.0f);
    flora_store_get_float(s_f.store, FLORA_HEAD_NVS_GYRO_Z, &b[2], 0.0f);

    /* Sterownik IMU trzyma offset u siebie i odejmuje go przy odczycie,
     * wiec wgrywamy go tam zamiast korygowac po drodze. */
    flora_icm20948_set_gyro_bias(s_f.imu, b);
    ESP_LOGI(TAG, "offset z NVS: %.3f %.3f %.3f dps", b[0], b[1], b[2]);
    return true;
}

static void bias_save(const float b[3])
{
    if (!s_f.store) {
        ESP_LOGW(TAG, "brak NVS - offset nie zostanie zapamietany");
        return;
    }
    flora_store_set_float(s_f.store, FLORA_HEAD_NVS_GYRO_X, b[0]);
    flora_store_set_float(s_f.store, FLORA_HEAD_NVS_GYRO_Y, b[1]);
    flora_store_set_float(s_f.store, FLORA_HEAD_NVS_GYRO_Z, b[2]);
    flora_store_set_bool(s_f.store, FLORA_HEAD_NVS_GYRO_OK, true);
    ESP_LOGI(TAG, "offset zapisany w NVS");
}

static void cal_begin(void)
{
    s_f.cal_n = 0;
    s_f.cal_peak_rate = 0.0f;
    memset(s_f.cal_sum, 0, sizeof(s_f.cal_sum));
    float zero[3] = { 0, 0, 0 };
    /* Zerujemy offset w sterowniku, zeby mierzyc wartosc bezwzgledna,
     * a nie poprawke do poprzedniej. */
    flora_icm20948_set_gyro_bias(s_f.imu, zero);
    ESP_LOGW(TAG, "kalibracja offsetu: NIE RUSZAJ glowica przez 2 s");
}

/**
 * Jeden krok kalibracji. Zwraca true, gdy skonczona (udana albo nie).
 *
 * Bezruch sprawdzamy w TRAKCIE, nie tylko na wejsciu: szarpniecie w polowie
 * pomiaru przesunelo by offset, a bledny offset daje narastajacy blad kursu
 * rzedu kilku stopni na minute - czyli gorzej niz brak kalibracji.
 */
static bool cal_step(const flora_icm20948_data_t *d)
{
    for (int i = 0; i < 3; i++) {
        float r = fabsf(d->gyro_dps[i]);
        if (r > s_f.cal_peak_rate) {
            s_f.cal_peak_rate = r;
        }
        s_f.cal_sum[i] += d->gyro_dps[i];
    }
    float g_dev = fabsf(s_f.att.magnitude_g - 1.0f);

    if (s_f.cal_peak_rate > CAL_MAX_RATE_DPS || g_dev > CAL_MAX_G_DEV) {
        s_f.cal_attempts++;
        ESP_LOGW(TAG, "kalibracja przerwana - ruch w trakcie "
                      "(zyro do %.2f dps, wypadkowa %.3f g), proba %d z %d",
                 s_f.cal_peak_rate, s_f.att.magnitude_g,
                 s_f.cal_attempts, CAL_MAX_ATTEMPTS);
        return true;
    }
    if (++s_f.cal_n < CAL_SAMPLES) {
        return false;
    }

    float b[3];
    for (int i = 0; i < 3; i++) {
        b[i] = (float)(s_f.cal_sum[i] / CAL_SAMPLES);
    }
    flora_icm20948_set_gyro_bias(s_f.imu, b);
    ESP_LOGI(TAG, "offset wyznaczony: %.3f %.3f %.3f dps", b[0], b[1], b[2]);
    bias_save(b);
    s_f.cal_attempts = 0;
    return true;
}

/* ================= KALIBRACJA MAGNETOMETRU ================= */

static void magcal_load(void)
{
    flora_magcal_identity(&s_f.mag_cal);
    s_f.mag_cal_ok = false;
    bool ok = false;
    if (s_f.store) {
        flora_store_get_bool(s_f.store, FLORA_HEAD_NVS_MAG_OK, &ok, false);
    }
    if (ok) {
        flora_magcal_t *c = &s_f.mag_cal;
        flora_store_get_float(s_f.store, FLORA_HEAD_NVS_MAG_OX,   &c->ox, 0.0f);
        flora_store_get_float(s_f.store, FLORA_HEAD_NVS_MAG_OY,   &c->oy, 0.0f);
        flora_store_get_float(s_f.store, FLORA_HEAD_NVS_MAG_OZ,   &c->oz, 0.0f);
        flora_store_get_float(s_f.store, FLORA_HEAD_NVS_MAG_W00,  &c->w[0][0], 1.0f);
        flora_store_get_float(s_f.store, FLORA_HEAD_NVS_MAG_W01,  &c->w[0][1], 0.0f);
        flora_store_get_float(s_f.store, FLORA_HEAD_NVS_MAG_W11,  &c->w[1][1], 1.0f);
        flora_store_get_float(s_f.store, FLORA_HEAD_NVS_MAG_HREF, &c->h_ref_ut, 0.0f);
        flora_store_get_float(s_f.store, FLORA_HEAD_NVS_MAG_DIP,  &c->dip_ref_deg, 0.0f);
        c->w[1][0] = c->w[0][1];
        s_f.mag_cal_ok = true;
        ESP_LOGI(TAG, "kalibracja magnetometru z NVS: hard iron (%.2f, %.2f, %.2f) uT, "
                      "pole poziome %.1f uT, inklinacja %.1f st",
                 c->ox, c->oy, c->oz, c->h_ref_ut, c->dip_ref_deg);
    } else {
        ESP_LOGW(TAG, "magnetometr BEZ kalibracji - kurs moze miec bledy zalezne od kierunku; "
                      "uruchom cmd/mag_cal");
    }
    flora_magdist_set_ref(&s_f.dist, s_f.mag_cal_ok,
                          s_f.mag_cal.h_ref_ut, s_f.mag_cal.dip_ref_deg);
}

static void magcal_save(const flora_magcal_t *c)
{
    if (!s_f.store) {
        ESP_LOGW(TAG, "brak NVS - kalibracja magnetometru tylko do restartu");
        return;
    }
    flora_store_set_float(s_f.store, FLORA_HEAD_NVS_MAG_OX,   c->ox);
    flora_store_set_float(s_f.store, FLORA_HEAD_NVS_MAG_OY,   c->oy);
    flora_store_set_float(s_f.store, FLORA_HEAD_NVS_MAG_OZ,   c->oz);
    flora_store_set_float(s_f.store, FLORA_HEAD_NVS_MAG_W00,  c->w[0][0]);
    flora_store_set_float(s_f.store, FLORA_HEAD_NVS_MAG_W01,  c->w[0][1]);
    flora_store_set_float(s_f.store, FLORA_HEAD_NVS_MAG_W11,  c->w[1][1]);
    flora_store_set_float(s_f.store, FLORA_HEAD_NVS_MAG_HREF, c->h_ref_ut);
    flora_store_set_float(s_f.store, FLORA_HEAD_NVS_MAG_DIP,  c->dip_ref_deg);
    flora_store_set_bool(s_f.store, FLORA_HEAD_NVS_MAG_OK, true);
}

static void magcal_start(uint32_t seconds)
{
    flora_magcal_begin(&s_collect);
    s_f.collecting     = true;
    s_f.collect_end_us = esp_timer_get_time() + (int64_t)seconds * 1000000;
    s_f.mag_valid      = false;
    ESP_LOGW(TAG, "kalibracja magnetometru: obracaj glowica wokol pionu przez %" PRIu32
                  " s (2-3 pelne obroty)", seconds);
}

static void magcal_finish(void)
{
    s_f.collecting = false;
    flora_magcal_t cal = s_f.mag_cal;
    flora_magcal_report_t rep = flora_magcal_fit(&s_collect, FLORA_HEAD_MAG_INCL_DEG, &cal);

    if (rep.status == FLORA_MAGCAL_OK) {
        s_f.mag_cal    = cal;
        s_f.mag_cal_ok = true;
        magcal_save(&cal);
        flora_magdist_set_ref(&s_f.dist, true, cal.h_ref_ut, cal.dip_ref_deg);
        ESP_LOGI(TAG, "kalibracja magnetometru OK: %u probek, %u/12 sektorow, osie %.2f, "
                      "rozrzut %.1f%%, hard iron (%.2f, %.2f, %.2f) uT",
                 rep.n, rep.sectors, rep.axis_ratio, rep.rms_pct, cal.ox, cal.oy, cal.oz);
    } else {
        ESP_LOGE(TAG, "kalibracja magnetometru NIEUDANA: %s (%u probek, %u/12 sektorow, "
                      "osie %.2f, rozrzut %.1f%%) - zostaje poprzednia",
                 flora_magcal_status_name(rep.status), rep.n, rep.sectors,
                 rep.axis_ratio, rep.rms_pct);
    }

    flora_mag_cal_result_t m = {
        .status           = (uint8_t)rep.status,
        .sectors          = rep.sectors,
        .samples          = rep.n,
        .offset_x_cut     = (int16_t)lroundf(cal.ox * 100.0f),
        .offset_y_cut     = (int16_t)lroundf(cal.oy * 100.0f),
        .axis_ratio_milli = (uint16_t)lroundf(fminf(rep.axis_ratio, 65.0f) * 1000.0f),
        .rms_cpct         = (uint16_t)lroundf(fminf(rep.rms_pct, 650.0f) * 100.0f),
        .field_h_cut      = (uint16_t)lroundf(cal.h_ref_ut * 100.0f),
        .dip_cdeg         = (int16_t)lroundf(cal.dip_ref_deg * 100.0f),
        .offset_z_cut     = (int16_t)lroundf(fmaxf(-327.0f, fminf(327.0f, cal.oz)) * 100.0f),
    };
    /* Wynik jest jednorazowy i wazny - z potwierdzeniem. Blokuje do
     * kilkuset ms, ale to raz na kalibracje. */
    if (flora_link_send_reliable(s_f.down, FLORA_MSG_MAG_CAL_RESULT, &m, sizeof m, 200) != ESP_OK) {
        ESP_LOGW(TAG, "wynik kalibracji nie dotarl do bryly");
    }
}

/* ================= POZIOM AKCELEROMETRU =================
 *
 * Plytka nigdy nie lezy idealnie poziomo na maszcie, a staly blad
 * pochylenia i przechylu wchodzi w kompensacje przechylu magnetometru.
 * Przy inklinacji 66 st (Krakow) mnozy sie przez tan(66) ~ 2,2: plytka
 * przekrzywiona o 2 st daje do +-4,5 st bledu kursu, zaleznie od kierunku.
 *
 * Pomiar w jednej pozycji widzi sume: przekrzywienie plytki + pochylosc
 * podloza. Po obrocie robota o 180 st pochylosc podloza zmienia znak
 * w ukladzie robota, a przekrzywienie plytki nie - srednia z dwoch
 * pozycji daje samo przekrzywienie (dla malych katow). */

static void level_load(void)
{
    bool ok = false;
    if (s_f.store) {
        flora_store_get_bool(s_f.store, FLORA_HEAD_NVS_LVL_OK, &ok, false);
    }
    if (ok) {
        flora_store_get_float(s_f.store, FLORA_HEAD_NVS_LVL_PITCH, &s_f.lvl_off_pitch, 0.0f);
        flora_store_get_float(s_f.store, FLORA_HEAD_NVS_LVL_ROLL,  &s_f.lvl_off_roll,  0.0f);
        ESP_LOGI(TAG, "poziom z NVS: pochylenie %.2f st, przechyl %.2f st",
                 s_f.lvl_off_pitch, s_f.lvl_off_roll);
    } else {
        ESP_LOGW(TAG, "poziom akcelerometru BEZ kalibracji - uruchom cmd/level_cal");
    }
}

static void level_send(uint8_t status, float pitch, float roll)
{
    flora_level_cal_result_t m = {
        .status         = status,
        .step           = s_f.lvl_step,
        .pitch_cdeg     = (int16_t)lroundf(pitch * 100.0f),
        .roll_cdeg      = (int16_t)lroundf(roll * 100.0f),
        .off_pitch_cdeg = (int16_t)lroundf(s_f.lvl_off_pitch * 100.0f),
        .off_roll_cdeg  = (int16_t)lroundf(s_f.lvl_off_roll * 100.0f),
    };
    if (flora_link_send_reliable(s_f.down, FLORA_MSG_LEVEL_CAL_RESULT, &m, sizeof m, 200) != ESP_OK) {
        ESP_LOGW(TAG, "wynik kalibracji poziomu nie dotarl do bryly");
    }
}

static void level_start(uint8_t step)
{
    s_f.lvl_measuring = true;
    s_f.lvl_step      = step;
    s_f.lvl_n         = 0;
    s_f.lvl_peak_rate = 0.0f;
    memset(s_f.lvl_sum, 0, sizeof(s_f.lvl_sum));
    ESP_LOGW(TAG, "kalibracja poziomu, krok %u: NIE RUSZAJ robota przez 2 s", step);
}

/** Jeden tick pomiaru poziomu na SUROWYM akcelerometrze (bez offsetu). */
static void level_step(const flora_icm20948_data_t *d)
{
    for (int i = 0; i < 3; i++) {
        float r = fabsf(d->gyro_dps[i]);
        if (r > s_f.lvl_peak_rate) s_f.lvl_peak_rate = r;
        s_f.lvl_sum[i] += d->accel_g[i];
    }
    float g = sqrtf(d->accel_g[0] * d->accel_g[0] + d->accel_g[1] * d->accel_g[1] +
                    d->accel_g[2] * d->accel_g[2]);
    if (s_f.lvl_peak_rate > CAL_MAX_RATE_DPS || fabsf(g - 1.0f) > CAL_MAX_G_DEV) {
        s_f.lvl_measuring = false;
        ESP_LOGW(TAG, "kalibracja poziomu przerwana - ruch w trakcie");
        level_send(FLORA_LEVEL_CAL_MOVED, 0.0f, 0.0f);
        return;
    }
    if (++s_f.lvl_n < CAL_SAMPLES) {
        return;
    }
    s_f.lvl_measuring = false;

    float a[3];
    for (int i = 0; i < 3; i++) a[i] = (float)(s_f.lvl_sum[i] / CAL_SAMPLES);
    head_attitude_t raw = head_attitude_from_accel(a);

    if (s_f.lvl_step == 1) {
        s_f.lvl_have_first  = true;
        s_f.lvl_first_pitch = raw.pitch_deg;
        s_f.lvl_first_roll  = raw.roll_deg;
        ESP_LOGI(TAG, "poziom krok 1: %.2f / %.2f st - obroc robota o 180 st i wykonaj krok 2",
                 raw.pitch_deg, raw.roll_deg);
        level_send(FLORA_LEVEL_CAL_FIRST_OK, raw.pitch_deg, raw.roll_deg);
        return;
    }
    if (s_f.lvl_step == 2 && !s_f.lvl_have_first) {
        level_send(FLORA_LEVEL_CAL_NO_FIRST, raw.pitch_deg, raw.roll_deg);
        return;
    }

    if (s_f.lvl_step == 2) {
        s_f.lvl_off_pitch = 0.5f * (s_f.lvl_first_pitch + raw.pitch_deg);
        s_f.lvl_off_roll  = 0.5f * (s_f.lvl_first_roll  + raw.roll_deg);
        ESP_LOGI(TAG, "pochylosc podloza: %.2f / %.2f st (odjeta)",
                 0.5f * (s_f.lvl_first_pitch - raw.pitch_deg),
                 0.5f * (s_f.lvl_first_roll  - raw.roll_deg));
        s_f.lvl_have_first = false;
    } else {
        /* Krok 0: zakladamy plaskie podloze - caly odczyt to przekrzywienie. */
        s_f.lvl_off_pitch = raw.pitch_deg;
        s_f.lvl_off_roll  = raw.roll_deg;
    }
    if (s_f.store) {
        flora_store_set_float(s_f.store, FLORA_HEAD_NVS_LVL_PITCH, s_f.lvl_off_pitch);
        flora_store_set_float(s_f.store, FLORA_HEAD_NVS_LVL_ROLL,  s_f.lvl_off_roll);
        flora_store_set_bool(s_f.store, FLORA_HEAD_NVS_LVL_OK, true);
    }
    ESP_LOGI(TAG, "poziom zapisany: pochylenie %.2f st, przechyl %.2f st",
             s_f.lvl_off_pitch, s_f.lvl_off_roll);
    level_send(FLORA_LEVEL_CAL_OK, raw.pitch_deg, raw.roll_deg);
}

/* ================= MAGNETOMETR ================= */

/** Odczyt magnetometru i korekta Kalmanem. Wolane co MAG_DIVIDER tickow. */
static void mag_step(void)
{
    flora_icm20948_mag_t mg;
    esp_err_t r = flora_icm20948_read_mag(s_f.imu, &mg);
    if (r == ESP_ERR_NOT_FINISHED) {
        return;   /* probka jeszcze nie gotowa - to nie blad */
    }
    if (r != ESP_OK || mg.overflow) {
        s_f.mag_valid = false;
        return;
    }
    s_f.mag_seen = true;

    /* Przechyl PLYTKI wzgledem grawitacji (tilt_raw), nie robota: pole
     * mierzy plytka, wiec do poziomu obraca sie wektor w jej osiach.
     * Offset z level_cal (przekrzywienie plytki na maszcie) jest dla
     * przechylu robota - tu wprowadzilby blad rowny temu przekrzywieniu. */
    const float pitch = s_f.tilt_raw.pitch_deg, roll = s_f.tilt_raw.roll_deg;

    if (s_f.collecting) {
        if (!flora_magcal_add(&s_collect, mg.field_ut, pitch, roll)) {
            s_f.collect_end_us = 0;   /* bufor pelny - konczymy wczesniej */
        }
        s_f.mag_valid = false;    /* surowe probki nie ida do kursu */
        return;
    }

    /* Bez kalibracji korekta tozsamosciowa - samo poziomowanie. */
    flora_magcal_out_t mo;
    flora_magcal_apply(&s_f.mag_cal, mg.field_ut, pitch, roll, &mo);
    s_f.mag_h       = mo.h_ut;
    s_f.mag_dip     = mo.dip_deg;
    s_f.mag_total   = mo.total_ut;
    s_f.mag_heading = mo.heading_deg;

    bool tilted = fabsf(s_f.att.roll_deg) > TILT_LIMIT_DEG ||
                  fabsf(s_f.att.pitch_deg) > TILT_LIMIT_DEG ||
                  fabsf(s_f.att.magnitude_g - 1.0f) > MAG_MAX_G_DEV;
    float sigma = tilted ? MAG_SIGMA_TILTED_DEG : MAG_SIGMA_DEG;

    bool agrees = s_f.state != HEAD_FUSION_RUN ||
                  flora_hkf_would_accept(&s_f.kf, s_f.mag_heading, sigma);
    bool disturbed = flora_magdist_step(&s_f.dist, MAG_PERIOD_MS, s_f.mag_h, s_f.mag_dip,
                                        s_f.mag_total, agrees);
    s_f.mag_valid = !disturbed;

    if (s_f.state == HEAD_FUSION_RUN && !disturbed) {
        flora_hkf_update(&s_f.kf, s_f.mag_heading, sigma);
    }
}

/* ================= TELEMETRIA ================= */

/**
 * Tempo zmiany kursu kompasowego [st/s], dodatnie W PRAWO (CW).
 *
 * Osie IMU: Z w gore, wiec dodatnie omega_z to obrot W LEWO (CCW),
 * a kurs z magnetometru rosnie w prawo. Bez tej zamiany zyroskop
 * i magnetometr ciagna kurs w przeciwne strony.
 */
static inline float heading_rate_dps(const flora_icm20948_data_t *d)
{
    return -d->gyro_dps[2];
}

/**
 * ZUPT: na postoju srednie tempo kursu z zyroskopu to sam dryf.
 * Postoj mowi bryla (enkodery), a glowica sprawdza, czy sama sie nie
 * obraca - przy recznej kalibracji ktos moze krecic zdjeta glowica,
 * gdy kola stoja. Wolane w RUN, po predykcji.
 */
static void zupt_step(const flora_icm20948_data_t *d)
{
    float r = heading_rate_dps(d);
    bool quiet = head_cmd_hub_still() && !s_f.collecting &&
                 fabsf(d->gyro_dps[0]) < ZUPT_MAX_RATE_DPS &&
                 fabsf(d->gyro_dps[1]) < ZUPT_MAX_RATE_DPS &&
                 fabsf(r) < ZUPT_MAX_RATE_DPS &&
                 fabsf(s_f.att.magnitude_g - 1.0f) < ZUPT_MAX_G_DEV;
    if (!quiet) {
        s_f.zupt_n   = 0;
        s_f.zupt_sum = 0.0;
        return;
    }
    s_f.zupt_sum += r;
    if (++s_f.zupt_n < ZUPT_SAMPLES) {
        return;
    }
    float z = (float)(s_f.zupt_sum / s_f.zupt_n);
    if (flora_hkf_update_bias(&s_f.kf, z, ZUPT_SIGMA_DPS) && s_f.zupt_count++ == 0) {
        ESP_LOGI(TAG, "ZUPT: pierwszy pomiar dryfu na postoju, %.3f st/s", z);
    }
    s_f.zupt_n   = 0;
    s_f.zupt_sum = 0.0;
}

static float heading_sigma(void)
{
    return s_f.state == HEAD_FUSION_RUN ? flora_hkf_sigma(&s_f.kf) : 0.0f;
}

static void send_attitude(int64_t sample_us, const flora_icm20948_data_t *d)
{
    uint8_t flags = 0;
    if (s_f.heading_valid)                      flags |= FLORA_ATT_HEADING_VALID;
    if (s_f.mag_valid)                          flags |= FLORA_ATT_MAG_VALID;
    if (s_f.dist.disturbed)                     flags |= FLORA_ATT_MAG_DISTURBED;
    if (head_cmd_time_synced())                 flags |= FLORA_ATT_TIME_SYNCED;
    if (fabsf(s_f.att.roll_deg)  > TILT_LIMIT_DEG ||
        fabsf(s_f.att.pitch_deg) > TILT_LIMIT_DEG) flags |= FLORA_ATT_TILTED;

    /* quality zostaje dla zgodnosci (panel, starsze narzedzia): liczona
     * z sigma tak, by 2 st dawaly ~90, a 10 st ~50, jak dawniej. */
    float sig = heading_sigma();
    uint8_t quality = 0;
    if (s_f.state == HEAD_FUSION_RUN) {
        float q = 100.0f - 5.0f * sig;
        quality = (uint8_t)(q < 0.0f ? 0.0f : q);
    }

    flora_attitude_t m = {
        .timestamp_us  = head_cmd_stamp(sample_us),
        .heading_cdeg  = (uint16_t)lroundf(head_wrap360(s_f.kf.psi) * 100.0f) % 36000,
        .rate_z_cdps   = (int16_t)lroundf(heading_rate_dps(d) * 100.0f),
        .pitch_cdeg    = (int16_t)lroundf(s_f.att.pitch_deg * 100.0f),
        .roll_cdeg     = (int16_t)lroundf(s_f.att.roll_deg * 100.0f),
        .quality       = quality,
        .flags         = flags,
        .heading_sigma_cdeg = (uint16_t)lroundf(fminf(sig, 650.0f) * 100.0f),
    };
    flora_link_send(s_f.down, FLORA_MSG_ATTITUDE, &m, sizeof m);
}

static void send_mag_info(void)
{
    uint8_t flags = 0;
    if (s_f.mag_cal_ok)     flags |= FLORA_MAG_CAL_OK;
    if (s_f.dist.disturbed) flags |= FLORA_MAG_DISTURBED;
    if (s_f.collecting)     flags |= FLORA_MAG_COLLECTING;

    float bias_mdps = s_f.kf.bz * 1000.0f;
    if (bias_mdps > 32767.0f)  bias_mdps = 32767.0f;
    if (bias_mdps < -32767.0f) bias_mdps = -32767.0f;

    flora_mag_info_t m = {
        .timestamp_us       = head_cmd_stamp(esp_timer_get_time()),
        .field_h_cut        = (uint16_t)lroundf(fminf(s_f.mag_h, 650.0f) * 100.0f),
        .dip_cdeg           = (int16_t)lroundf(s_f.mag_dip * 100.0f),
        .field_total_cut    = (uint16_t)lroundf(fminf(s_f.mag_total, 650.0f) * 100.0f),
        .gyro_bias_mdps     = (int16_t)lroundf(bias_mdps),
        .heading_sigma_cdeg = (uint16_t)lroundf(fminf(heading_sigma(), 650.0f) * 100.0f),
        .flags              = flags,
        .cause              = s_f.dist.cause,
    };
    flora_link_send(s_f.down, FLORA_MSG_MAG_INFO, &m, sizeof m);
}

/* ================= PETLA ================= */

/* Takt z esp_timer, nie z vTaskDelayUntil: zegar FreeRTOS ma 100 Hz, wiec
 * pdMS_TO_TICKS(16) to 1 takt = 10 ms i petla szla 100 Hz zamiast 60.
 * Timer daje dokladnie 1/60 s niezaleznie od CONFIG_FREERTOS_HZ. */
static void loop_tick(void *arg)
{
    xTaskNotifyGive((TaskHandle_t)arg);
}

static void fusion_task(void *arg)
{
    s_f.last_us = esp_timer_get_time();

    esp_timer_handle_t timer;
    const esp_timer_create_args_t targs = {
        .callback = loop_tick,
        .arg      = xTaskGetCurrentTaskHandle(),
        .name     = "fusion_tick",
    };
    ESP_ERROR_CHECK(esp_timer_create(&targs, &timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer, 1000000 / LOOP_HZ));

    while (1) {
        /* pdTRUE: przy spoznieniu zbiera zalegle takty w jeden - petla
         * nie nadrabia seria, tylko liczy dalej z prawdziwym dt. */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        s_f.tick++;

        /* Komendy zmieniajace tempo obslugujemy niezaleznie od stanu. */
        uint8_t hz;
        if (head_cmd_take_att_hz(&hz)) {
            uint8_t div = (uint8_t)(LOOP_HZ / hz);
            s_f.att_divider = div ? div : 1;
            ESP_LOGI(TAG, "ATTITUDE co %u tick = %u Hz",
                     s_f.att_divider, LOOP_HZ / s_f.att_divider);
        }

        /* --- stany bez odczytu IMU --- */
        if (s_f.state == HEAD_FUSION_INIT) {
            if (s_f.imu && in_state_ms() > 200) {
                go(HEAD_FUSION_BIAS_LOAD);
            }
            continue;
        }
        if (s_f.state == HEAD_FUSION_BIAS_LOAD) {
            go(bias_load() ? HEAD_FUSION_ALIGN : HEAD_FUSION_BIAS_CAL);
            if (s_f.state == HEAD_FUSION_BIAS_CAL) {
                cal_begin();
            }
            continue;
        }

        /* --- odczyt IMU --- */
        flora_icm20948_data_t d;
        if (flora_icm20948_read(s_f.imu, &d) != ESP_OK) {
            s_f.errors++;
            if (++s_f.fail_streak >= FAULT_THRESHOLD && s_f.state != HEAD_FUSION_FAULT) {
                ESP_LOGE(TAG, "IMU nie odpowiada (%" PRIu32 " bledow z rzedu)",
                         s_f.fail_streak);
                s_f.heading_valid = false;
                go(HEAD_FUSION_FAULT);
            }
            /* W FAULT nadal nadajemy, zeby bryla odroznila "glowica nie wie"
             * od "glowica nie zyje" - to drugie poznaje po braku ramek. */
            if (s_f.state == HEAD_FUSION_FAULT && s_f.tick % s_f.att_divider == 0) {
                flora_icm20948_data_t zero = { 0 };
                send_attitude(esp_timer_get_time(), &zero);
            }
            continue;
        }
        int64_t sample_us = esp_timer_get_time();
        s_f.last = d;
        if (s_f.fail_streak >= FAULT_THRESHOLD) {
            ESP_LOGI(TAG, "IMU znowu odpowiada");
        }
        s_f.fail_streak = 0;
        if (s_f.state == HEAD_FUSION_FAULT) {
            go(HEAD_FUSION_ALIGN);
        }

        /* Pochylenie i przechyl: zyroskop + powolna korekta z akcelerometru.
         * Po przerwie (FAULT, start) od nowa z samego akcelerometru. */
        float tdt = s_f.tilt_init ? (float)(sample_us - s_f.tilt_last_us) / 1e6f : 0.0f;
        if (tdt > 0.0f && tdt < TILT_MAX_GAP_S) {
            s_f.tilt_raw = head_tilt_fuse(s_f.tilt_raw, d.gyro_dps, d.accel_g, tdt,
                                          TILT_TAU_S, TILT_MAX_G_DEV);
        } else {
            s_f.tilt_raw = head_attitude_from_accel(d.accel_g);
        }
        s_f.tilt_init    = true;
        s_f.tilt_last_us = sample_us;

        s_f.att = s_f.tilt_raw;
        s_f.att.pitch_deg -= s_f.lvl_off_pitch;   /* przekrzywienie plytki */
        s_f.att.roll_deg  -= s_f.lvl_off_roll;

        /* --- kalibracja zyro --- */
        if (s_f.state == HEAD_FUSION_BIAS_CAL) {
            if (cal_step(&d)) {
                if (s_f.cal_attempts >= CAL_MAX_ATTEMPTS) {
                    ESP_LOGE(TAG, "kalibracja nie udala sie %d razy - "
                                  "pracuje bez offsetu, kurs bedzie dryfowal",
                             CAL_MAX_ATTEMPTS);
                    s_f.cal_attempts = 0;
                    go(HEAD_FUSION_ALIGN);
                } else if (s_f.cal_n >= CAL_SAMPLES) {
                    go(HEAD_FUSION_ALIGN);
                } else {
                    cal_begin();   /* kolejna proba */
                }
            }
            continue;
        }

        /* --- magnetometr (10 Hz) --- */
        if (s_f.tick % MAG_DIVIDER == 0 && flora_icm20948_has_magnetometer(s_f.imu)) {
            mag_step();
        }

        /* --- ALIGN: pierwszy kurs bezwzgledny --- */
        if (s_f.state == HEAD_FUSION_ALIGN) {
            flora_hkf_cfg_t kcfg = FLORA_HKF_DEFAULT_CFG();
            if (s_f.mag_valid) {
                flora_hkf_init(&s_f.kf, &kcfg, s_f.mag_heading, MAG_SIGMA_DEG);
                s_f.heading_valid = true;
                ESP_LOGI(TAG, "kurs ustawiony na %.1f st z magnetometru", s_f.mag_heading);
                go(HEAD_FUSION_RUN);
            } else if (in_state_ms() > 2000) {
                /* Bez magnetometru kurs jest tylko wzgledny wobec startu.
                 * Bryla rozpozna to po MAG_VALID = 0 i po ogromnej sigma. */
                ESP_LOGW(TAG, "brak magnetometru - kurs bedzie wzgledny");
                flora_hkf_init(&s_f.kf, &kcfg, 0.0f, NOMAG_SIGMA0_DEG);
                s_f.heading_valid = true;
                go(HEAD_FUSION_RUN);
            } else {
                continue;
            }
            s_f.last_us = sample_us;
        }

        /* --- RUN: predykcja z zyroskopu --- */
        float dt_s = (float)(sample_us - s_f.last_us) / 1000000.0f;
        s_f.last_us = sample_us;
        if (dt_s > 0.0f && dt_s < 0.5f) {
            flora_hkf_predict(&s_f.kf, heading_rate_dps(&d), dt_s);
        }
        zupt_step(&d);

        /* --- komendy kalibracji --- */
        uint8_t lstep;
        if (!s_f.lvl_measuring && head_cmd_take_level_cal(&lstep)) {
            level_start(lstep);
        }
        if (s_f.lvl_measuring) {
            level_step(&d);
        }
        uint32_t secs;
        if (head_cmd_take_mag_cal(&secs)) {
            magcal_start(secs);
        }
        if (s_f.collecting && sample_us >= s_f.collect_end_us) {
            magcal_finish();
        }
        if (head_cmd_take_gyro_cal()) {
            s_f.cal_attempts = 0;
            go(HEAD_FUSION_BIAS_CAL);
            cal_begin();
            continue;
        }

        if (s_f.tick % s_f.att_divider == 0) {
            send_attitude(sample_us, &d);
        }
        if (s_f.tick % INFO_DIVIDER == 0 && s_f.mag_seen) {
            send_mag_info();
        }
    }
}

esp_err_t head_fusion_start(flora_icm20948_handle_t imu, flora_link_handle_t down)
{
    ESP_RETURN_ON_FALSE(down, ESP_ERR_INVALID_ARG, TAG, "brak lacza");

    memset(&s_f, 0, sizeof(s_f));
    s_f.imu         = imu;
    s_f.down        = down;
    s_f.att_divider = ATT_DIVIDER;
    s_f.state       = HEAD_FUSION_INIT;
    s_f.state_entered_us = esp_timer_get_time();

    if (flora_store_open(FLORA_HEAD_NVS_NS, &s_f.store) != ESP_OK) {
        ESP_LOGW(TAG, "NVS niedostepne - offset zyro i kalibracja magnetometru nie beda zapamietywane");
        s_f.store = NULL;
    }

    flora_magdist_cfg_t dcfg = FLORA_MAGDIST_DEFAULT_CFG();
    flora_magdist_init(&s_f.dist, &dcfg);
    magcal_load();
    level_load();

    ESP_RETURN_ON_FALSE(xTaskCreate(fusion_task, "head_fusion", 4096, NULL, 10, NULL) == pdPASS,
                        ESP_ERR_NO_MEM, TAG, "nie udalo sie utworzyc zadania");
    ESP_LOGI(TAG, "zadanie %d Hz, ATTITUDE co %d tick = %d Hz, Kalman kursu [psi, dryf]",
             LOOP_HZ, ATT_DIVIDER, LOOP_HZ / ATT_DIVIDER);
    return ESP_OK;
}

head_fusion_state_t head_fusion_state(void)  { return s_f.state; }
float    head_fusion_heading(void)           { return s_f.kf.psi; }
bool     head_fusion_heading_valid(void)     { return s_f.heading_valid; }
uint32_t head_fusion_errors(void)            { return s_f.errors; }

void head_fusion_debug(float accel_g[3], float gyro_dps[3], float *pitch_deg,
                       float *roll_deg)
{
    /* Bez blokady: czytamy z innego zadania i moze trafic sie mieszanka
     * dwoch kolejnych probek. Do podgladu w logu to bez znaczenia. */
    for (int i = 0; i < 3; i++) {
        accel_g[i]  = s_f.last.accel_g[i];
        gyro_dps[i] = s_f.last.gyro_dps[i];
    }
    *pitch_deg = s_f.att.pitch_deg;
    *roll_deg  = s_f.att.roll_deg;
}
