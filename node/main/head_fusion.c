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
#include "flora_msgs.h"
#include "flora_store.h"

static const char *TAG = "fusion";

/* Tempo petli. 60 Hz daje dokladnie 20 Hz ATTITUDE po podziale na 3,
 * a blad calkowania zyroskopu to 0,5 stopnia na probke przy obrocie
 * 30 st/s - pomijalny. */
#define LOOP_HZ        60
#define LOOP_MS        (1000 / LOOP_HZ)
#define MAG_DIVIDER    6     /**< magnetometr 10 Hz; korekta dryfu nie musi byc szybsza */
#define ATT_DIVIDER    3     /**< ATTITUDE 20 Hz, zgodnie z kontraktem */

/* Po tylu nieudanych odczytach z rzedu uznajemy IMU za martwy. Jeden
 * zgubiony odczyt to normalna kolizja na magistrali, nie awaria. */
#define FAULT_THRESHOLD 10

/* Stala czasowa korekty magnetometrem. Kilka sekund: dosc wolno, by szum
 * magnetometru nie trafial do kursu, dosc szybko, by dryf zyroskopu nie
 * narastal miedzy korektami. */
#define FUSION_TAU_S   4.0f

/* Przechyl, powyzej ktorego kurs przestaje byc wiarygodny i wedruje
 * flaga TILTED. Korekta dziala dalej, ale blad rosnie nieliniowo. */
#define TILT_LIMIT_DEG 35.0f

/* --- kalibracja offsetu --- */
#define CAL_SAMPLES        120    /**< 2 s przy 60 Hz */
#define CAL_MAX_RATE_DPS   2.0f   /**< powyzej tego uznajemy ruch */
#define CAL_MAX_G_DEV      0.08f  /**< odstepstwo wypadkowej od 1 g */
#define CAL_MAX_ATTEMPTS   3

static struct {
    flora_icm20948_handle_t imu;
    flora_link_handle_t     down;
    flora_store_t           store;

    head_fusion_state_t state;
    uint32_t  tick;
    int64_t   last_us;

    float     heading;
    float     mag_heading;
    bool      mag_valid;
    head_attitude_t att;

    uint8_t   att_divider;
    uint32_t  fail_streak;
    uint32_t  errors;
    bool      heading_valid;

    /* kalibracja */
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

/* ================= TELEMETRIA ================= */

static void send_attitude(int64_t sample_us, const flora_icm20948_data_t *d)
{
    uint8_t flags = 0;
    if (s_f.heading_valid)                      flags |= FLORA_ATT_HEADING_VALID;
    if (s_f.mag_valid)                          flags |= FLORA_ATT_MAG_VALID;
    if (fabsf(s_f.att.roll_deg)  > TILT_LIMIT_DEG ||
        fabsf(s_f.att.pitch_deg) > TILT_LIMIT_DEG) flags |= FLORA_ATT_TILTED;

    uint8_t quality = 0;
    if (s_f.state == HEAD_FUSION_RUN) {
        /* Jakosc spada przy braku magnetometru i przy duzym przechyle -
         * bryla ma po czym wazyc ten pomiar w filtrze. */
        quality = s_f.mag_valid ? 90 : 50;
        if (flags & FLORA_ATT_TILTED) {
            quality /= 2;
        }
    }

    flora_attitude_t m = {
        .timestamp_us  = (uint32_t)sample_us,
        .heading_cdeg  = (uint16_t)lroundf(head_wrap360(s_f.heading) * 100.0f),
        .rate_z_cdps   = (int16_t)lroundf(d->gyro_dps[2] * 100.0f),
        .pitch_cdeg    = (int16_t)lroundf(s_f.att.pitch_deg * 100.0f),
        .roll_cdeg     = (int16_t)lroundf(s_f.att.roll_deg * 100.0f),
        .quality       = quality,
        .flags         = flags,
    };
    flora_link_send(s_f.down, FLORA_MSG_ATTITUDE, &m, sizeof m);
}

/* ================= PETLA ================= */

static void fusion_task(void *arg)
{
    TickType_t next = xTaskGetTickCount();
    s_f.last_us = esp_timer_get_time();

    while (1) {
        vTaskDelayUntil(&next, pdMS_TO_TICKS(LOOP_MS));
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
        if (s_f.fail_streak >= FAULT_THRESHOLD) {
            ESP_LOGI(TAG, "IMU znowu odpowiada");
        }
        s_f.fail_streak = 0;
        if (s_f.state == HEAD_FUSION_FAULT) {
            go(HEAD_FUSION_ALIGN);
        }

        s_f.att = head_attitude_from_accel(d.accel_g);

        /* --- kalibracja --- */
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

        /* --- magnetometr --- */
        if (s_f.tick % MAG_DIVIDER == 0 && flora_icm20948_has_magnetometer(s_f.imu)) {
            flora_icm20948_mag_t mg;
            esp_err_t r = flora_icm20948_read_mag(s_f.imu, &mg);
            if (r == ESP_OK && !mg.overflow && head_mag_plausible(mg.field_ut)) {
                s_f.mag_heading = head_heading_tilt_compensated(mg.field_ut, s_f.att);
                s_f.mag_valid   = true;
            } else if (r != ESP_ERR_NOT_FINISHED) {
                /* NOT_FINISHED znaczy tylko "probka jeszcze nie gotowa",
                 * czyli nie jest bledem - nie kasujemy po niej waznosci. */
                s_f.mag_valid = false;
            }
        }

        /* --- ALIGN: pierwszy kurs bezwzgledny --- */
        if (s_f.state == HEAD_FUSION_ALIGN) {
            if (s_f.mag_valid) {
                s_f.heading = s_f.mag_heading;
                s_f.heading_valid = true;
                ESP_LOGI(TAG, "kurs ustawiony na %.1f st z magnetometru", s_f.heading);
                go(HEAD_FUSION_RUN);
            } else if (in_state_ms() > 2000) {
                /* Bez magnetometru kurs jest tylko wzgledny wobec startu.
                 * Bryla rozpozna to po MAG_VALID = 0. */
                ESP_LOGW(TAG, "brak magnetometru - kurs bedzie wzgledny");
                s_f.heading = 0.0f;
                s_f.heading_valid = true;
                go(HEAD_FUSION_RUN);
            } else {
                continue;
            }
        }

        /* --- RUN: fuzja --- */
        float dt_s = (float)(sample_us - s_f.last_us) / 1000000.0f;
        s_f.last_us = sample_us;
        if (dt_s > 0.0f && dt_s < 0.5f) {
            s_f.heading = head_fuse_heading(s_f.heading, d.gyro_dps[2], dt_s,
                                            s_f.mag_heading, s_f.mag_valid,
                                            FUSION_TAU_S);
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
        ESP_LOGW(TAG, "NVS niedostepne - offset zyro nie bedzie zapamietywany");
        s_f.store = NULL;
    }

    ESP_RETURN_ON_FALSE(xTaskCreate(fusion_task, "head_fusion", 4096, NULL, 10, NULL) == pdPASS,
                        ESP_ERR_NO_MEM, TAG, "nie udalo sie utworzyc zadania");
    ESP_LOGI(TAG, "zadanie %d Hz, ATTITUDE co %d tick = %d Hz",
             LOOP_HZ, ATT_DIVIDER, LOOP_HZ / ATT_DIVIDER);
    return ESP_OK;
}

head_fusion_state_t head_fusion_state(void)  { return s_f.state; }
float    head_fusion_heading(void)           { return s_f.heading; }
bool     head_fusion_heading_valid(void)     { return s_f.heading_valid; }
uint32_t head_fusion_errors(void)            { return s_f.errors; }
