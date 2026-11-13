/**
 * @file hub_headcal.c
 * @brief Kalibracje glowicy, opis w hub_headcal.h.
 *
 * Kalibracja magnetometru trwa dziesiatki sekund, wiec idzie we wlasnym,
 * jednorazowym zadaniu - handler MQTT tylko je uruchamia i od razu wraca.
 */
#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "hub_drive.h"
#include "hub_headcal.h"
#include "hub_links.h"
#include "hub_params.h"
#include "flora_encoders.h"
#include "flora_mqtt.h"
#include "flora_board.h"
#include "flora_motors.h"
#include "flora_msgs.h"

static const char *TAG = "kal_glowicy";

#define ROT_DPS        20.0f    /**< predkosc obrotu w trybie auto */
#define ROT_PWM        30.0f    /**< moc obrotu, gdy naped w trybie pwm */
#define REFRESH_MS     200
#define RESULT_WAIT_MS 5000

static volatile bool s_running;

typedef enum { JOB_MAG, JOB_LEVEL_AUTO, JOB_TRACK, JOB_WHEEL_GO } job_kind_t;

typedef struct {
    job_kind_t kind;
    bool       auto_rotate;
    uint32_t   seconds;
    uint32_t   turns;     /**< JOB_TRACK */
    float      dist_mm;   /**< JOB_WHEEL_GO */
} job_t;

#define LEVEL_TURN_DEG     177.0f   /**< zatrzymanie z zapasem na wybieg */
#define LEVEL_TURN_MAX_MS  30000
#define LEVEL_SETTLE_MS    1500     /**< drgania masztu po zatrzymaniu */

static job_t s_job;

bool hub_headcal_running(void)
{
    return s_running;
}

static esp_err_t send_cmd(uint8_t code, uint32_t arg)
{
    flora_link_handle_t h = hub_links_handle();
    if (!h) {
        return ESP_ERR_INVALID_STATE;
    }
    flora_cmd_t c = { .code = code, .arg = arg };
    return flora_link_send_reliable(h, FLORA_MSG_CMD, &c, sizeof c, 500);
}

/** Obrot w miejscu: dir +1 w lewo (CCW), -1 w prawo; false gdy naped odmowil. */
static bool rotate_dir(int dir)
{
    if (hub_drive_ready() && hub_drive_mode() == HUB_DRIVE_MODE_PID) {
        hub_params_t p;
        hub_params_get(&p);
        float v = dir * 0.5f * p.track_mm * ROT_DPS * 3.14159265f / 180.0f;
        return hub_drive_set_speed(-v, v) == ESP_OK;
    }
    return flora_motors_set(-dir * ROT_PWM, dir * ROT_PWM) == ESP_OK;
}

static bool rotate(void)
{
    return rotate_dir(1);
}

/** Najkrotsza roznica katow a - b, -180..180. */
static float diff180(float a, float b)
{
    float d = fmodf(a - b + 540.0f, 360.0f);
    if (d < 0.0f) d += 360.0f;
    return d - 180.0f;
}

/** Czeka na NOWY wynik poziomu (seq sprzed wyslania komendy). */
static bool wait_level(uint32_t *seq, flora_level_cal_result_t *res)
{
    for (int t = 0; t < RESULT_WAIT_MS; t += REFRESH_MS) {
        if (hub_links_get(FLORA_MSG_LEVEL_CAL_RESULT, res, sizeof *res, seq)) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(REFRESH_MS));
    }
    return false;
}

/**
 * Poziom w trybie auto: krok 1, obrot o 180 st wg kursu glowicy, krok 2.
 * Kurs z glowicy liczy obrot z zyroskopu, wiec 180 st wzglednie jest pewne
 * nawet bez kalibracji magnetometru; blad kilku stopni prawie nie zmienia
 * wyniku (pochylosc podloza znosi sie z dokladnoscia do cos(blad)).
 */
static void level_auto(void)
{
    uint32_t seq = 0;
    flora_level_cal_result_t res;
    hub_links_get(FLORA_MSG_LEVEL_CAL_RESULT, &res, sizeof res, &seq);

    if (send_cmd(FLORA_CMD_CALIBRATE_LEVEL, 1) != ESP_OK || !wait_level(&seq, &res)) {
        ESP_LOGE(TAG, "poziom: brak odpowiedzi glowicy na krok 1");
        return;
    }
    if (res.status != FLORA_LEVEL_CAL_FIRST_OK) {
        ESP_LOGE(TAG, "poziom: krok 1 nieudany (kod %u) - robot musi stac nieruchomo", res.status);
        return;
    }

    uint32_t seq_att = 0;
    flora_attitude_t att;
    if (!hub_links_get(FLORA_MSG_ATTITUDE, &att, sizeof att, &seq_att)) {
        ESP_LOGE(TAG, "poziom: brak kursu z glowicy - obroc recznie i uzyj krokow 1/2");
        return;
    }
    float prev = att.heading_cdeg / 100.0f, turned = 0.0f;
    bool ok = false;
    for (int t = 0; t < LEVEL_TURN_MAX_MS; t += 50) {
        if (flora_motors_is_stalled() || !rotate()) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
        if (hub_links_get(FLORA_MSG_ATTITUDE, &att, sizeof att, &seq_att)) {
            float h = att.heading_cdeg / 100.0f;
            turned += diff180(h, prev);
            prev = h;
            if (fabsf(turned) >= LEVEL_TURN_DEG) {
                ok = true;
                break;
            }
        }
    }
    hub_drive_stop();
    if (!ok) {
        ESP_LOGE(TAG, "poziom: obrot o 180 st nieudany (obrocono %.0f st)", fabsf(turned));
        return;
    }
    ESP_LOGI(TAG, "poziom: obrocono %.1f st, czekam na uspokojenie masztu", fabsf(turned));
    vTaskDelay(pdMS_TO_TICKS(LEVEL_SETTLE_MS));

    if (send_cmd(FLORA_CMD_CALIBRATE_LEVEL, 2) != ESP_OK || !wait_level(&seq, &res)) {
        ESP_LOGE(TAG, "poziom: brak odpowiedzi glowicy na krok 2");
        return;
    }
    if (res.status == FLORA_LEVEL_CAL_OK) {
        ESP_LOGI(TAG, "poziom zapisany: pochylenie %.2f st, przechyl %.2f st",
                 res.off_pitch_cdeg / 100.0f, res.off_roll_cdeg / 100.0f);
    } else {
        ESP_LOGE(TAG, "poziom: krok 2 nieudany (kod %u)", res.status);
    }
}

/* ================= KURS WZGLEDNY Z GLOWICY =================
 * Suma zmian kursu (CW dodatnie) z kolejnych ATTITUDE - obrot o dowolnie
 * wiele pelnych obrotow, bez zawijania 0/360. */
typedef struct {
    uint32_t seq;
    float    prev, turned;
} hdg_t;

static bool hdg_begin(hdg_t *h)
{
    flora_attitude_t att;
    h->seq = 0;
    h->turned = 0.0f;
    if (!hub_links_get(FLORA_MSG_ATTITUDE, &att, sizeof att, &h->seq) ||
        !(att.flags & FLORA_ATT_HEADING_VALID)) {
        return false;
    }
    h->prev = att.heading_cdeg / 100.0f;
    return true;
}

static void hdg_poll(hdg_t *h)
{
    flora_attitude_t att;
    if (hub_links_get(FLORA_MSG_ATTITUDE, &att, sizeof att, &h->seq)) {
        float v = att.heading_cdeg / 100.0f;
        h->turned += diff180(v, h->prev);
        h->prev = v;
    }
}

/** Czeka ms, zbierajac kurs (np. wybieg po zatrzymaniu). */
static void hdg_wait(hdg_t *h, int ms)
{
    for (int t = 0; t < ms; t += 50) {
        vTaskDelay(pdMS_TO_TICKS(50));
        hdg_poll(h);
    }
}

static bool enc_mm(int64_t *l_counts, int64_t *r_counts)
{
    flora_encoders_sample_t s;
    if (flora_encoders_read(&s) != ESP_OK) {
        return false;
    }
    *l_counts = s.left_counts;
    *r_counts = s.right_counts;
    return true;
}

/* ================= ROZSTAW KOL ================= */

#define TRACK_STOP_MARGIN_DEG 5.0f
#define TRACK_SETTLE_MS       1500
#define TRACK_MAX_SPREAD      0.05f   /**< wyniki z obu kierunkow roznia sie najwyzej o 5% */

/** Jeden kierunek; zwraca rozstaw [mm] albo 0 przy bledzie. */
static float track_one(int dir, uint32_t turns, float *turned_out)
{
    hdg_t h;
    int64_t l0, r0, l1, r1;
    if (!hdg_begin(&h) || !enc_mm(&l0, &r0)) {
        ESP_LOGE(TAG, "rozstaw: brak kursu z glowicy albo enkoderow");
        return 0.0f;
    }
    const float goal = 360.0f * turns - TRACK_STOP_MARGIN_DEG;
    const int max_ms = (int)(goal / ROT_DPS * 2000.0f) + 10000;
    bool ok = false;
    for (int t = 0; t < max_ms; t += 50) {
        if (flora_motors_is_stalled() || !rotate_dir(dir)) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
        hdg_poll(&h);
        if (fabsf(h.turned) >= goal) {
            ok = true;
            break;
        }
    }
    hub_drive_stop();
    if (!ok) {
        ESP_LOGE(TAG, "rozstaw: obrot przerwany po %.0f st", fabsf(h.turned));
        return 0.0f;
    }
    hdg_wait(&h, TRACK_SETTLE_MS);   /* wybieg tez sie liczy, po obu stronach */
    if (!enc_mm(&l1, &r1)) {
        return 0.0f;
    }
    float mm_l, mm_r;
    flora_encoders_get_mm_per_count(&mm_l, &mm_r);
    float ds_l = (float)(l1 - l0) * mm_l, ds_r = (float)(r1 - r0) * mm_r;
    /* Kurs CW, theta CCW: obrot w lewo daje ujemne turned. */
    float dth = -h.turned * 3.14159265f / 180.0f;
    *turned_out = h.turned;
    float track = (ds_r - ds_l) / dth;
    ESP_LOGI(TAG, "rozstaw, obrot %s: kola %.0f / %.0f mm, kurs %.1f st -> %.1f mm",
             dir > 0 ? "w lewo" : "w prawo", ds_l, ds_r, h.turned, track);
    return track;
}

/**
 * Rozstaw z obrotu w miejscu: (ds_P - ds_L) / kat z glowicy. Dwa kierunki,
 * bo resztkowy dryf zyroskopu dodaje sie w jednym i odejmuje w drugim.
 * Przy waznym magnetometrze kurs po pelnych obrotach jest bezwzgledny,
 * wiec blad skali zyroskopu tez nie wchodzi.
 */
static void track_cal(uint32_t turns)
{
    float tl = 0.0f, tr = 0.0f;
    float track_l = track_one(1, turns, &tl);
    float track_r = track_l > 0.0f ? track_one(-1, turns, &tr) : 0.0f;
    float track = 0.5f * (track_l + track_r);
    const char *status = "ok";
    if (track_l <= 0.0f || track_r <= 0.0f) {
        status = "przerwana";
    } else if (fabsf(track_l - track_r) > TRACK_MAX_SPREAD * track) {
        status = "kierunki niezgodne";
    } else if (hub_params_save_track(track) != ESP_OK) {
        status = "poza zakresem 250-600";
    }
    if (strcmp(status, "ok") == 0) {
        ESP_LOGI(TAG, "rozstaw zapisany: %.1f mm", track);
    } else {
        ESP_LOGE(TAG, "rozstaw: %s (%.1f / %.1f mm)", status, track_l, track_r);
    }
    flora_mqtt_publish_fmt("tele/track_cal", 1, true,
                           "{\"status\":\"%s\",\"track_mm\":%.1f,\"left_turn_mm\":%.1f,"
                           "\"right_turn_mm\":%.1f,\"turned_deg\":[%.1f,%.1f]}",
                           status, track, track_l, track_r, tl, tr);
}

/* ================= SREDNICA KOL =================
 * Krok 1 (wheel_go): jazda prosto o zadana droge z odometrii; zapamietane
 * drogi kol i zmiana kursu. Krok 2 (cmd/wheel_cal <mm>): droga z tasmy.
 *
 * Prawdziwe drogi kol: sL = a * odoL, sP = b * odoP (a, b - bledy skali).
 *   (sL + sP) / 2 = S             S - droga srodka (z tasmy, cieciwa -> luk)
 *    sP - sL      = W * dtheta    luk, ktory zatoczyl robot (kurs z glowicy)
 * Dwa rownania na a i b - skala i roznica kol z jednego przejazdu. */

#define WHEEL_SPEED_MM_S 100.0f
#define WHEEL_PWM        30.0f

static struct {
    bool  pending;
    float odo_l, odo_r;     /**< drogi kol z odometrii [mm] */
    float dth;              /**< obrot CCW [rad]; 0 gdy bez glowicy */
    bool  have_heading;
    float mm_l, mm_r;       /**< droga na zliczenie w chwili przejazdu */
} s_wheel;

static void wheel_go(float dist_mm)
{
    s_wheel.pending = false;
    hdg_t h;
    bool have_h = hdg_begin(&h);
    int64_t l0, r0, l1, r1;
    if (!enc_mm(&l0, &r0)) {
        ESP_LOGE(TAG, "kola: enkodery niedostepne");
        return;
    }
    float mm_l, mm_r;
    flora_encoders_get_mm_per_count(&mm_l, &mm_r);
    bool pid = hub_drive_ready() && hub_drive_mode() == HUB_DRIVE_MODE_PID;
    const int max_ms = (int)(dist_mm / 30.0f * 1000.0f);   /* najwolniej 30 mm/s */
    bool ok = false;
    for (int t = 0; t < max_ms; t += 50) {
        esp_err_t e = pid ? hub_drive_set_speed(WHEEL_SPEED_MM_S, WHEEL_SPEED_MM_S)
                          : flora_motors_set(WHEEL_PWM, WHEEL_PWM);
        if (flora_motors_is_stalled() || e != ESP_OK) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
        if (have_h) hdg_poll(&h);
        if (!enc_mm(&l1, &r1)) break;
        float mean = 0.5f * ((float)(l1 - l0) * mm_l + (float)(r1 - r0) * mm_r);
        if (mean >= dist_mm) {
            ok = true;
            break;
        }
    }
    hub_drive_stop();
    if (!ok) {
        ESP_LOGE(TAG, "kola: przejazd przerwany (utyk, blad napedu albo za wolno)");
        flora_mqtt_publish_fmt("tele/wheel_cal", 1, true, "{\"status\":\"przerwana\"}");
        return;
    }
    if (have_h) hdg_wait(&h, TRACK_SETTLE_MS); else vTaskDelay(pdMS_TO_TICKS(TRACK_SETTLE_MS));
    enc_mm(&l1, &r1);

    s_wheel.odo_l = (float)(l1 - l0) * mm_l;
    s_wheel.odo_r = (float)(r1 - r0) * mm_r;
    s_wheel.have_heading = have_h;
    s_wheel.dth  = have_h ? -h.turned * 3.14159265f / 180.0f : 0.0f;
    s_wheel.mm_l = mm_l;
    s_wheel.mm_r = mm_r;
    s_wheel.pending = true;
    ESP_LOGW(TAG, "kola: przejechane wg odometrii %.0f / %.0f mm, kurs %+.1f st%s - "
                  "zmierz tasma odleglosc start-koniec i wyslij cmd/wheel_cal <mm>",
             s_wheel.odo_l, s_wheel.odo_r, have_h ? h.turned : 0.0f,
             have_h ? "" : " (bez glowicy - tylko skala)");
    flora_mqtt_publish_fmt("tele/wheel_cal", 1, true,
                           "{\"status\":\"czekam na pomiar\",\"odo_l_mm\":%.0f,\"odo_r_mm\":%.0f,"
                           "\"turn_deg\":%.1f,\"heading\":%s}",
                           s_wheel.odo_l, s_wheel.odo_r, have_h ? h.turned : 0.0f,
                           have_h ? "true" : "false");
}

static esp_err_t wheel_apply(float tape_mm)
{
    if (!s_wheel.pending) {
        return ESP_ERR_INVALID_STATE;   /* najpierw cmd/wheel_cal go */
    }
    hub_params_t p;
    hub_params_get(&p);
    /* Tasma mierzy cieciwe; droga srodka po luku: S = c * (th/2) / sin(th/2). */
    float half = 0.5f * fabsf(s_wheel.dth);
    float S = half > 1e-4f ? tape_mm * half / sinf(half) : tape_mm;
    float wd = p.track_mm * s_wheel.dth;
    float a = (2.0f * S - wd) / (2.0f * s_wheel.odo_l);
    float b = (2.0f * S + wd) / (2.0f * s_wheel.odo_r);
    if (!(a > 0.8f && a < 1.2f && b > 0.8f && b < 1.2f)) {
        ESP_LOGE(TAG, "kola: poprawka poza 0,8-1,2 (L %.3f, P %.3f) - zly pomiar?", a, b);
        return ESP_ERR_INVALID_ARG;
    }
    const float k = FLORA_COUNTS_PER_WHEEL_REV / 3.14159265f;
    float d_l = s_wheel.mm_l * a * k, d_r = s_wheel.mm_r * b * k;
    float wheel = 0.5f * (d_l + d_r), ratio = d_r / d_l;
    esp_err_t err = hub_params_save_wheels(wheel, ratio);
    if (err == ESP_OK) {
        s_wheel.pending = false;
        ESP_LOGI(TAG, "kola zapisane: srednia %.2f mm, P/L %.4f (poprawka L %.4f, P %.4f)",
                 wheel, ratio, a, b);
        flora_mqtt_publish_fmt("tele/wheel_cal", 1, true,
                               "{\"status\":\"ok\",\"wheel_mm\":%.2f,\"wheel_ratio\":%.4f,"
                               "\"scale_l\":%.4f,\"scale_r\":%.4f,\"tape_mm\":%.0f}",
                               wheel, ratio, a, b, tape_mm);
    }
    return err;
}

static void job_task(void *arg)
{
    const job_t job = s_job;
    if (job.kind != JOB_MAG) {
        if (job.kind == JOB_LEVEL_AUTO)     level_auto();
        else if (job.kind == JOB_TRACK)     track_cal(job.turns);
        else                                wheel_go(job.dist_mm);
        hub_drive_stop();
        s_running = false;
        vTaskDelete(NULL);
        return;
    }

    /* Numer ostatniego wyniku przed startem - czekamy na NOWY. */
    uint32_t seq = 0;
    flora_mag_cal_result_t res;
    hub_links_get(FLORA_MSG_MAG_CAL_RESULT, &res, sizeof res, &seq);

    esp_err_t err = send_cmd(FLORA_CMD_CALIBRATE_MAG, job.seconds);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "glowica nie przyjela komendy kalibracji: %s", esp_err_to_name(err));
        goto done;
    }
    ESP_LOGW(TAG, "kalibracja magnetometru %s, %" PRIu32 " s%s", job.auto_rotate ? "auto" : "reczna",
             job.seconds, job.auto_rotate ? " - robot obraca sie w miejscu" :
                                            " - obracaj glowice recznie 2-3 razy");

    const int64_t end = esp_timer_get_time() + (int64_t)job.seconds * 1000000;
    bool aborted = false;
    while (esp_timer_get_time() < end) {
        if (job.auto_rotate) {
            /* Odswiezamy co 200 ms - przy okazji wylapujemy utyk. */
            if (flora_motors_is_stalled() || !rotate()) {
                aborted = true;
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(REFRESH_MS));
    }
    if (job.auto_rotate) {
        hub_drive_stop();
    }
    if (aborted) {
        ESP_LOGE(TAG, "obrot przerwany (utyk albo blad napedu) - wynik bedzie z niepelnych danych");
    }

    /* Glowica konczy zbieranie po swoim zegarze - chwila zapasu na
     * dopasowanie i wyslanie wyniku. Publikacje na MQTT robi telemetria. */
    for (int t = 0; t < RESULT_WAIT_MS; t += REFRESH_MS) {
        if (hub_links_get(FLORA_MSG_MAG_CAL_RESULT, &res, sizeof res, &seq)) {
            if (res.status == 0) {
                ESP_LOGI(TAG, "kalibracja magnetometru udana (osie %.2f, rozrzut %.1f%%)",
                         res.axis_ratio_milli / 1000.0f, res.rms_cpct / 100.0f);
            } else {
                ESP_LOGE(TAG, "kalibracja magnetometru nieudana, kod %u - patrz head/mag_cal",
                         res.status);
            }
            goto done;
        }
        vTaskDelay(pdMS_TO_TICKS(REFRESH_MS));
    }
    ESP_LOGE(TAG, "brak wyniku kalibracji z glowicy");

done:
    s_running = false;
    vTaskDelete(NULL);
}

esp_err_t hub_headcal_mag(const char *arg)
{
    if (s_running) {
        return ESP_ERR_INVALID_STATE;
    }
    job_t job = { .kind = JOB_MAG, .auto_rotate = true, .seconds = 40 };
    if (strncmp(arg, "manual", 6) == 0) {
        job.auto_rotate = false;
        job.seconds = 60;
        arg += 6;
    } else if (strncmp(arg, "auto", 4) == 0) {
        arg += 4;
    } else if (arg[0]) {
        return ESP_ERR_INVALID_ARG;
    }
    while (*arg == ' ') arg++;
    if (*arg) {
        char *end;
        unsigned long s = strtoul(arg, &end, 10);
        if (end == arg || *end || s < 10 || s > 120) {
            return ESP_ERR_INVALID_ARG;
        }
        job.seconds = (uint32_t)s;
    }
    if (job.auto_rotate && flora_motors_is_stalled()) {
        return ESP_ERR_INVALID_STATE;
    }

    s_job = job;
    s_running = true;
    if (xTaskCreate(job_task, "kal_glowicy", 3584, NULL, 4, NULL) != pdPASS) {
        s_running = false;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t hub_headcal_level(const char *arg)
{
    if (s_running) {
        return ESP_ERR_INVALID_STATE;
    }
    if (strcmp(arg, "flat") == 0) return send_cmd(FLORA_CMD_CALIBRATE_LEVEL, 0);
    if (strcmp(arg, "1") == 0)    return send_cmd(FLORA_CMD_CALIBRATE_LEVEL, 1);
    if (strcmp(arg, "2") == 0)    return send_cmd(FLORA_CMD_CALIBRATE_LEVEL, 2);
    if (strcmp(arg, "auto") != 0 && arg[0]) {
        return ESP_ERR_INVALID_ARG;
    }
    if (flora_motors_is_stalled()) {
        return ESP_ERR_INVALID_STATE;
    }
    s_job = (job_t){ .kind = JOB_LEVEL_AUTO };
    s_running = true;
    if (xTaskCreate(job_task, "kal_glowicy", 3584, NULL, 4, NULL) != pdPASS) {
        s_running = false;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t hub_headcal_gyro(void)
{
    if (s_running) {
        return ESP_ERR_INVALID_STATE;
    }
    return send_cmd(FLORA_CMD_CALIBRATE_GYRO, 0);
}

static esp_err_t start_job(const job_t *job)
{
    if (s_running) {
        return ESP_ERR_INVALID_STATE;
    }
    if (flora_motors_is_stalled()) {
        return ESP_ERR_INVALID_STATE;
    }
    s_job = *job;
    s_running = true;
    if (xTaskCreate(job_task, "kal_jazda", 3584, NULL, 4, NULL) != pdPASS) {
        s_running = false;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t hub_headcal_track(const char *arg)
{
    job_t job = { .kind = JOB_TRACK, .turns = 3 };
    if (arg[0]) {
        char *end;
        unsigned long n = strtoul(arg, &end, 10);
        if (end == arg || *end || n < 1 || n > 10) {
            return ESP_ERR_INVALID_ARG;
        }
        job.turns = (uint32_t)n;
    }
    return start_job(&job);
}

esp_err_t hub_headcal_wheel(const char *arg)
{
    if (strncmp(arg, "go", 2) == 0) {
        job_t job = { .kind = JOB_WHEEL_GO, .dist_mm = 3000.0f };
        const char *p = arg + 2;
        while (*p == ' ') p++;
        if (*p) {
            char *end;
            float d = strtof(p, &end);
            if (end == p || *end || !(d >= 1000.0f && d <= 10000.0f)) {
                return ESP_ERR_INVALID_ARG;
            }
            job.dist_mm = d;
        }
        return start_job(&job);
    }
    if (s_running) {
        return ESP_ERR_INVALID_STATE;
    }
    char *end;
    float tape = strtof(arg, &end);
    if (end == arg || *end || !(tape >= 500.0f && tape <= 12000.0f)) {
        return ESP_ERR_INVALID_ARG;
    }
    return wheel_apply(tape);
}
