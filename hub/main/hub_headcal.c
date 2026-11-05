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
#include "flora_board.h"
#include "flora_motors.h"
#include "flora_msgs.h"

static const char *TAG = "kal_glowicy";

#define ROT_DPS        20.0f    /**< predkosc obrotu w trybie auto */
#define ROT_PWM        30.0f    /**< moc obrotu, gdy naped w trybie pwm */
#define REFRESH_MS     200
#define RESULT_WAIT_MS 5000

static volatile bool s_running;

typedef enum { JOB_MAG, JOB_LEVEL_AUTO } job_kind_t;

typedef struct {
    job_kind_t kind;
    bool       auto_rotate;
    uint32_t   seconds;
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

/** Obrot w miejscu w lewo; false gdy naped odmowil (np. utyk). */
static bool rotate(void)
{
    if (hub_drive_ready() && hub_drive_mode() == HUB_DRIVE_MODE_PID) {
        float v = 0.5f * FLORA_TRACK_MM * ROT_DPS * 3.14159265f / 180.0f;
        return hub_drive_set_speed(-v, v) == ESP_OK;
    }
    return flora_motors_set(-ROT_PWM, ROT_PWM) == ESP_OK;
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

static void job_task(void *arg)
{
    const job_t job = s_job;
    if (job.kind == JOB_LEVEL_AUTO) {
        level_auto();
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
    if (xTaskCreate(job_task, "kal_glowicy", 3072, NULL, 4, NULL) != pdPASS) {
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
    if (xTaskCreate(job_task, "kal_glowicy", 3072, NULL, 4, NULL) != pdPASS) {
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
