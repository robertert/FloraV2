/**
 * @file hub_drive.c
 * @brief Petla predkosci kol, opis i procedura strojenia w hub_drive.h.
 *
 * Takt 20 ms na esp_timer, jak straznik utyku w flora_motors: zawieszone
 * zadanie aplikacji nie zatrzyma petli w polowie z moca na silnikach.
 * Przy ~30 zliczeniach na mm i 100 mm/s to ~60 zliczen na takt, wiec
 * rozdzielczosc pomiaru predkosci jest wystarczajaca.
 *
 * Stan dzielony z zadaniem MQTT chroni mutex. Kolejnosc blokad zawsze
 * ta sama: najpierw nasz, potem wewnetrzny flora_motors - straznik utyku
 * naszego nie bierze, wiec zakleszczenia nie ma.
 */
#include <inttypes.h>
#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "hub_drive.h"
#include "flora_board.h"
#include "flora_encoders.h"
#include "flora_motors.h"
#include "flora_pid.h"
#include "flora_store.h"

static const char *TAG = "petla";

#define LOOP_MS 20

/* Filtr pomiaru predkosci. Pojedynczy takt 20 ms jest szarpany (zliczenia
 * sa calkowite, a PCNT czytamy w losowym miejscu okresu enkodera), wiec
 * lekko wygladzamy - mocniej opozniloby petle. */
#define SPEED_ALPHA 0.5f

static struct {
    SemaphoreHandle_t  lock;
    esp_timer_handle_t timer;
    bool               ready;

    hub_drive_mode_t mode;
    flora_pid_t      pid_l, pid_r;
    float            acc;     /**< mm/s^2, 0 = bez rampy */
    float            vmax;

    float   target_l, target_r;   /**< zadana od uzytkownika */
    float   sp_l, sp_r;           /**< zadana po rampie */
    float   v_l, v_r;             /**< zmierzona, przefiltrowana */
    bool    active;
    int64_t step_until_us;        /**< 0 = brak testu skoku */

    flora_encoders_sample_t prev;
    bool                    have_prev;
} s_d;

static const char *mode_name(hub_drive_mode_t m)
{
    return m == HUB_DRIVE_MODE_PID ? "pid" : "pwm";
}

/** Zatrzymanie bez rampy. Zaklada trzymana blokade. */
static void halt_locked(void)
{
    s_d.target_l = s_d.target_r = 0.0f;
    s_d.sp_l = s_d.sp_r = 0.0f;
    s_d.step_until_us = 0;
    s_d.active = false;
    flora_pid_reset(&s_d.pid_l);
    flora_pid_reset(&s_d.pid_r);
    flora_motors_stop();
}

/** Przesuwa zadana po rampie w strone celu. */
static float ramp(float sp, float target, float dt)
{
    if (s_d.acc <= 0.0f) {
        return target;
    }
    float step = s_d.acc * dt;
    if (target > sp + step) return sp + step;
    if (target < sp - step) return sp - step;
    return target;
}

static void loop_tick(void *arg)
{
    flora_encoders_sample_t now;
    if (flora_encoders_read(&now) != ESP_OK) {
        return;
    }

    xSemaphoreTake(s_d.lock, portMAX_DELAY);

    float dt = 0.0f;
    if (s_d.have_prev) {
        dt = (float)(now.timestamp_us - s_d.prev.timestamp_us) / 1e6f;
        float vl, vr;
        flora_encoders_speed_mm_s(&s_d.prev, &now, &vl, &vr);
        s_d.v_l += SPEED_ALPHA * (vl - s_d.v_l);
        s_d.v_r += SPEED_ALPHA * (vr - s_d.v_r);
    }
    s_d.prev = now;
    s_d.have_prev = true;

    if (s_d.mode != HUB_DRIVE_MODE_PID || dt <= 0.0f) {
        s_d.active = false;
        goto out;
    }

    /* Utyk zatrzasnal blokade - nic nie wystawiamy, a calka nie moze
     * puchnac na stojacym kole w oczekiwaniu na clear_stall. */
    if (flora_motors_is_stalled()) {
        if (s_d.active || s_d.target_l != 0.0f || s_d.target_r != 0.0f) {
            ESP_LOGW(TAG, "utyk - petla zatrzymana, zadane wyzerowane");
            halt_locked();
        }
        goto out;
    }

    if (s_d.step_until_us && now.timestamp_us >= s_d.step_until_us) {
        s_d.step_until_us = 0;
        s_d.target_l = s_d.target_r = 0.0f;
    }

    s_d.sp_l = ramp(s_d.sp_l, s_d.target_l, dt);
    s_d.sp_r = ramp(s_d.sp_r, s_d.target_r, dt);

    /* Rampa doszla do zera: wybieg zamiast regulowania wokol zera, ktore
     * dawaloby brzeczenie i drganie kol na postoju. */
    if (s_d.sp_l == 0.0f && s_d.sp_r == 0.0f) {
        if (s_d.active) {
            halt_locked();
        }
        goto out;
    }

    float ul = flora_pid_update(&s_d.pid_l, s_d.sp_l, s_d.v_l, dt);
    float ur = flora_pid_update(&s_d.pid_r, s_d.sp_r, s_d.v_r, dt);
    esp_err_t err = flora_motors_set(ul, ur);
    if (err == ESP_ERR_INVALID_STATE) {
        /* Straznik zatrzasnal blokade miedzy sprawdzeniem a zapisem. */
        halt_locked();
    } else {
        s_d.active = true;
    }

out:
    xSemaphoreGive(s_d.lock);
}

/** Czyta nastawy z NVS do lokalnych zmiennych - bez blokady, NVS bywa wolne. */
static void load(flora_pid_cfg_t *cfg, float *acc, float *vmax, hub_drive_mode_t *mode)
{
    flora_pid_cfg_t c = FLORA_PID_DEFAULT_CFG();
    c.kp  = FLORA_PID_KP_DEFAULT;
    c.ki  = FLORA_PID_KI_DEFAULT;
    c.kd  = FLORA_PID_KD_DEFAULT;
    c.kff = FLORA_PID_KFF_DEFAULT;
    c.ks  = FLORA_PID_KS_DEFAULT;
    *acc  = FLORA_PID_ACC_DEFAULT;
    *vmax = FLORA_PID_VMAX_DEFAULT;
    char m[8] = "pwm";

    flora_store_t h;
    if (flora_store_open(FLORA_NVS_NS_CALIB, &h) == ESP_OK) {
        flora_store_get_float(h, FLORA_NVS_PID_KP,   &c.kp,  FLORA_PID_KP_DEFAULT);
        flora_store_get_float(h, FLORA_NVS_PID_KI,   &c.ki,  FLORA_PID_KI_DEFAULT);
        flora_store_get_float(h, FLORA_NVS_PID_KD,   &c.kd,  FLORA_PID_KD_DEFAULT);
        flora_store_get_float(h, FLORA_NVS_PID_KFF,  &c.kff, FLORA_PID_KFF_DEFAULT);
        flora_store_get_float(h, FLORA_NVS_PID_KS,   &c.ks,  FLORA_PID_KS_DEFAULT);
        flora_store_get_float(h, FLORA_NVS_PID_ACC,  acc,    FLORA_PID_ACC_DEFAULT);
        flora_store_get_float(h, FLORA_NVS_PID_VMAX, vmax,   FLORA_PID_VMAX_DEFAULT);
        flora_store_get_str(h, FLORA_NVS_DRIVE_MODE, m, sizeof m, "pwm");
        flora_store_close(h);
    }
    *cfg  = c;
    *mode = strcmp(m, "pid") == 0 ? HUB_DRIVE_MODE_PID : HUB_DRIVE_MODE_PWM;
}

static void apply_mode_locked(hub_drive_mode_t mode)
{
    if (mode == s_d.mode) {
        return;
    }
    /* Zmiana trybu zawsze przez zatrzymanie - nie przejmujemy w locie
     * mocy wystawionej przez drugi tryb. */
    halt_locked();
    s_d.mode = mode;
    ESP_LOGW(TAG, "tryb napedu: %s", mode_name(mode));
}

void hub_drive_reload(void)
{
    if (!s_d.ready) {
        return;
    }
    flora_pid_cfg_t cfg;
    float acc, vmax;
    hub_drive_mode_t mode;
    load(&cfg, &acc, &vmax, &mode);

    xSemaphoreTake(s_d.lock, portMAX_DELAY);
    flora_pid_set_cfg(&s_d.pid_l, &cfg);
    flora_pid_set_cfg(&s_d.pid_r, &cfg);
    s_d.acc  = acc;
    s_d.vmax = vmax;
    apply_mode_locked(mode);
    xSemaphoreGive(s_d.lock);

    ESP_LOGI(TAG, "nastawy: kp=%.3f ki=%.3f kd=%.3f kff=%.3f ks=%.1f acc=%.0f vmax=%.0f tryb=%s",
             cfg.kp, cfg.ki, cfg.kd, cfg.kff, cfg.ks, acc, vmax, mode_name(mode));
}

esp_err_t hub_drive_start(void)
{
    ESP_RETURN_ON_FALSE(!s_d.ready, ESP_ERR_INVALID_STATE, TAG, "juz wystartowala");

    s_d.lock = xSemaphoreCreateMutex();
    ESP_RETURN_ON_FALSE(s_d.lock, ESP_ERR_NO_MEM, TAG, "brak pamieci na mutex");

    flora_pid_cfg_t cfg;
    load(&cfg, &s_d.acc, &s_d.vmax, &s_d.mode);
    flora_pid_init(&s_d.pid_l, &cfg);
    flora_pid_init(&s_d.pid_r, &cfg);

    const esp_timer_create_args_t args = { .callback = loop_tick, .name = "drive_pid" };
    ESP_RETURN_ON_ERROR(esp_timer_create(&args, &s_d.timer), TAG, "timer petli");
    ESP_RETURN_ON_ERROR(esp_timer_start_periodic(s_d.timer, LOOP_MS * 1000), TAG, "start petli");
    s_d.ready = true;

    ESP_LOGI(TAG, "petla predkosci co %d ms, tryb %s; kp=%.3f ki=%.3f kd=%.3f kff=%.3f ks=%.1f",
             LOOP_MS, mode_name(s_d.mode), cfg.kp, cfg.ki, cfg.kd, cfg.kff, cfg.ks);
    return ESP_OK;
}

bool hub_drive_ready(void)
{
    return s_d.ready;
}

esp_err_t hub_drive_set_speed(float left_mm_s, float right_mm_s)
{
    ESP_RETURN_ON_FALSE(s_d.ready, ESP_ERR_INVALID_STATE, TAG, "petla nie dziala");
    if (flora_motors_is_stalled()) {
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreTake(s_d.lock, portMAX_DELAY);
    esp_err_t err = ESP_OK;
    if (s_d.mode != HUB_DRIVE_MODE_PID) {
        err = ESP_ERR_INVALID_STATE;
    } else {
        float v = s_d.vmax;
        s_d.target_l = fmaxf(-v, fminf(v, left_mm_s));
        s_d.target_r = fmaxf(-v, fminf(v, right_mm_s));
        s_d.step_until_us = 0;   /* zwykla jazda przejmuje od testu skoku */
    }
    xSemaphoreGive(s_d.lock);
    return err;
}

esp_err_t hub_drive_stop(void)
{
    if (!s_d.ready) {
        return flora_motors_stop();
    }
    xSemaphoreTake(s_d.lock, portMAX_DELAY);
    halt_locked();
    xSemaphoreGive(s_d.lock);
    return ESP_OK;
}

esp_err_t hub_drive_step(float mm_s, uint32_t ms)
{
    ESP_RETURN_ON_FALSE(s_d.ready, ESP_ERR_INVALID_STATE, TAG, "petla nie dziala");
    if (ms == 0 || ms > HUB_DRIVE_STEP_MAX_MS) {
        return ESP_ERR_INVALID_ARG;
    }
    if (flora_motors_is_stalled()) {
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreTake(s_d.lock, portMAX_DELAY);
    esp_err_t err = ESP_OK;
    if (s_d.mode != HUB_DRIVE_MODE_PID) {
        err = ESP_ERR_INVALID_STATE;
    } else if (fabsf(mm_s) > s_d.vmax) {
        err = ESP_ERR_INVALID_ARG;
    } else {
        s_d.target_l = s_d.target_r = mm_s;
        s_d.step_until_us = esp_timer_get_time() + (int64_t)ms * 1000;
        ESP_LOGI(TAG, "skok %.0f mm/s przez %" PRIu32 " ms", mm_s, ms);
    }
    xSemaphoreGive(s_d.lock);
    return err;
}

esp_err_t hub_drive_set_mode(hub_drive_mode_t mode)
{
    ESP_RETURN_ON_FALSE(s_d.ready, ESP_ERR_INVALID_STATE, TAG, "petla nie dziala");
    xSemaphoreTake(s_d.lock, portMAX_DELAY);
    apply_mode_locked(mode);
    xSemaphoreGive(s_d.lock);
    return ESP_OK;
}

hub_drive_mode_t hub_drive_mode(void)
{
    return s_d.mode;
}

float hub_drive_vmax(void)
{
    return s_d.ready ? s_d.vmax : FLORA_PID_VMAX_DEFAULT;
}

void hub_drive_get_state(hub_drive_state_t *out)
{
    memset(out, 0, sizeof(*out));
    if (!s_d.ready) {
        return;
    }
    xSemaphoreTake(s_d.lock, portMAX_DELAY);
    out->mode   = s_d.mode;
    out->active = s_d.active;
    out->sp_l   = s_d.sp_l;
    out->sp_r   = s_d.sp_r;
    out->v_l    = s_d.v_l;
    out->v_r    = s_d.v_r;
    out->u_l    = s_d.pid_l.last.out;
    out->u_r    = s_d.pid_r.last.out;
    out->ff_l   = s_d.pid_l.last.ff;
    out->ff_r   = s_d.pid_r.last.ff;
    out->i_l    = s_d.pid_l.last.i;
    out->i_r    = s_d.pid_r.last.i;
    xSemaphoreGive(s_d.lock);
}
