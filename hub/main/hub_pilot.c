/**
 * @file hub_pilot.c
 * @brief Jazda do punktu na robocie, opis w hub_pilot.h.
 */
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "hub_autocfg.h"
#include "hub_drive.h"
#include "hub_nav.h"
#include "hub_ota.h"
#include "hub_params.h"
#include "hub_pilot.h"
#include "flora_motors.h"
#include "flora_mqtt.h"

static const char *TAG = "pilot";

#define PERIOD_MS  50
#define HOLD_MS    300    /**< waznosc zadanej predkosci w hub_drive */

static SemaphoreHandle_t s_lock;
static flora_pilot_t     s_pilot;
static bool              s_started;

static const char *state_name(flora_pilot_state_t s)
{
    switch (s) {
    case FLORA_PILOT_TURN:   return "turn";
    case FLORA_PILOT_DRIVE:  return "drive";
    case FLORA_PILOT_FINAL:  return "final";
    case FLORA_PILOT_DONE:   return "done";
    case FLORA_PILOT_FAILED: return "failed";
    default:                 return "idle";
    }
}

static const char *fail_name(flora_pilot_fail_t f)
{
    switch (f) {
    case FLORA_PILOT_FAIL_NO_PROGRESS: return "no_progress";
    case FLORA_PILOT_FAIL_TIMEOUT:     return "timeout";
    case FLORA_PILOT_FAIL_CANCELLED:   return "cancelled";
    default:                           return "none";
    }
}

static void load_cfg(flora_pilot_cfg_t *c)
{
    *c = (flora_pilot_cfg_t)FLORA_PILOT_DEFAULT_CFG();
    hub_params_t p;
    hub_params_get(&p);
    c->track_mm   = p.track_mm;
    c->w_max_dps  = hub_autocfg_get(HUB_AC_P_WMAX);
    c->pos_tol_mm = hub_autocfg_get(HUB_AC_P_TOL_MM);
    if (c->w_min_dps > c->w_max_dps) {
        c->w_min_dps = c->w_max_dps;
    }
    /* Kolo zewnetrzne w poprawce kursu jedzie szybciej niz srodek - zostawiamy
     * na to zapas ponizej pid_vmax, inaczej hub_drive przycialby zadana
     * i robot nie skrecalby tak, jak chce pilot. */
    const float turn_mm_s = c->w_max_dps * 0.017453292f * 0.5f * c->track_mm;
    float v = hub_autocfg_get(HUB_AC_P_VMAX);
    const float room = hub_drive_vmax() - turn_mm_s;
    if (v > room) {
        v = room;
    }
    if (v < c->v_min) {
        v = c->v_min;
    }
    c->v_max = v;
}

/** Wspolne warunki startu zadania. */
static esp_err_t precheck(void)
{
    if (!s_started || !hub_autocfg_on(HUB_AC_PILOT_EN)) {
        return ESP_ERR_NOT_ALLOWED;
    }
    if (!hub_drive_ready() || hub_drive_mode() != HUB_DRIVE_MODE_PID || !hub_nav_ready() ||
        flora_motors_is_stalled() || hub_ota_in_progress()) {
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
}

esp_err_t hub_pilot_goto(float x_mm, float y_mm)
{
    esp_err_t err = precheck();
    if (err != ESP_OK) {
        return err;
    }
    flora_pilot_cfg_t cfg;
    load_cfg(&cfg);
    xSemaphoreTake(s_lock, portMAX_DELAY);
    flora_pilot_set_cfg(&s_pilot, &cfg);
    flora_pilot_goto(&s_pilot, x_mm, y_mm);
    xSemaphoreGive(s_lock);
    ESP_LOGI(TAG, "cel (%.0f, %.0f) mm, v %.0f mm/s", x_mm, y_mm, cfg.v_max);
    return ESP_OK;
}

esp_err_t hub_pilot_goto_pose(float x_mm, float y_mm, float th_deg)
{
    esp_err_t err = precheck();
    if (err != ESP_OK) {
        return err;
    }
    flora_pilot_cfg_t cfg;
    load_cfg(&cfg);
    xSemaphoreTake(s_lock, portMAX_DELAY);
    flora_pilot_set_cfg(&s_pilot, &cfg);
    flora_pilot_goto_pose(&s_pilot, x_mm, y_mm, th_deg);
    xSemaphoreGive(s_lock);
    ESP_LOGI(TAG, "cel (%.0f, %.0f) mm, kurs %.0f st", x_mm, y_mm, th_deg);
    return ESP_OK;
}

esp_err_t hub_pilot_turn(float th_deg)
{
    esp_err_t err = precheck();
    if (err != ESP_OK) {
        return err;
    }
    flora_pilot_cfg_t cfg;
    load_cfg(&cfg);
    xSemaphoreTake(s_lock, portMAX_DELAY);
    flora_pilot_set_cfg(&s_pilot, &cfg);
    flora_pilot_turn(&s_pilot, th_deg);
    xSemaphoreGive(s_lock);
    ESP_LOGI(TAG, "obrot do %.0f st", th_deg);
    return ESP_OK;
}

void hub_pilot_cancel(void)
{
    if (!s_started) {
        return;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    const bool was_busy = flora_pilot_busy(&s_pilot);
    flora_pilot_cancel(&s_pilot);
    xSemaphoreGive(s_lock);
    if (was_busy) {
        /* Naped zatrzyma zadanie pilota w najblizszym takcie (widzi zmiane
         * stanu); tu nie ruszamy silnikow, bo przerywa nas zwykle reczna
         * komenda jazdy, ktora zaraz zada wlasna moc. */
        ESP_LOGW(TAG, "zadanie przerwane");
    }
}

flora_pilot_state_t hub_pilot_state(flora_pilot_fail_t *fail)
{
    if (!s_started) {
        if (fail) *fail = FLORA_PILOT_FAIL_NONE;
        return FLORA_PILOT_IDLE;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    const flora_pilot_state_t st = s_pilot.state;
    if (fail) {
        *fail = s_pilot.fail;
    }
    xSemaphoreGive(s_lock);
    return st;
}

esp_err_t hub_pilot_cmd_goto(const char *arg)
{
    if (strcmp(arg, "stop") == 0) {
        hub_pilot_cancel();
        return ESP_OK;
    }
    char *end;
    const float x = strtof(arg, &end);
    if (end == arg || *end != ' ' || !isfinite(x)) {
        return ESP_ERR_INVALID_ARG;
    }
    const char *p = end;
    const float y = strtof(p, &end);
    if (end == p || !isfinite(y)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (*end == 0) {
        return hub_pilot_goto(x, y);
    }
    if (*end != ' ') {
        return ESP_ERR_INVALID_ARG;
    }
    p = end;
    const float th = strtof(p, &end);
    if (end == p || *end || !isfinite(th)) {
        return ESP_ERR_INVALID_ARG;
    }
    return hub_pilot_goto_pose(x, y, th);
}

esp_err_t hub_pilot_cmd_turn(const char *arg)
{
    char *end;
    const float th = strtof(arg, &end);
    if (end == arg || *end || !isfinite(th)) {
        return ESP_ERR_INVALID_ARG;
    }
    return hub_pilot_turn(th);
}

static void publish(const flora_pilot_out_t *o, const hub_nav_pose_t *pose)
{
    if (!flora_mqtt_is_connected()) {
        return;
    }
    flora_mqtt_publish_fmt("tele/pilot", 0, false,
                           "{\"state\":\"%s\",\"fail\":\"%s\",\"dist_mm\":%.0f,\"err_deg\":%.1f,"
                           "\"v_l\":%.0f,\"v_r\":%.0f,\"x_mm\":%.0f,\"y_mm\":%.0f,\"th_deg\":%.1f}",
                           state_name(o->state), fail_name(o->fail), o->dist_mm, o->err_deg,
                           o->v_l, o->v_r, pose->x_mm, pose->y_mm, pose->th_deg);
}

static void pilot_task(void *arg)
{
    TickType_t wake = xTaskGetTickCount();
    int64_t prev_us = esp_timer_get_time();
    bool was_busy = false;
    uint32_t tick = 0;

    for (;; tick++) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(PERIOD_MS));
        const int64_t now = esp_timer_get_time();
        const float dt = (float)(now - prev_us) / 1e6f;
        prev_us = now;

        xSemaphoreTake(s_lock, portMAX_DELAY);
        bool busy = flora_pilot_busy(&s_pilot);
        if (!busy && !was_busy) {
            xSemaphoreGive(s_lock);
            continue;
        }

        hub_nav_pose_t pose = { 0 };
        flora_pilot_out_t o = s_pilot.last;
        o.state = s_pilot.state;
        o.fail = s_pilot.fail;
        if (busy) {
            /* Wylaczenie przelacznika, utyk, zmiana trybu albo OTA w trakcie
             * jazdy koncza zadanie - to nie sa stany do przeczekania. */
            if (!hub_autocfg_on(HUB_AC_PILOT_EN) || flora_motors_is_stalled() ||
                hub_drive_mode() != HUB_DRIVE_MODE_PID || hub_ota_in_progress() ||
                !hub_nav_get(0, &pose)) {
                flora_pilot_cancel(&s_pilot);
                o.state = s_pilot.state;
                o.fail = s_pilot.fail;
                o.v_l = o.v_r = 0.0f;
            } else {
                o = flora_pilot_update(&s_pilot, pose.x_mm, pose.y_mm, pose.th_deg, dt);
            }
            busy = flora_pilot_busy(&s_pilot);
        } else {
            hub_nav_get(0, &pose);
        }
        xSemaphoreGive(s_lock);

        if (busy) {
            /* Odmowa straznika (NOT_ALLOWED) to nie blad pilota: probujemy
             * dalej, a licznik braku postepu zakonczy zadanie, gdy przeszkoda
             * nie zniknie. */
            const esp_err_t err = hub_drive_set_speed_hold(o.v_l, o.v_r, HOLD_MS);
            if (err != ESP_OK && err != ESP_ERR_NOT_ALLOWED) {
                hub_pilot_cancel();
            }
            if (tick % (500 / PERIOD_MS) == 0) {
                publish(&o, &pose);
            }
        } else {
            /* Koniec zadania (takze przerwanego). Reczna jazda, ktora nas
             * przerwala, zadala juz wlasna predkosc - wtedy nie zatrzymujemy. */
            if (o.fail != FLORA_PILOT_FAIL_CANCELLED) {
                hub_drive_stop();
            }
            ESP_LOGI(TAG, "koniec: %s%s%s, poza (%.0f, %.0f) %.0f st",
                     state_name(o.state),
                     o.state == FLORA_PILOT_FAILED ? " / " : "",
                     o.state == FLORA_PILOT_FAILED ? fail_name(o.fail) : "",
                     pose.x_mm, pose.y_mm, pose.th_deg);
            publish(&o, &pose);
        }
        was_busy = busy;
    }
}

esp_err_t hub_pilot_start(void)
{
    if (s_started) {
        return ESP_ERR_INVALID_STATE;
    }
    s_lock = xSemaphoreCreateMutex();
    if (!s_lock) {
        return ESP_ERR_NO_MEM;
    }
    flora_pilot_cfg_t cfg = FLORA_PILOT_DEFAULT_CFG();
    flora_pilot_init(&s_pilot, &cfg);
    if (xTaskCreate(pilot_task, "pilot", 4096, NULL, 5, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    s_started = true;
    return ESP_OK;
}
