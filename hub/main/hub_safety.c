/**
 * @file hub_safety.c
 * @brief Straznik napedu na sprzecie, opis w hub_safety.h.
 */
#include <inttypes.h>
#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "hub_autocfg.h"
#include "hub_links.h"
#include "hub_power.h"
#include "hub_safety.h"
#include "hub_telemetry.h"
#include "flora_mqtt.h"
#include "flora_msgs.h"

static const char *TAG = "straznik";

#define PERIOD_MS      50
#define VERDICT_MAX_US 300000   /**< starszy werdykt = brak zgody */

_Static_assert(FLORA_GUARD_SECTORS == FLORA_OBSTACLE_SECTORS, "sektory straznika i kontraktu");
_Static_assert(FLORA_GUARD_SECTOR_UNKNOWN == FLORA_OBSTACLE_UNKNOWN, "kod 'nie wiem'");
_Static_assert(FLORA_GUARD_SECTOR_CLEAR == FLORA_OBSTACLE_CLEAR, "kod 'czysto'");

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static flora_guard_out_t s_out;       /**< pod s_mux */
static int64_t           s_out_us;    /**< pod s_mux */
static volatile bool     s_running;

bool hub_safety_enabled(void)
{
    return s_running && hub_autocfg_on(HUB_AC_GUARD_EN);
}

float hub_safety_permit(float left, float right)
{
    if (!hub_autocfg_on(HUB_AC_GUARD_EN)) {
        return 1.0f;
    }
    if (left == 0.0f && right == 0.0f) {
        return 1.0f;
    }
    if (!s_running) {
        return 0.0f;      /* wlaczony, ale zadanie nie wystartowalo */
    }
    flora_guard_out_t o;
    int64_t t;
    portENTER_CRITICAL(&s_mux);
    o = s_out;
    t = s_out_us;
    portEXIT_CRITICAL(&s_mux);
    if (esp_timer_get_time() - t > VERDICT_MAX_US) {
        return 0.0f;
    }
    return flora_guard_permit(&o, left, right);
}

bool hub_safety_autonomy_ok(void)
{
    if (!hub_autocfg_on(HUB_AC_GUARD_EN)) {
        return true;
    }
    if (!s_running) {
        return false;
    }
    bool ok;
    int64_t t;
    portENTER_CRITICAL(&s_mux);
    ok = s_out.autonomy_ok;
    t = s_out_us;
    portEXIT_CRITICAL(&s_mux);
    return ok && esp_timer_get_time() - t <= VERDICT_MAX_US;
}

void hub_safety_get(flora_guard_out_t *out)
{
    portENTER_CRITICAL(&s_mux);
    *out = s_out;
    portEXIT_CRITICAL(&s_mux);
}

static void load_cfg(flora_guard_cfg_t *c)
{
    *c = (flora_guard_cfg_t)FLORA_GUARD_DEFAULT_CFG();
    c->mask          = (uint32_t)hub_autocfg_get(HUB_AC_GUARD_MASK);
    c->stop_front_mm = hub_autocfg_get(HUB_AC_G_FRONT_MM);
    c->stop_rear_mm  = hub_autocfg_get(HUB_AC_G_REAR_MM);
    c->tilt_stop_deg = hub_autocfg_get(HUB_AC_G_TILT_DEG);
    c->tilt_clear_deg = 0.75f * c->tilt_stop_deg;
    c->v_stop        = hub_autocfg_get(HUB_AC_G_V_STOP);
    c->v_clear       = c->v_stop + 0.4f;
}

static const char *num(char *buf, size_t cap, float v)
{
    if (isnan(v) || v < 0.0f) {
        return "null";
    }
    snprintf(buf, cap, "%.0f", v);
    return buf;
}

static void publish(const flora_guard_out_t *o, bool enabled)
{
    if (!flora_mqtt_is_connected()) {
        return;
    }
    char bf[12], br[12];
    flora_mqtt_publish_fmt("tele/safety", 0, false,
                           "{\"en\":%s,\"fwd\":%s,\"rev\":%s,\"turn\":%s,\"auto\":%s,"
                           "\"limit\":%.2f,\"reasons\":%" PRIu32 ",\"front_mm\":%s,\"rear_mm\":%s}",
                           enabled ? "true" : "false",
                           o->allow_fwd ? "true" : "false", o->allow_rev ? "true" : "false",
                           o->allow_turn ? "true" : "false", o->autonomy_ok ? "true" : "false",
                           o->power_limit, o->reasons,
                           num(bf, sizeof bf, o->front_mm), num(br, sizeof br, o->rear_mm));
}

static void safety_task(void *arg)
{
    flora_guard_cfg_t cfg;
    flora_guard_t g;
    load_cfg(&cfg);
    flora_guard_init(&g, &cfg);
    uint32_t gen = hub_autocfg_generation();

    flora_attitude_t att = { 0 };
    flora_obstacles_t obs = { 0 };
    uint32_t seq_att = 0, seq_obs = 0;
    int64_t att_us = 0, obs_us = 0;
    bool have_att = false, have_obs = false;

    bool was_enabled = false;
    uint32_t last_reasons = 0;
    uint32_t tick = 0;
    TickType_t wake = xTaskGetTickCount();

    for (;; tick++) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(PERIOD_MS));
        const bool enabled = hub_autocfg_on(HUB_AC_GUARD_EN);
        if (!enabled) {
            if (was_enabled) {
                /* Wylaczenie: czysty stan na nastepne wlaczenie. */
                flora_guard_init(&g, &cfg);
                was_enabled = false;
                last_reasons = 0;
                publish(&g.last, false);
                ESP_LOGW(TAG, "straznik wylaczony");
            }
            continue;
        }
        if (gen != hub_autocfg_generation()) {
            gen = hub_autocfg_generation();
            load_cfg(&cfg);
            flora_guard_set_cfg(&g, &cfg);
        }

        const int64_t now = esp_timer_get_time();
        if (hub_links_get(FLORA_MSG_ATTITUDE, &att, sizeof att, &seq_att)) {
            att_us = now;
            have_att = true;
        }
        if (hub_links_get(FLORA_MSG_OBSTACLES, &obs, sizeof obs, &seq_obs)) {
            obs_us = now;
            have_obs = true;
        }

        flora_guard_in_t in = {
            .now_ms     = (uint32_t)(now / 1000),
            .att_valid  = have_att,
            .att_age_ms = have_att ? (uint32_t)((now - att_us) / 1000) : 0,
            .pitch_deg  = att.pitch_cdeg / 100.0f,
            .roll_deg   = att.roll_cdeg / 100.0f,
            .obs_valid  = have_obs && obs.valid,
            .obs_age_ms = have_obs ? (uint32_t)((now - obs_us) / 1000) : 0,
        };
        memcpy(in.sectors, obs.sectors, sizeof in.sectors);
        for (int i = 0; i < FLORA_GUARD_SONARS; i++) {
            in.sonar_mm[i] = hub_telemetry_sonar_near(i);
        }
        in.bus_v_valid = hub_power_bus_v(&in.bus_v);

        const flora_guard_out_t o = flora_guard_update(&g, &in);
        portENTER_CRITICAL(&s_mux);
        s_out = o;
        s_out_us = now;
        portEXIT_CRITICAL(&s_mux);

        if (!was_enabled) {
            was_enabled = true;
            ESP_LOGW(TAG, "straznik wlaczony, maska %" PRIu32, cfg.mask);
        }
        if (o.reasons != last_reasons) {
            ESP_LOGW(TAG, "powody %" PRIu32 " -> %" PRIu32 " (przod %d, tyl %d, obrot %d)",
                     last_reasons, o.reasons, o.allow_fwd, o.allow_rev, o.allow_turn);
            last_reasons = o.reasons;
            publish(&o, true);
        } else if (tick % (1000 / PERIOD_MS) == 0) {
            publish(&o, true);
        }
    }
}

esp_err_t hub_safety_start(void)
{
    if (s_running) {
        return ESP_ERR_INVALID_STATE;
    }
    s_out = (flora_guard_out_t){ .allow_fwd = true, .allow_rev = true, .allow_turn = true,
                                 .autonomy_ok = true, .power_limit = 1.0f,
                                 .front_mm = -1.0f, .rear_mm = -1.0f };
    /* Priorytet nad telemetria i misja: werdykt ma byc swiezy. */
    if (xTaskCreate(safety_task, "safety", 4096, NULL, 6, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    s_running = true;
    return ESP_OK;
}
