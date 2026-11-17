/**
 * @file hub_mission.c
 * @brief Misja na robocie, opis w hub_mission.h.
 */
#include <inttypes.h>
#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"

#include "hub_autocfg.h"
#include "hub_links.h"
#include "hub_mission.h"
#include "hub_nav.h"
#include "hub_pilot.h"
#include "hub_power.h"
#include "hub_safety.h"
#include "hub_water.h"
#include "flora_mission.h"
#include "flora_motors.h"
#include "flora_mqtt.h"
#include "flora_msgs.h"

static const char *TAG = "misja";

/** Pomiar slonca starszy niz to nie liczy sie (SUN idzie co 1 s). */
#define LUX_MAX_AGE_US 5000000

static flora_mission_t s_mission;
static uint32_t        s_gen;
static bool            s_started;
static volatile bool   s_water_ok = true, s_may_sleep = true;
static volatile int64_t s_manual_us;
static volatile bool   s_req_seek, s_req_reset;

static const char *state_name(flora_mission_state_t s)
{
    switch (s) {
    case FLORA_MISSION_DWELL:        return "dwell";
    case FLORA_MISSION_SEEK_MOVE:    return "seek_move";
    case FLORA_MISSION_SEEK_MEASURE: return "seek_measure";
    case FLORA_MISSION_WATER:        return "water";
    case FLORA_MISSION_LOW_BATT:     return "low_batt";
    case FLORA_MISSION_HOLD:         return "hold";
    case FLORA_MISSION_FAULT:        return "fault";
    default:                         return "idle";
    }
}

static const char *fault_name(flora_mission_fault_t f)
{
    switch (f) {
    case FLORA_MISSION_FAULT_STALL:   return "stall";
    case FLORA_MISSION_FAULT_BLOCKED: return "blocked";
    default:                          return "none";
    }
}

static const char *result_name(flora_seek_result_t r)
{
    switch (r) {
    case FLORA_SEEK_RES_IMPROVED: return "jasniej";
    case FLORA_SEEK_RES_NO_GAIN:  return "bez zysku";
    case FLORA_SEEK_RES_ABORTED:  return "przerwane";
    case FLORA_SEEK_RES_BLOCKED:  return "zablokowane";
    default:                      return "-";
    }
}

static void load_seek_cfg(flora_seek_cfg_t *c)
{
    *c = (flora_seek_cfg_t)FLORA_SEEK_DEFAULT_CFG();
    hub_area_t a;
    hub_autocfg_area(&a);
    c->area_valid   = a.valid;
    c->xmin = a.xmin;  c->ymin = a.ymin;
    c->xmax = a.xmax;  c->ymax = a.ymax;
    c->probe_len_mm = hub_autocfg_get(HUB_AC_S_LEN_MM);
    c->shade_ratio  = hub_autocfg_get(HUB_AC_S_RATIO);
    c->shade_hold_s = (uint32_t)hub_autocfg_get(HUB_AC_S_HOLD_S);
    c->min_dwell_s  = (uint32_t)hub_autocfg_get(HUB_AC_S_DWELL_S);
    if (c->min_len_mm > 0.5f * c->probe_len_mm) {
        c->min_len_mm = 0.5f * c->probe_len_mm;
    }
}

bool hub_mission_water_ok(void)  { return s_water_ok; }
bool hub_mission_may_sleep(void) { return s_may_sleep; }

void hub_mission_note_manual(void)
{
    s_manual_us = esp_timer_get_time();
}

esp_err_t hub_mission_cmd(const char *arg)
{
    if (strcmp(arg, "stop") == 0) {
        /* Najpierw naped, potem przelacznik: stop ma zadzialac takze wtedy,
         * gdy zapis do NVS sie nie uda. */
        hub_pilot_cancel();
        return hub_autocfg_set("mission_en", "0");
    }
    if (!hub_autocfg_on(HUB_AC_MISSION_EN)) {
        return ESP_ERR_NOT_ALLOWED;
    }
    if (strcmp(arg, "seek") == 0)  { s_req_seek = true;  return ESP_OK; }
    if (strcmp(arg, "reset") == 0) { s_req_reset = true; return ESP_OK; }
    return ESP_ERR_INVALID_ARG;
}

static void publish(const flora_mission_out_t *o, float lux, float contrast)
{
    if (!flora_mqtt_is_connected()) {
        return;
    }
    char bl[16] = "null", br[16] = "null";
    if (!isnan(o->seek.lux_filt)) snprintf(bl, sizeof bl, "%.0f", o->seek.lux_filt);
    if (!isnan(o->seek.lux_ref))  snprintf(br, sizeof br, "%.0f", o->seek.lux_ref);
    (void)lux;
    flora_mqtt_publish_fmt("tele/mission", 0, false,
                           "{\"state\":\"%s\",\"fault\":\"%s\",\"shade\":%s,\"day\":%s,"
                           "\"lux\":%s,\"lux_ref\":%s,\"contrast\":%.2f,"
                           "\"shade_s\":%" PRIu32 ",\"wait_s\":%" PRIu32 ",\"probe\":%u}",
                           state_name(o->state), fault_name(o->fault),
                           o->seek.shade ? "true" : "false", o->seek.day ? "true" : "false",
                           bl, br, contrast, o->seek.shade_s, o->seek.wait_s,
                           (unsigned)o->seek.probe);
}

void hub_mission_tick(void)
{
    static uint32_t seq_sun, tick;
    static int64_t  sun_us;
    static float    lux, contrast;
    static bool     have_sun, nav_force_fail, was_enabled;
    static int      last_state = -1;

    if (!s_started) {
        return;
    }
    const bool enabled = hub_autocfg_on(HUB_AC_MISSION_EN);
    if (!enabled && !was_enabled) {
        s_water_ok = true;
        s_may_sleep = true;
        s_req_seek = s_req_reset = false;
        return;      /* wylaczona: zadnych odczytow, zadnych publikacji */
    }
    const int64_t now_us = esp_timer_get_time();
    const uint32_t now_s = (uint32_t)(now_us / 1000000);

    if (s_gen != hub_autocfg_generation()) {
        s_gen = hub_autocfg_generation();
        flora_seek_cfg_t sc;
        load_seek_cfg(&sc);
        flora_mission_set_seek_cfg(&s_mission, &sc);
    }
    if (s_req_reset) {
        s_req_reset = false;
        flora_mission_reset(&s_mission, now_s);
        ESP_LOGW(TAG, "reset: zatrzask zdjety, postoj od nowa");
    }
    if (s_req_seek) {
        s_req_seek = false;
        flora_mission_force_seek(&s_mission);
        ESP_LOGW(TAG, "wymuszone szukanie");
    }

    flora_sun_t sun;
    if (hub_links_get(FLORA_MSG_SUN, &sun, sizeof sun, &seq_sun)) {
        float l[4];
        for (int i = 0; i < 4; i++) {
            l[i] = sun.lux_div4[i] * 4.0f;
        }
        lux = 0.25f * (l[0] + l[1] + l[2] + l[3]);
        contrast = flora_seek_contrast(l);
        sun_us = now_us;
        have_sun = true;
    }

    hub_nav_pose_t pose = { 0 };
    const bool pose_ok = hub_nav_ready() && hub_nav_get(0, &pose);

    flora_pilot_fail_t pfail;
    flora_seek_nav_t nav;
    switch (hub_pilot_state(&pfail)) {
    case FLORA_PILOT_TURN:
    case FLORA_PILOT_DRIVE:
    case FLORA_PILOT_FINAL:  nav = FLORA_SEEK_NAV_BUSY;    break;
    case FLORA_PILOT_DONE:   nav = FLORA_SEEK_NAV_ARRIVED; break;
    case FLORA_PILOT_FAILED: nav = FLORA_SEEK_NAV_FAILED;  break;
    default:                 nav = FLORA_SEEK_NAV_IDLE;    break;
    }
    if (nav_force_fail) {
        /* Pilot odmowil przyjecia celu - jego stan to wynik POPRZEDNIEGO
         * zadania i nie wolno go wziac za dojazd. */
        nav = FLORA_SEEK_NAV_FAILED;
        nav_force_fail = false;
    }

    flora_energy_out_t en;
    hub_power_get(&en);
    float elev = 0.0f, az = 0.0f;
    const bool sun_known = hub_power_sun(&elev, &az);

    flora_mission_in_t in = {
        .enabled = enabled,
        .seek = {
            .now_s         = now_s,
            .lux_valid     = have_sun && now_us - sun_us <= LUX_MAX_AGE_US,
            .lux           = lux,
            .contrast      = contrast,
            .elev_known    = sun_known,
            .sun_elev_deg  = elev,
            /* b_deg to kurs kompasowy osi X mapy; azymut (CW od polnocy)
             * w ukladzie mapy (CCW od X) to b - azymut. */
            .sun_dir_valid = sun_known && pose_ok,
            .sun_map_deg   = pose.b_deg - az,
            .x_mm = pose.x_mm, .y_mm = pose.y_mm,
            .nav = nav,
        },
        .pose_ok       = pose_ok,
        .drive_allowed = en.allow_drive,
        .autonomy_ok   = hub_safety_autonomy_ok(),
        .manual        = s_manual_us &&
                         now_us - s_manual_us < HUB_MISSION_MANUAL_HOLD_S * 1000000LL,
        .water_busy    = hub_water_busy(),
        .stalled       = flora_motors_is_stalled(),
    };
    const flora_mission_out_t o = flora_mission_update(&s_mission, &in);
    s_water_ok = o.water_ok;
    s_may_sleep = o.may_sleep;

    if (o.cmd == FLORA_SEEK_CMD_GOTO) {
        const esp_err_t err = hub_pilot_goto(o.gx, o.gy);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "pilot nie przyjal celu (%.0f, %.0f): %s", o.gx, o.gy,
                     esp_err_to_name(err));
            nav_force_fail = true;
        } else {
            ESP_LOGI(TAG, "punkt %u: (%.0f, %.0f), jasnosc %.0f lx", (unsigned)o.seek.probe,
                     o.gx, o.gy, o.seek.lux_filt);
        }
    } else if (o.cmd == FLORA_SEEK_CMD_STOP) {
        hub_pilot_cancel();
    }
    if (o.seek.result != FLORA_SEEK_RES_NONE) {
        ESP_LOGW(TAG, "szukanie zakonczone: %s, jasnosc %.0f lx, poza (%.0f, %.0f)",
                 result_name(o.seek.result), o.seek.lux_filt, pose.x_mm, pose.y_mm);
    }

    if ((int)o.state != last_state) {
        if (o.state == FLORA_MISSION_FAULT) {
            ESP_LOGE(TAG, "awaria: %s - misja stoi do cmd/mission reset", fault_name(o.fault));
        } else {
            ESP_LOGI(TAG, "stan: %s", state_name(o.state));
        }
        last_state = (int)o.state;
        publish(&o, lux, contrast);
        tick = 0;
    } else if (tick % 10 == 0) {
        publish(&o, lux, contrast);
    }
    tick++;
    was_enabled = enabled;
}

esp_err_t hub_mission_start(void)
{
    flora_mission_cfg_t cfg = FLORA_MISSION_DEFAULT_CFG();
    flora_seek_cfg_t sc;
    load_seek_cfg(&sc);
    flora_mission_init(&s_mission, &cfg, &sc);
    s_gen = hub_autocfg_generation();
    s_started = true;
    return ESP_OK;
}
