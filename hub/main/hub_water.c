/**
 * @file hub_water.c
 * @brief Automatyczne podlewanie, opis w hub_water.h.
 */
#include <inttypes.h>
#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"

#include "hub_autocfg.h"
#include "hub_mission.h"
#include "hub_ota.h"
#include "hub_power.h"
#include "hub_sensorcal.h"
#include "hub_telemetry.h"
#include "hub_water.h"
#include "flora_motors.h"
#include "flora_mqtt.h"
#include "flora_pump.h"
#include "flora_water.h"

static const char *TAG = "podlewanie";

static flora_water_t s_water;
static uint32_t      s_gen;
static bool          s_started, s_active;
static uint32_t      s_pump_total_ms;      /**< stan licznika pompki przy ostatnim kroku */
static volatile bool s_busy;
static volatile bool s_reset_req;
static int           s_last_state = -1;
static bool          s_no_cal_logged;

static const char *state_name(flora_water_state_t s)
{
    switch (s) {
    case FLORA_WATER_OK:        return "ok";
    case FLORA_WATER_DRY:       return "dry";
    case FLORA_WATER_DOSING:    return "dosing";
    case FLORA_WATER_SOAK:      return "soak";
    case FLORA_WATER_NO_WATER:  return "no_water";
    case FLORA_WATER_LIMIT:     return "limit";
    case FLORA_WATER_NO_SENSOR: return "no_sensor";
    default:                    return "fault";
    }
}

static const char *fault_name(flora_water_fault_t f)
{
    switch (f) {
    case FLORA_WATER_FAULT_NO_RISE: return "no_rise";
    case FLORA_WATER_FAULT_PUMP:    return "pump";
    default:                        return "none";
    }
}

static void load_cfg(flora_water_cfg_t *c)
{
    *c = (flora_water_cfg_t)FLORA_WATER_DEFAULT_CFG();
    c->dry_pct      = hub_autocfg_get(HUB_AC_W_DRY);
    c->wet_pct      = hub_autocfg_get(HUB_AC_W_WET);
    c->dose_ml      = hub_autocfg_get(HUB_AC_W_DOSE_ML);
    c->soak_s       = (uint32_t)hub_autocfg_get(HUB_AC_W_SOAK_S);
    c->daily_max_ml = hub_autocfg_get(HUB_AC_W_DAY_ML);
}

bool hub_water_busy(void)
{
    return s_busy;
}

esp_err_t hub_water_cmd(const char *arg)
{
    if (strcmp(arg, "reset") != 0) {
        return ESP_ERR_INVALID_ARG;
    }
    s_reset_req = true;      /* wykona zadanie hub_auto w najblizszym kroku */
    return ESP_OK;
}

static void publish(const flora_water_out_t *o, bool no_cal)
{
    if (!flora_mqtt_is_connected()) {
        return;
    }
    char b[12] = "null";
    if (!isnan(o->median_pct)) {
        snprintf(b, sizeof b, "%.1f", o->median_pct);
    }
    flora_mqtt_publish_fmt("tele/water", 0, false,
                           "{\"state\":\"%s\",\"fault\":\"%s\",\"median_pct\":%s,"
                           "\"day_ml\":%.0f,\"remaining_ml\":%.0f}",
                           no_cal ? "no_cal" : state_name(o->state), fault_name(o->fault), b,
                           o->day_ml, o->remaining_ml);
}

void hub_water_tick(void)
{
    static uint32_t tick;
    if (!s_started) {
        return;
    }
    flora_pump_stats_t st = { 0 };     /* bez pompki get_stats nic nie wpisuje */
    flora_pump_get_stats(&st);

    if (!hub_autocfg_on(HUB_AC_WATER_EN)) {
        if (s_active) {
            s_active = false;
            s_busy = false;
            s_last_state = -1;
            ESP_LOGW(TAG, "automatyczne podlewanie wylaczone");
            flora_mqtt_publish_str("tele/water", "{\"state\":\"off\"}", 0, false);
        }
        s_pump_total_ms = st.total_run_ms;
        return;
    }
    if (!s_active) {
        /* Wlaczenie: czysta karta - stare okno mediany i licznik dobowy
         * sprzed przerwy nic nie mowia o tym, co jest teraz. */
        flora_water_cfg_t cfg;
        load_cfg(&cfg);
        flora_water_init(&s_water, &cfg);
        s_gen = hub_autocfg_generation();
        s_pump_total_ms = st.total_run_ms;
        s_active = true;
        s_no_cal_logged = false;
        tick = 0;
        ESP_LOGW(TAG, "automatyczne podlewanie wlaczone: %.0f / %.0f %%, dawka %.0f ml",
                 cfg.dry_pct, cfg.wet_pct, cfg.dose_ml);
    }
    if (s_gen != hub_autocfg_generation()) {
        s_gen = hub_autocfg_generation();
        flora_water_cfg_t cfg;
        load_cfg(&cfg);
        flora_water_set_cfg(&s_water, &cfg);
    }
    if (s_reset_req) {
        s_reset_req = false;
        flora_water_reset_fault(&s_water);
        ESP_LOGW(TAG, "zatrzask awarii zdjety");
    }

    const float ml_s = hub_sensorcal_pump_ml_s();
    if (ml_s <= 0.0f) {
        if (!s_no_cal_logged) {
            s_no_cal_logged = true;
            ESP_LOGW(TAG, "brak kalibracji pompki (cmd/pump_cal) - podlewanie czeka");
        }
        s_busy = false;
        if (tick++ % 10 == 0) {
            publish(&s_water.last, true);
        }
        s_pump_total_ms = st.total_run_ms;
        return;
    }
    s_no_cal_logged = false;

    float l, r;
    flora_motors_get_last(&l, &r);
    flora_energy_out_t en;
    hub_power_get(&en);

    flora_water_in_t in = {
        .now_s         = (uint32_t)(esp_timer_get_time() / 1000000),
        .water_present = hub_telemetry_water_present(),
        .allowed       = l == 0.0f && r == 0.0f && hub_mission_water_ok() &&
                         en.allow_water && !hub_ota_in_progress(),
        .pump_busy     = flora_pump_is_running(),
        .max_chunk_ml  = ml_s * (HUB_PUMP_MAX_RUN_MS / 1000.0f),
    };
    float pct;
    if (hub_telemetry_soil_read(&pct) == ESP_OK) {
        in.soil_valid = true;
        in.soil_pct = pct;
    }
    /* Licznik pompki rosnie po zakonczeniu kazdej pracy, takze recznej. */
    if (st.total_run_ms != s_pump_total_ms) {
        in.delivered_ml = (float)(uint32_t)(st.total_run_ms - s_pump_total_ms) / 1000.0f * ml_s;
        s_pump_total_ms = st.total_run_ms;
    }

    const flora_water_out_t o = flora_water_update(&s_water, &in);
    s_busy = o.state == FLORA_WATER_DOSING;

    if (o.request_ml > 0.0f && !in.pump_busy) {
        const esp_err_t err = flora_pump_dose_ml(o.request_ml);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "porcja %.0f ml (wilgotnosc %.1f %%, dzis %.0f ml)",
                     o.request_ml, o.median_pct, o.day_ml);
        } else if (err != ESP_ERR_INVALID_STATE) {
            /* INVALID_STATE to przerwa miedzy porcjami - ponowimy za chwile.
             * Reszta (brak wody, limit czasu pompki) wyjdzie w stanie. */
            ESP_LOGW(TAG, "pompka odmowila: %s", esp_err_to_name(err));
        }
    }

    if ((int)o.state != s_last_state) {
        if (o.state == FLORA_WATER_FAULT) {
            ESP_LOGE(TAG, "awaria: %s - podlewanie stoi do cmd/water reset",
                     fault_name(o.fault));
        } else {
            ESP_LOGI(TAG, "stan: %s", state_name(o.state));
        }
        s_last_state = (int)o.state;
        publish(&o, false);
    } else if (tick % 10 == 0) {
        publish(&o, false);
    }
    tick++;
}

esp_err_t hub_water_start(void)
{
    flora_water_cfg_t cfg;
    load_cfg(&cfg);
    flora_water_init(&s_water, &cfg);
    s_started = true;
    return ESP_OK;
}
