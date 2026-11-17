/**
 * @file hub_autocfg.c
 * @brief Przelaczniki i progi funkcji autonomicznych, opis w hub_autocfg.h.
 *
 * Wartosci leza w tablicy float w RAM: zapis i odczyt jednego slowa sa na
 * ESP32 niepodzielne, wiec czytac wolno skadkolwiek bez blokady (takze
 * z timera napedu). NVS dotykamy tylko przy starcie i przy zmianie.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"

#include "hub_autocfg.h"
#include "flora_board.h"
#include "flora_mqtt.h"
#include "flora_store.h"

static const char *TAG = "autocfg";

typedef enum { G_SWITCH, G_GUARD, G_PILOT, G_WATER, G_SEEK, G_ENERGY, G_COUNT } group_t;

static const char *const GROUP_TOPIC[G_COUNT] = {
    "config_auto", "config_guard", "config_pilot", "config_water", "config_seek",
    "config_energy",
};

/* Klucz MQTT = klucz NVS (najwyzej 15 znakow). Kolejnosc jak hub_ac_key_t. */
static const struct {
    const char *key;
    group_t     group;
    float       def, lo, hi;
    bool        integer;   /**< przelaczniki, maska i sekundy: tylko liczby calkowite */
} KEYS[HUB_AC_COUNT] = {
    [HUB_AC_GUARD_EN]   = { "guard_en",   G_SWITCH, 0, 0, 1, true },
    [HUB_AC_PILOT_EN]   = { "pilot_en",   G_SWITCH, 0, 0, 1, true },
    [HUB_AC_WATER_EN]   = { "water_en",   G_SWITCH, 0, 0, 1, true },
    [HUB_AC_MISSION_EN] = { "mission_en", G_SWITCH, 0, 0, 1, true },
    [HUB_AC_SLEEP_EN]   = { "sleep_en",   G_SWITCH, 0, 0, 1, true },
    [HUB_AC_MDD_KEY_EN] = { "mdd_key_en", G_SWITCH, 0, 0, 1, true },

    [HUB_AC_GUARD_MASK] = { "guard_mask", G_GUARD, 31, 0, 31, true },
    [HUB_AC_G_FRONT_MM] = { "g_front_mm", G_GUARD, 350, 150, 2000, false },
    [HUB_AC_G_REAR_MM]  = { "g_rear_mm",  G_GUARD, 300, 200, 2000, false },
    [HUB_AC_G_TILT_DEG] = { "g_tilt_deg", G_GUARD, 20, 5, 45, false },
    [HUB_AC_G_V_STOP]   = { "g_v_stop",   G_GUARD, 11.8f, 10.5f, 13.0f, false },

    [HUB_AC_P_VMAX]     = { "p_vmax",     G_PILOT, 120, 30, 500, false },
    [HUB_AC_P_WMAX]     = { "p_wmax",     G_PILOT, 25, 5, 90, false },
    [HUB_AC_P_TOL_MM]   = { "p_tol_mm",   G_PILOT, 60, 20, 500, false },

    [HUB_AC_W_DRY]      = { "w_dry",      G_WATER, 35, 5, 90, false },
    [HUB_AC_W_WET]      = { "w_wet",      G_WATER, 55, 10, 95, false },
    [HUB_AC_W_DOSE_ML]  = { "w_dose_ml",  G_WATER, 50, 5, 500, false },
    [HUB_AC_W_SOAK_S]   = { "w_soak_s",   G_WATER, 600, 30, 7200, true },
    [HUB_AC_W_DAY_ML]   = { "w_day_ml",   G_WATER, 500, 10, 3000, false },

    [HUB_AC_S_LEN_MM]   = { "s_len_mm",   G_SEEK, 1500, 500, 5000, false },
    [HUB_AC_S_RATIO]    = { "s_ratio",    G_SEEK, 0.5f, 0.1f, 0.9f, false },
    [HUB_AC_S_HOLD_S]   = { "s_hold_s",   G_SEEK, 600, 10, 7200, true },
    [HUB_AC_S_DWELL_S]  = { "s_dwell_s",  G_SEEK, 1800, 60, 43200, true },

    [HUB_AC_E_V_SAVE]   = { "e_v_save",   G_ENERGY, 12.6f, 11.5f, 13.4f, false },
    [HUB_AC_E_V_SURV]   = { "e_v_surv",   G_ENERGY, 12.2f, 11.0f, 13.2f, false },
    [HUB_AC_E_NIGHT_S]  = { "e_night_s",  G_ENERGY, 1800, 0, 21600, true },
    [HUB_AC_E_DAY_S]    = { "e_day_s",    G_ENERGY, 0, 0, 21600, true },
};

static volatile float    s_val[HUB_AC_COUNT];
static volatile uint32_t s_gen;
static hub_area_t        s_area;

void hub_autocfg_load(void)
{
    for (int i = 0; i < HUB_AC_COUNT; i++) {
        s_val[i] = KEYS[i].def;
    }
    flora_store_t h;
    if (flora_store_open(FLORA_NVS_NS_AUTO, &h) != ESP_OK) {
        return;
    }
    for (int i = 0; i < HUB_AC_COUNT; i++) {
        float v = KEYS[i].def;
        flora_store_get_float(h, KEYS[i].key, &v, KEYS[i].def);
        /* Zakres mogl sie zmienic miedzy wersjami - wartosc spoza niego
         * wraca do domyslnej, zamiast zostac po cichu. */
        if (!isfinite(v) || v < KEYS[i].lo || v > KEYS[i].hi) {
            v = KEYS[i].def;
        }
        s_val[i] = v;
    }
    float a[4];
    size_t len = sizeof a;
    if (flora_store_get_blob(h, FLORA_NVS_AUTO_AREA, a, &len) == ESP_OK && len == sizeof a &&
        isfinite(a[0]) && isfinite(a[1]) && isfinite(a[2]) && isfinite(a[3]) &&
        a[2] > a[0] && a[3] > a[1]) {
        s_area = (hub_area_t){ .valid = true, .xmin = a[0], .ymin = a[1],
                               .xmax = a[2], .ymax = a[3] };
    }
    flora_store_close(h);

    const char *names[] = { "straznik", "pilot", "podlewanie", "misja", "sen", "klucz MDD" };
    char on[80] = "";
    for (int i = 0; i <= HUB_AC_MDD_KEY_EN; i++) {
        if (hub_autocfg_on((hub_ac_key_t)i)) {
            strlcat(on, on[0] ? ", " : "", sizeof on);
            strlcat(on, names[i], sizeof on);
        }
    }
    ESP_LOGI(TAG, "funkcje autonomiczne wlaczone: %s", on[0] ? on : "zadna");
}

float hub_autocfg_get(hub_ac_key_t key)
{
    return s_val[key];
}

bool hub_autocfg_on(hub_ac_key_t key)
{
    return s_val[key] >= 0.5f;
}

uint32_t hub_autocfg_generation(void)
{
    return s_gen;
}

void hub_autocfg_area(hub_area_t *out)
{
    *out = s_area;
}

esp_err_t hub_autocfg_set(const char *key, const char *value)
{
    int idx = -1;
    for (int i = 0; i < HUB_AC_COUNT; i++) {
        if (strcmp(key, KEYS[i].key) == 0) {
            idx = i;
            break;
        }
    }
    if (idx < 0) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    const bool erase = strcmp(value, "-") == 0;
    float v = KEYS[idx].def;
    if (!erase) {
        char *end;
        v = strtof(value, &end);
        if (end == value || *end || !isfinite(v) || v < KEYS[idx].lo || v > KEYS[idx].hi ||
            (KEYS[idx].integer && v != floorf(v))) {
            return ESP_ERR_INVALID_ARG;
        }
    }
    /* Pary, ktore musza zostac w porzadku. */
    const float dry  = idx == HUB_AC_W_DRY    ? v : s_val[HUB_AC_W_DRY];
    const float wet  = idx == HUB_AC_W_WET    ? v : s_val[HUB_AC_W_WET];
    const float save = idx == HUB_AC_E_V_SAVE ? v : s_val[HUB_AC_E_V_SAVE];
    const float surv = idx == HUB_AC_E_V_SURV ? v : s_val[HUB_AC_E_V_SURV];
    if (wet < dry + 5.0f || surv > save - 0.1f) {
        return ESP_ERR_INVALID_ARG;
    }

    flora_store_t h;
    esp_err_t err = flora_store_open(FLORA_NVS_NS_AUTO, &h);
    if (err != ESP_OK) {
        return err;
    }
    if (erase) {
        err = flora_store_has(h, key) ? flora_store_erase_key(h, key) : ESP_OK;
    } else {
        err = flora_store_set_float(h, key, v);
    }
    flora_store_close(h);
    if (err != ESP_OK) {
        return err;
    }
    s_val[idx] = v;
    s_gen = s_gen + 1;
    if (KEYS[idx].group == G_SWITCH) {
        ESP_LOGW(TAG, "%s = %d", key, v >= 0.5f);
    } else {
        ESP_LOGI(TAG, "%s = %g", key, v);
    }
    hub_autocfg_publish();
    return ESP_OK;
}

esp_err_t hub_autocfg_area_cmd(const char *arg)
{
    flora_store_t h;
    esp_err_t err;
    if (strcmp(arg, "-") == 0) {
        err = flora_store_open(FLORA_NVS_NS_AUTO, &h);
        if (err != ESP_OK) {
            return err;
        }
        err = flora_store_has(h, FLORA_NVS_AUTO_AREA)
              ? flora_store_erase_key(h, FLORA_NVS_AUTO_AREA) : ESP_OK;
        flora_store_close(h);
        if (err == ESP_OK) {
            s_area = (hub_area_t){ 0 };
            s_gen = s_gen + 1;
            ESP_LOGW(TAG, "obszar roboczy skasowany");
            hub_autocfg_publish();
        }
        return err;
    }

    float a[4];
    const char *p = arg;
    for (int i = 0; i < 4; i++) {
        char *end;
        a[i] = strtof(p, &end);
        if (end == p || !isfinite(a[i]) || fabsf(a[i]) > 100000.0f ||
            (i < 3 ? *end != ' ' : *end != 0)) {
            return ESP_ERR_INVALID_ARG;
        }
        p = end;
    }
    if (a[2] - a[0] < 1000.0f || a[3] - a[1] < 1000.0f) {
        return ESP_ERR_INVALID_ARG;
    }
    err = flora_store_open(FLORA_NVS_NS_AUTO, &h);
    if (err != ESP_OK) {
        return err;
    }
    err = flora_store_set_blob(h, FLORA_NVS_AUTO_AREA, a, sizeof a);
    flora_store_close(h);
    if (err == ESP_OK) {
        /* Struktura ma kilka slow, ale czyta ja tylko zadanie misji raz na
         * sekunde, a zmiana to reczna komenda - wyscig bez znaczenia. */
        s_area = (hub_area_t){ .valid = true, .xmin = a[0], .ymin = a[1],
                               .xmax = a[2], .ymax = a[3] };
        s_gen = s_gen + 1;
        ESP_LOGW(TAG, "obszar roboczy: x %.0f..%.0f, y %.0f..%.0f mm", a[0], a[2], a[1], a[3]);
        hub_autocfg_publish();
    }
    return err;
}

void hub_autocfg_publish(void)
{
    if (!flora_mqtt_is_connected()) {
        return;
    }
    for (int g = 0; g < G_COUNT; g++) {
        /* Osobny temat na grupe: bufor flora_mqtt ma 256 B. */
        char buf[224];
        int n = snprintf(buf, sizeof buf, "{");
        bool first = true;
        for (int i = 0; i < HUB_AC_COUNT && n < (int)sizeof buf; i++) {
            if (KEYS[i].group != g) {
                continue;
            }
            n += snprintf(buf + n, sizeof buf - n, "%s\"%s\":%g", first ? "" : ",",
                          KEYS[i].key, s_val[i]);
            first = false;
        }
        if (g == G_SEEK && n < (int)sizeof buf) {
            const hub_area_t a = s_area;
            if (a.valid) {
                n += snprintf(buf + n, sizeof buf - n, ",\"area\":[%.0f,%.0f,%.0f,%.0f]",
                              a.xmin, a.ymin, a.xmax, a.ymax);
            } else {
                n += snprintf(buf + n, sizeof buf - n, ",\"area\":null");
            }
        }
        if (n >= (int)sizeof buf - 1) {
            ESP_LOGE(TAG, "%s nie miesci sie w buforze", GROUP_TOPIC[g]);
            continue;
        }
        buf[n++] = '}';
        buf[n] = 0;
        flora_mqtt_publish_str(GROUP_TOPIC[g], buf, 1, true);
    }
}
