/**
 * @file hub_params.c
 * @brief Parametry geometrii i miejsca pracy, opis w hub_params.h.
 */
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "esp_log.h"

#include "hub_nav.h"
#include "hub_params.h"
#include "flora_board.h"
#include "flora_encoders.h"
#include "flora_mqtt.h"
#include "flora_store.h"

static const char *TAG = "parametry";

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static hub_params_t s_p;

/** Klucz MQTT = klucz NVS, zakres, gdzie w strukturze i ktora flaga "_set". */
typedef enum { F_TRACK, F_WHEEL, F_RATIO, F_CAM_X, F_CAM_Y, F_LAT, F_LON, F_DECL, F_CAM_YAW, F_CAM_H, F_TAG_H } field_t;

static const struct {
    const char *key;
    float       lo, hi;
    field_t     f;
} KEYS[] = {
    { FLORA_NVS_TRACK_MM,    250.0f,  600.0f, F_TRACK },
    { FLORA_NVS_WHEEL_MM,     80.0f,  200.0f, F_WHEEL },
    { FLORA_NVS_WHEEL_RATIO,   0.9f,    1.1f, F_RATIO },
    { FLORA_NVS_CAM_X_MM,   -500.0f,  800.0f, F_CAM_X },
    { FLORA_NVS_CAM_Y_MM,   -300.0f,  300.0f, F_CAM_Y },
    { FLORA_NVS_SITE_LAT,    -90.0f,   90.0f, F_LAT },
    { FLORA_NVS_SITE_LON,   -180.0f,  180.0f, F_LON },
    { FLORA_NVS_MAG_DECL,    -30.0f,   30.0f, F_DECL },
    /* Na koncu: KEYS[] indeksowane jest kolejnoscia field_t. */
    { FLORA_NVS_CAM_YAW,     -45.0f,   45.0f, F_CAM_YAW },
    { FLORA_NVS_CAM_H,         0.0f, 2000.0f, F_CAM_H },
    { FLORA_NVS_TAG_H,         0.0f, 2000.0f, F_TAG_H },
};
#define KEY_COUNT (sizeof KEYS / sizeof KEYS[0])

static float default_of(field_t f)
{
    switch (f) {
        case F_TRACK: return FLORA_TRACK_MM;
        case F_WHEEL: return FLORA_WHEEL_DIAMETER_MM;
        case F_RATIO: return 1.0f;
        case F_CAM_X: return FLORA_NAV_CAM_X_MM;
        case F_CAM_Y: return FLORA_NAV_CAM_Y_MM;
        case F_LAT:   return (float)FLORA_SITE_LAT_DEG;
        case F_LON:   return (float)FLORA_SITE_LON_DEG;
        case F_DECL:  return FLORA_MAG_DECLINATION_DEG;
        case F_CAM_YAW: return FLORA_NAV_CAM_YAW_DEG;
        case F_CAM_H:   return FLORA_NAV_CAM_H_MM;
        case F_TAG_H:   return FLORA_NAV_TAG_H_MM;
    }
    return 0.0f;
}

static void put(hub_params_t *p, field_t f, float v)
{
    switch (f) {
        case F_TRACK: p->track_mm    = v; break;
        case F_WHEEL: p->wheel_mm    = v; break;
        case F_RATIO: p->wheel_ratio = v; break;
        case F_CAM_X: p->cam_x_mm    = v; break;
        case F_CAM_Y: p->cam_y_mm    = v; break;
        case F_LAT:   p->lat_deg     = v; break;
        case F_LON:   p->lon_deg     = v; break;
        case F_DECL:  p->decl_deg    = v; break;
        case F_CAM_YAW: p->cam_yaw_deg = v; break;
        case F_CAM_H:   p->cam_h_mm    = v; break;
        case F_TAG_H:   p->tag_h_mm    = v; break;
    }
}

/** Czyta wszystko z NVS do nowej struktury (bez blokady - NVS bywa wolne). */
static hub_params_t read_all(void)
{
    hub_params_t p = { 0 };
    flora_store_t h;
    bool open = flora_store_open(FLORA_NVS_NS_CALIB, &h) == ESP_OK;
    for (size_t i = 0; i < KEY_COUNT; i++) {
        float v = default_of(KEYS[i].f);
        bool has = open && flora_store_has(h, KEYS[i].key);
        if (has) {
            flora_store_get_float(h, KEYS[i].key, &v, v);
        }
        put(&p, KEYS[i].f, v);
        switch (KEYS[i].f) {
            case F_TRACK:                  p.track_set |= has; break;
            case F_WHEEL: case F_RATIO:    p.wheel_set |= has; break;
            case F_CAM_X: case F_CAM_Y: case F_CAM_YAW:
            case F_CAM_H: case F_TAG_H:    p.cam_set |= has; break;
            case F_LAT: case F_LON:        p.site_set  |= has; break;
            case F_DECL:                   break;
        }
    }
    if (open) {
        flora_store_close(h);
    }
    return p;
}

void hub_params_mm_per_count(const hub_params_t *p, float *left, float *right)
{
    /* Srednia srednica D i stosunek r = dP / dL: dL = 2D / (1 + r), dP = r dL. */
    float d_l = 2.0f * p->wheel_mm / (1.0f + p->wheel_ratio);
    float d_r = d_l * p->wheel_ratio;
    const float counts = FLORA_COUNTS_PER_WHEEL_REV;
    *left  = 3.14159265f * d_l / counts;
    *right = 3.14159265f * d_r / counts;
}

/** Wczytanie i rozeslanie do uzytkownikow (enkodery, EKF). */
static void reload(void)
{
    hub_params_t p = read_all();
    portENTER_CRITICAL(&s_lock);
    s_p = p;
    portEXIT_CRITICAL(&s_lock);

    float l, r;
    hub_params_mm_per_count(&p, &l, &r);
    /* Enkodery moga nie dzialac (gola plytka) - wtedy nic do zmiany. */
    flora_encoders_set_mm_per_count(l, r);
    hub_nav_params_changed();
}

void hub_params_load(void)
{
    hub_params_t p = read_all();
    portENTER_CRITICAL(&s_lock);
    s_p = p;
    portEXIT_CRITICAL(&s_lock);
    ESP_LOGI(TAG, "rozstaw %.1f mm%s, kolo %.2f mm (P/L %.4f)%s, kamera (%.0f, %.0f) mm, "
                  "obrot %.1f st%s",
             p.track_mm, p.track_set ? "" : " [domyslny]",
             p.wheel_mm, p.wheel_ratio, p.wheel_set ? "" : " [domyslne]",
             p.cam_x_mm, p.cam_y_mm, p.cam_yaw_deg, p.cam_set ? "" : " [domyslna]");
}

void hub_params_get(hub_params_t *out)
{
    portENTER_CRITICAL(&s_lock);
    *out = s_p;
    portEXIT_CRITICAL(&s_lock);
}

static esp_err_t store_float(const char *key, float v, bool erase)
{
    flora_store_t h;
    esp_err_t err = flora_store_open(FLORA_NVS_NS_CALIB, &h);
    if (err == ESP_OK) {
        err = erase ? flora_store_erase_key(h, key) : flora_store_set_float(h, key, v);
        flora_store_close(h);
    }
    return err;
}

esp_err_t hub_params_set(const char *key, const char *value)
{
    size_t i;
    for (i = 0; i < KEY_COUNT; i++) {
        if (strcmp(key, KEYS[i].key) == 0) {
            break;
        }
    }
    if (i == KEY_COUNT) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    bool erase = strcmp(value, "-") == 0;
    float v = 0.0f;
    if (!erase) {
        char *end;
        v = strtof(value, &end);
        if (end == value || *end || !isfinite(v) || v < KEYS[i].lo || v > KEYS[i].hi) {
            return ESP_ERR_INVALID_ARG;
        }
    }
    esp_err_t err = store_float(KEYS[i].key, v, erase);
    if (err == ESP_OK) {
        reload();
        hub_params_publish();
    }
    return err;
}

esp_err_t hub_params_save_track(float track_mm)
{
    if (!(track_mm >= KEYS[F_TRACK].lo && track_mm <= KEYS[F_TRACK].hi)) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = store_float(FLORA_NVS_TRACK_MM, track_mm, false);
    if (err == ESP_OK) {
        reload();
        hub_params_publish();
    }
    return err;
}

esp_err_t hub_params_save_wheels(float wheel_mm, float wheel_ratio)
{
    if (!(wheel_mm >= KEYS[F_WHEEL].lo && wheel_mm <= KEYS[F_WHEEL].hi) ||
        !(wheel_ratio >= KEYS[F_RATIO].lo && wheel_ratio <= KEYS[F_RATIO].hi)) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = store_float(FLORA_NVS_WHEEL_MM, wheel_mm, false);
    if (err == ESP_OK) {
        err = store_float(FLORA_NVS_WHEEL_RATIO, wheel_ratio, false);
    }
    if (err == ESP_OK) {
        reload();
        hub_params_publish();
    }
    return err;
}

void hub_params_publish(void)
{
    hub_params_t p;
    hub_params_get(&p);
    /* Wlasny bufor: tresc nie miesci sie w 256 B publish_fmt. */
    char buf[400];
    snprintf(buf, sizeof buf,
             "{\"track_mm\":%.1f,\"track_set\":%s,\"wheel_mm\":%.2f,"
             "\"wheel_ratio\":%.4f,\"wheel_set\":%s,\"cam_x_mm\":%.0f,"
             "\"cam_y_mm\":%.0f,\"cam_yaw_deg\":%.1f,\"cam_h_mm\":%.0f,\"tag_h_mm\":%.0f,"
             "\"cam_set\":%s,\"site_lat\":%.5f,"
             "\"site_lon\":%.5f,\"site_set\":%s,\"mag_decl\":%.1f,\"tags\":%d}",
             p.track_mm, p.track_set ? "true" : "false",
             p.wheel_mm, p.wheel_ratio, p.wheel_set ? "true" : "false",
             p.cam_x_mm, p.cam_y_mm, p.cam_yaw_deg, p.cam_h_mm, p.tag_h_mm,
             p.cam_set ? "true" : "false",
             p.lat_deg, p.lon_deg, p.site_set ? "true" : "false",
             p.decl_deg, hub_nav_tag_count());
    flora_mqtt_publish_str("config_nav", buf, 1, true);
}
