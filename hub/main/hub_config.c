/**
 * @file hub_config.c
 * @brief Ustawienia bryly zapisywane do NVS przez MQTT.
 *
 * Te same klucze i przestrzenie co w flora_board.h, wiec reszta kodu
 * (bringup_net_connect, inicjalizacja gleby) czyta je bez zmian. Wartosci
 * wkompilowane zostaja domyslnymi, gdy klucza w NVS brak.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"

#include "hub_config.h"
#include "flora_board.h"
#include "flora_mqtt.h"
#include "flora_soil.h"
#include "flora_store.h"

static const char *TAG = "config";

/** "-" kasuje klucz, czyli przywraca wartosc wkompilowana - tak wraca sie
 *  np. z brokera na serwerze (z loginem) do brokera na laptopie (bez). */
static esp_err_t set_str(const char *ns, const char *key, const char *value, size_t max)
{
    bool erase = strcmp(value, "-") == 0;
    if (!erase && (value[0] == 0 || strlen(value) > max)) {
        return ESP_ERR_INVALID_ARG;
    }
    flora_store_t h;
    esp_err_t err = flora_store_open(ns, &h);
    if (err == ESP_OK) {
        err = erase ? flora_store_erase_key(h, key) : flora_store_set_str(h, key, value);
        flora_store_close(h);
    }
    return err;
}

/** Zapis jednego punktu kalibracji gleby i natychmiastowe uzycie obu. */
static esp_err_t set_soil(const char *key, const char *value)
{
    char *end;
    long v = strtol(value, &end, 10);
    if (end == value || *end || v < 0 || v > 4095) {
        return ESP_ERR_INVALID_ARG;
    }
    flora_store_t h;
    esp_err_t err = flora_store_open(FLORA_NVS_NS_CALIB, &h);
    if (err != ESP_OK) {
        return err;
    }
    int32_t dry = FLORA_SOIL_RAW_DRY, wet = FLORA_SOIL_RAW_WET;
    err = flora_store_set_i32(h, key, (int32_t)v);
    flora_store_get_i32(h, FLORA_NVS_SOIL_DRY, &dry, FLORA_SOIL_RAW_DRY);
    flora_store_get_i32(h, FLORA_NVS_SOIL_WET, &wet, FLORA_SOIL_RAW_WET);
    flora_store_close(h);

    /* Czujnik moze byc niezainicjalizowany (gola plytka) - zapis w NVS
     * i tak sie liczy, zadziala przy nastepnym starcie. */
    if (err == ESP_OK && flora_soil_set_calibration(dry, wet) != ESP_OK) {
        ESP_LOGW(TAG, "kalibracja zapisana, czujnik gleby niedostepny - zadziala po restarcie");
    }
    return err;
}

esp_err_t hub_config_set(const char *key, const char *value)
{
    esp_err_t err;
    if (strcmp(key, "wifi_ssid") == 0) {
        err = set_str(FLORA_NVS_NS_NET, FLORA_NVS_WIFI_SSID, value, 32);
    } else if (strcmp(key, "wifi_pass") == 0) {
        err = set_str(FLORA_NVS_NS_NET, FLORA_NVS_WIFI_PASS, value, 64);
    } else if (strcmp(key, "mqtt_uri") == 0) {
        /* Tylko schematy, ktore ESP-MQTT naprawde obsluzy - literowka typu
         * "mqtt:/host" wychodzilaby dopiero po restarcie, jako brak brokera. */
        if (strcmp(value, "-") != 0 &&
            strncmp(value, "mqtt://", 7) != 0 && strncmp(value, "mqtts://", 8) != 0) {
            return ESP_ERR_INVALID_ARG;
        }
        err = set_str(FLORA_NVS_NS_NET, FLORA_NVS_MQTT_URI, value, 95);
    } else if (strcmp(key, "mqtt_user") == 0) {
        err = set_str(FLORA_NVS_NS_NET, FLORA_NVS_MQTT_USER, value, 32);
    } else if (strcmp(key, "mqtt_pass") == 0) {
        err = set_str(FLORA_NVS_NS_NET, FLORA_NVS_MQTT_PASS, value, 64);
    } else if (strcmp(key, "soil_dry") == 0) {
        err = set_soil(FLORA_NVS_SOIL_DRY, value);
    } else if (strcmp(key, "soil_wet") == 0) {
        err = set_soil(FLORA_NVS_SOIL_WET, value);
    } else {
        return ESP_ERR_NOT_SUPPORTED;
    }

    if (err == ESP_OK) {
        /* Haslo nie trafia do logu, reszta tak. */
        ESP_LOGI(TAG, "zapisano %s = %s", key,
                 strstr(key, "_pass") ? "***" : value);
        hub_config_publish();
    }
    return err;
}

void hub_config_publish(void)
{
    char ssid[33], uri[96], muser[33];
    int32_t dry = FLORA_SOIL_RAW_DRY, wet = FLORA_SOIL_RAW_WET;

    strncpy(ssid, FLORA_WIFI_SSID_DEFAULT, sizeof ssid - 1); ssid[sizeof ssid - 1] = 0;
    strncpy(uri, FLORA_MQTT_URI_DEFAULT, sizeof uri - 1);    uri[sizeof uri - 1] = 0;
    strncpy(muser, FLORA_MQTT_USER_DEFAULT, sizeof muser - 1); muser[sizeof muser - 1] = 0;

    flora_store_t h;
    if (flora_store_open(FLORA_NVS_NS_NET, &h) == ESP_OK) {
        flora_store_get_str(h, FLORA_NVS_WIFI_SSID, ssid, sizeof ssid, FLORA_WIFI_SSID_DEFAULT);
        flora_store_get_str(h, FLORA_NVS_MQTT_URI, uri, sizeof uri, FLORA_MQTT_URI_DEFAULT);
        flora_store_get_str(h, FLORA_NVS_MQTT_USER, muser, sizeof muser, FLORA_MQTT_USER_DEFAULT);
        flora_store_close(h);
    }
    if (flora_store_open(FLORA_NVS_NS_CALIB, &h) == ESP_OK) {
        flora_store_get_i32(h, FLORA_NVS_SOIL_DRY, &dry, FLORA_SOIL_RAW_DRY);
        flora_store_get_i32(h, FLORA_NVS_SOIL_WET, &wet, FLORA_SOIL_RAW_WET);
        flora_store_close(h);
    }

    /* SSID moze zawierac cudzyslow - zwykly %s zepsulby JSON. Zamieniamy
     * na apostrof: to tylko podglad, wartosc w NVS zostaje nietknieta. */
    for (char *c = ssid; *c; c++) {
        if (*c == '"' || *c == '\\') { *c = '\''; }
    }
    for (char *c = muser; *c; c++) {
        if (*c == '"' || *c == '\\') { *c = '\''; }
    }
    for (char *c = uri; *c; c++) {
        if (*c == '"' || *c == '\\') { *c = '\''; }
    }
    /* Retain: panel otwarty pozniej od razu widzi ustawienia. */
    flora_mqtt_publish_fmt("config", 1, true,
                           "{\"wifi_ssid\":\"%s\",\"mqtt_uri\":\"%s\",\"mqtt_user\":\"%s\","
                           "\"soil_dry\":%ld,\"soil_wet\":%ld}",
                           ssid, uri, muser, (long)dry, (long)wet);
}
