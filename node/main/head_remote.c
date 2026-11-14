/**
 * @file head_remote.c
 * @brief Komendy serwisowe glowicy - patrz head_remote.h.
 */
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"

#include "head_remote.h"
#include "flora_board.h"
#include "flora_diag.h"
#include "flora_mqtt.h"
#include "flora_ota.h"
#include "flora_store.h"

static const char *TAG = "zdalne";

#define CMD_PREFIX FLORA_MQTT_BASE_TOPIC "/headcmd/"

static esp_timer_handle_t s_reboot;

static void ack(const char *cmd, esp_err_t err)
{
    char topic[48];
    snprintf(topic, sizeof topic, "headack/%s", cmd);
    flora_mqtt_publish_str(topic, err == ESP_OK ? "ok" : esp_err_to_name(err), 0, false);
}

static void reboot_now(void *arg)
{
    esp_restart();
}

/** Restart z opoznieniem, zeby ack zdazyl wyjsc do brokera. */
static esp_err_t cmd_reboot(void)
{
    const esp_timer_create_args_t args = { .callback = reboot_now, .name = "reboot" };
    if (!s_reboot) {
        esp_err_t err = esp_timer_create(&args, &s_reboot);
        if (err != ESP_OK) {
            return err;
        }
    }
    ESP_LOGW(TAG, "restart na zadanie z brokera");
    esp_timer_stop(s_reboot);   /* powtorzona komenda nie przesuwa restartu */
    return esp_timer_start_once(s_reboot, 500 * 1000);
}

/** "-" kasuje klucz, czyli przywraca wartosc wkompilowana. */
static esp_err_t set_str(const char *key, const char *value, size_t max)
{
    bool erase = strcmp(value, "-") == 0;
    if (!erase && (value[0] == 0 || strlen(value) > max)) {
        return ESP_ERR_INVALID_ARG;
    }
    flora_store_t h;
    esp_err_t err = flora_store_open(FLORA_NVS_NS_NET, &h);
    if (err == ESP_OK) {
        err = erase ? flora_store_erase_key(h, key) : flora_store_set_str(h, key, value);
        flora_store_close(h);
    }
    /* Skasowanie klucza, ktorego nie bylo, to tez "przywrocone domyslne". */
    if (erase && err == ESP_ERR_NOT_FOUND) {
        err = ESP_OK;
    }
    return err;
}

/** Ustawienia sieci glowicy. Te same klucze i limity co w bryle (hub_config.c). */
static esp_err_t cmd_config(const char *key, const char *value)
{
    if (strcmp(key, "wifi_ssid") == 0) { return set_str(FLORA_NVS_WIFI_SSID, value, 32); }
    if (strcmp(key, "wifi_pass") == 0) { return set_str(FLORA_NVS_WIFI_PASS, value, 64); }
    if (strcmp(key, "mqtt_user") == 0) { return set_str(FLORA_NVS_MQTT_USER, value, 32); }
    if (strcmp(key, "mqtt_pass") == 0) { return set_str(FLORA_NVS_MQTT_PASS, value, 64); }
    if (strcmp(key, "mqtt_uri") == 0) {
        /* Tylko schematy, ktore ESP-MQTT obsluzy - literowka wyszlaby
         * dopiero po restarcie, jako brak brokera. */
        if (strcmp(value, "-") != 0 &&
            strncmp(value, "mqtt://", 7) != 0 && strncmp(value, "mqtts://", 8) != 0) {
            return ESP_ERR_INVALID_ARG;
        }
        return set_str(FLORA_NVS_MQTT_URI, value, 95);
    }
    return ESP_ERR_NOT_SUPPORTED;
}

static void on_cmd(const char *topic, size_t topic_len,
                   const char *data, size_t data_len, void *ctx)
{
    /* Temat jest pelny, z prefiksem: <base>/headcmd/<nazwa>. */
    const size_t prefix = strlen(CMD_PREFIX);
    if (topic_len <= prefix || topic_len - prefix >= 32) {
        return;
    }
    char cmd[32];
    memcpy(cmd, topic + prefix, topic_len - prefix);
    cmd[topic_len - prefix] = 0;

    /* Dane nie sa zakonczone zerem; przycinamy biale znaki z konca, bo
     * mosquitto_pub latwo dokleja znak nowej linii. Bufor miesci najdluzsza
     * wartosc ustawien (mqtt_uri, 95 znakow). */
    char arg[100];
    size_t n = data_len < sizeof arg - 1 ? data_len : sizeof arg - 1;
    memcpy(arg, data, n);
    while (n > 0 && (arg[n - 1] == '\n' || arg[n - 1] == '\r' || arg[n - 1] == ' ')) {
        n--;
    }
    arg[n] = 0;

    esp_err_t err;
    if      (strcmp(cmd, "log") == 0)         { err = flora_diag_set_level(arg); }
    else if (strcmp(cmd, "reboot") == 0)      { err = cmd_reboot(); }
    else if (strcmp(cmd, "ota") == 0)         { err = flora_ota_start(arg); }
    else if (strncmp(cmd, "config/", 7) == 0) { err = cmd_config(cmd + 7, arg); }
    else                                      { err = ESP_ERR_NOT_SUPPORTED; }

    ESP_LOGI(TAG, "%s = \"%s\" -> %s", cmd,
             strstr(cmd, "_pass") ? "***" : arg, esp_err_to_name(err));
    ack(cmd, err);
}

esp_err_t head_remote_start(void)
{
    return flora_mqtt_subscribe("headcmd/#", 1, on_cmd, NULL);
}
