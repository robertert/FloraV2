/**
 * @file hub_remote.c
 * @brief Komendy z brokera (cmd/...): naped, pompka, zasilanie masztu.
 *
 * DEAD-MAN NAPEDU: kierunek jazdy obowiazuje tylko DRIVE_HOLD_MS od
 * ostatniej komendy. Panel, dopoki kierunek jest wlaczony, powtarza ja
 * co kilkaset ms. Bez tego zerwanie WiFi w trakcie jazdy oznaczaloby
 * robota jadacego przed siebie az do utyku - a straznik utyku reaguje
 * dopiero na zablokowane kolo, nie na zjazd z trawnika.
 *
 * Pompka nie potrzebuje wlasnego dead-mana: flora_pump sam ucina dawke
 * po max_run_ms i pilnuje limitu dobowego oraz poziomu wody.
 */
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"

#include "hub_remote.h"
#include "hub_telemetry.h"
#include "flora_board.h"
#include "flora_motors.h"
#include "flora_mqtt.h"
#include "flora_pump.h"
#include "flora_pwrsw.h"

static const char *TAG = "zdalne";

#define DRIVE_HOLD_MS      1500
#define DRIVE_POWER_DEFAULT  30
#define DEADMAN_CHECK_MS    100

/* Zapis z zadania MQTT, odczyt z timera - pojedyncze slowa, bez blokady. */
static volatile int     s_drive_power = DRIVE_POWER_DEFAULT;
static volatile int64_t s_drive_last_us;
static volatile bool    s_driving;

static esp_timer_handle_t s_deadman;

static void ack(const char *cmd, esp_err_t err)
{
    char topic[48];
    snprintf(topic, sizeof topic, "ack/%s", cmd);
    flora_mqtt_publish_str(topic, err == ESP_OK ? "ok" : esp_err_to_name(err), 0, false);
}

static void deadman_check(void *arg)
{
    if (s_driving && esp_timer_get_time() - s_drive_last_us > DRIVE_HOLD_MS * 1000LL) {
        s_driving = false;
        flora_motors_stop();
        ESP_LOGW(TAG, "brak odswiezenia cmd/drive przez %d ms - stop", DRIVE_HOLD_MS);
    }
}

static esp_err_t cmd_drive(const char *arg)
{
    float p = (float)s_drive_power;
    float l, r;
    if      (strcmp(arg, "fwd")   == 0) { l =  p; r =  p; }
    else if (strcmp(arg, "back")  == 0) { l = -p; r = -p; }
    else if (strcmp(arg, "left")  == 0) { l = -p; r =  p; }
    else if (strcmp(arg, "right") == 0) { l =  p; r = -p; }
    else if (strcmp(arg, "stop")  == 0) {
        s_driving = false;
        return flora_motors_stop();
    } else {
        return ESP_ERR_INVALID_ARG;
    }

    /* Znacznik przed zadaniem mocy, zeby timer nie zatrzymal swiezej komendy. */
    s_drive_last_us = esp_timer_get_time();
    esp_err_t err = flora_motors_set(l, r);
    s_driving = (err == ESP_OK);
    return err;
}

static esp_err_t cmd_drive_power(const char *arg)
{
    char *end;
    long v = strtol(arg, &end, 10);
    if (end == arg || *end || v < 0 || v > 100) {
        return ESP_ERR_INVALID_ARG;
    }
    s_drive_power = (int)v;
    return ESP_OK;
}

static esp_err_t cmd_pump(const char *arg)
{
    if (strcmp(arg, "off") == 0) {
        return flora_pump_stop();
    }
    if (strcmp(arg, "on") == 0) {
        /* Dokladnie max_run_ms, nie "nieskonczonosc" do przyciecia: limit
         * dobowy flora_pump liczy zadany czas przed przycieciem, wiec za
         * duza wartosc zostalaby odrzucona jako przekroczenie limitu. */
        return flora_pump_run_ms(HUB_PUMP_MAX_RUN_MS);
    }
    char *end;
    unsigned long ms = strtoul(arg, &end, 10);
    if (end == arg || *end || ms == 0 || ms > HUB_PUMP_MAX_RUN_MS) {
        return ESP_ERR_INVALID_ARG;
    }
    return flora_pump_run_ms((uint32_t)ms);
}

static esp_err_t cmd_mast(const char *arg)
{
    flora_pwrsw_handle_t h = hub_telemetry_mast();
    if (!h) {
        return ESP_ERR_INVALID_STATE;
    }
    if (strcmp(arg, "on") == 0)  { return flora_pwrsw_on(h); }
    if (strcmp(arg, "off") == 0) { return flora_pwrsw_off(h); }
    return ESP_ERR_INVALID_ARG;
}

static void on_cmd(const char *topic, size_t topic_len,
                   const char *data, size_t data_len, void *ctx)
{
    /* Temat jest pelny, z prefiksem: <base>/cmd/<nazwa>. */
    const size_t prefix = strlen(FLORA_MQTT_BASE_TOPIC "/cmd/");
    if (topic_len <= prefix || topic_len - prefix >= 32) {
        return;
    }
    char cmd[32];
    memcpy(cmd, topic + prefix, topic_len - prefix);
    cmd[topic_len - prefix] = 0;

    /* Dane nie sa zakonczone zerem; przycinamy tez biale znaki z konca,
     * bo mosquitto_pub -m z echo latwo dokleja znak nowej linii. */
    char arg[32];
    size_t n = data_len < sizeof arg - 1 ? data_len : sizeof arg - 1;
    memcpy(arg, data, n);
    while (n > 0 && (arg[n - 1] == '\n' || arg[n - 1] == '\r' || arg[n - 1] == ' ')) {
        n--;
    }
    arg[n] = 0;

    esp_err_t err;
    if      (strcmp(cmd, "drive") == 0)       { err = cmd_drive(arg); }
    else if (strcmp(cmd, "drive_power") == 0) { err = cmd_drive_power(arg); }
    else if (strcmp(cmd, "pump") == 0)        { err = cmd_pump(arg); }
    else if (strcmp(cmd, "mast") == 0)        { err = cmd_mast(arg); }
    else if (strcmp(cmd, "stall_clear") == 0) { err = flora_motors_clear_stall(); }
    else                                      { err = ESP_ERR_NOT_SUPPORTED; }

    /* Powtarzane co 300 ms cmd/drive nie zasmieca logu. */
    if (err != ESP_OK || strcmp(cmd, "drive") != 0) {
        ESP_LOGI(TAG, "%s = \"%s\" -> %s", cmd, arg, esp_err_to_name(err));
    }
    ack(cmd, err);
}

esp_err_t hub_remote_start(void)
{
    const esp_timer_create_args_t args = {
        .callback = deadman_check,
        .name     = "deadman",
    };
    esp_err_t err = esp_timer_create(&args, &s_deadman);
    if (err == ESP_OK) {
        err = esp_timer_start_periodic(s_deadman, DEADMAN_CHECK_MS * 1000);
    }
    if (err != ESP_OK) {
        /* Bez dead-mana nie przyjmujemy komend - naped bez niego jest
         * niebezpieczny, a reszta komend nie jest warta tego ryzyka. */
        ESP_LOGE(TAG, "timer dead-man: %s - sterowanie wylaczone", esp_err_to_name(err));
        return err;
    }
    return flora_mqtt_subscribe("cmd/#", 1, on_cmd, NULL);
}
