/**
 * @file hub_rec.c
 * @brief Nagrywanie przejazdu, opis w hub_rec.h.
 *
 * Pisze kilka zadan (nav 50 Hz, polecenia MQTT przy resecie pozy i zmianie
 * parametrow), a StreamBuffer FreeRTOS jest bezpieczny tylko dla jednego
 * pisarza - stad mutex. Pisarz nigdy nie czeka na miejsce: pelny bufor
 * to zgubiona linia i licznik, nie zatrzymana nawigacja.
 */
#include <inttypes.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/stream_buffer.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"

#include "hub_nav.h"
#include "hub_params.h"
#include "hub_rec.h"
#include "flora_mqtt.h"
#include "flora_navlog.h"

static const char *TAG = "nagranie";

#define BUF_BYTES     12288   /**< ~6 s nagrania, gdy broker chwilowo nie odbiera */
#define MSG_BYTES     1400    /**< jedna wiadomosc MQTT - ponizej typowego MTU */
#define FLUSH_MS      200
#define STATUS_EVERY  25      /**< tele/rec co 25 oproznien = 5 s */
#define REC_LINE_MAX      160

static struct {
    StreamBufferHandle_t sb;
    SemaphoreHandle_t    wlock;
    volatile bool        active;
    uint32_t             seq;
    uint32_t             drops;
    uint32_t             lines;
    uint32_t             sent_bytes;
} s_r;

bool hub_rec_active(void)
{
    return s_r.active;
}

static void put(const char *line, size_t n)
{
    xSemaphoreTake(s_r.wlock, portMAX_DELAY);
    if (xStreamBufferSpacesAvailable(s_r.sb) >= n) {
        xStreamBufferSend(s_r.sb, line, n, 0);
        s_r.lines++;
    } else {
        s_r.drops++;   /* cala linia albo nic - polowka zepsulaby parser */
    }
    xSemaphoreGive(s_r.wlock);
}

void hub_rec_printf(const char *fmt, ...)
{
    if (!s_r.active) {
        return;
    }
    char line[REC_LINE_MAX];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(line, sizeof line, fmt, ap);
    va_end(ap);
    if (n <= 0 || n >= (int)sizeof line) {
        s_r.drops++;
        return;
    }
    put(line, (size_t)n);
}

void hub_rec_header(void)
{
    if (!s_r.active) {
        return;
    }
    hub_params_t p;
    hub_params_get(&p);
    float mm_l, mm_r;
    hub_params_mm_per_count(&p, &mm_l, &mm_r);
    hub_rec_printf(FLORA_NAVLOG_FMT_P, mm_l, mm_r, p.track_mm, p.cam_x_mm, p.cam_y_mm,
                   p.lat_deg, p.lon_deg, p.decl_deg);
    hub_nav_rec_tags();
}

static void publish_status(void)
{
    flora_mqtt_publish_fmt("tele/rec", 0, false,
                           "{\"active\":%s,\"seq\":%" PRIu32 ",\"lines\":%" PRIu32
                           ",\"drops\":%" PRIu32 ",\"kb\":%" PRIu32 "}",
                           s_r.active ? "true" : "false", s_r.seq, s_r.lines, s_r.drops,
                           s_r.sent_bytes / 1024);
}

static void rec_task(void *arg)
{
    /* Bufor wiadomosci: "#<seq>\n" + pelne linie. Niepelna koncowka czeka
     * na nastepne oproznienie w `carry`. */
    static char msg[MSG_BYTES + 16];
    static char carry[REC_LINE_MAX];
    size_t carry_n = 0;
    uint32_t tick = 0;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(FLUSH_MS));
        if (++tick % STATUS_EVERY == 0) {
            publish_status();
        }
        for (;;) {
            int head = snprintf(msg, 16, "#%" PRIu32 "\n", s_r.seq);
            size_t n = head;
            memcpy(msg + n, carry, carry_n);
            n += carry_n;
            n += xStreamBufferReceive(s_r.sb, msg + n, MSG_BYTES - n, 0);
            if (n == (size_t)head) {
                break;   /* nic nowego */
            }
            /* Tnij na ostatnim koncu linii; reszta do nastepnej paczki. */
            size_t cut = n;
            while (cut > (size_t)head && msg[cut - 1] != '\n') cut--;
            carry_n = n - cut;
            memcpy(carry, msg + cut, carry_n);
            if (cut == (size_t)head) {
                break;   /* sama niepelna linia - czekamy na reszte */
            }
            if (flora_mqtt_publish("rec/data", msg, cut, 0, false) == ESP_OK) {
                s_r.sent_bytes += cut;
            } else {
                s_r.drops++;
            }
            s_r.seq++;   /* tez przy bledzie - luka w numerach to zgubiona paczka */
            if (cut < MSG_BYTES - REC_LINE_MAX) {
                break;   /* bufor oprozniony */
            }
        }
    }
}

esp_err_t hub_rec_init(void)
{
    s_r.sb    = xStreamBufferCreate(BUF_BYTES, 1);
    s_r.wlock = xSemaphoreCreateMutex();
    ESP_RETURN_ON_FALSE(s_r.sb && s_r.wlock, ESP_ERR_NO_MEM, TAG, "brak pamieci na bufor");
    ESP_RETURN_ON_FALSE(xTaskCreate(rec_task, "rec", 3072, NULL, 3, NULL) == pdPASS,
                        ESP_ERR_NO_MEM, TAG, "zadanie rec");
    return ESP_OK;
}

esp_err_t hub_rec_cmd(const char *arg)
{
    ESP_RETURN_ON_FALSE(s_r.sb, ESP_ERR_INVALID_STATE, TAG, "rekorder nie wystartowal");
    if (strcmp(arg, "on") == 0) {
        if (!s_r.active) {
            s_r.drops = s_r.lines = s_r.sent_bytes = 0;
            s_r.active = true;
            hub_rec_printf(FLORA_NAVLOG_FMT_V, FLORA_NAVLOG_VERSION);
            hub_rec_header();
            ESP_LOGI(TAG, "start, wiadomosci od #%" PRIu32, s_r.seq);
        }
    } else if (strcmp(arg, "off") == 0) {
        if (s_r.active) {
            s_r.active = false;
            ESP_LOGI(TAG, "stop: %" PRIu32 " linii, %" PRIu32 " zgubionych", s_r.lines, s_r.drops);
        }
    } else {
        return ESP_ERR_INVALID_ARG;
    }
    publish_status();
    return ESP_OK;
}
