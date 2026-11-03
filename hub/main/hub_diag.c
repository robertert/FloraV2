/**
 * @file hub_diag.c
 * @brief Logi przez MQTT i informacja o starcie.
 *
 * LOGI: esp_log_set_vprintf() przechwytuje kazda linie logu. Konsola USB
 * dostaje ja bez zmian, a linie W i E dodatkowo trafiaja do kolejki, z ktorej
 * osobne zadanie publikuje je na tele/log. Samo przechwycenie nigdy nie
 * czeka na siec: logi pisze tez zadanie lacza z glowica czy timer pompki,
 * a im nie wolno blokowac sie na brokerze.
 *
 * Kolejka trzyma linie takze bez polaczenia, wiec bledy z inicjalizacji i
 * z okresu bez sieci (np. dlaczego WiFi padlo) wychodza po polaczeniu.
 * Pelna kolejka gubi nowe linie i liczy je - pomijanie jest jawne.
 *
 * PETLA LOGOW: publikacja, ktora sie nie uda, sama loguje blad (flora_mqtt).
 * Linie z wlasnego zadania sa wiec pomijane - inaczej kazdy blad publikacji
 * rodzilby kolejny.
 */
#include <inttypes.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_system.h"

#include "hub_diag.h"
#include "flora_board.h"
#include "flora_mqtt.h"
#include "flora_store.h"

static const char *TAG = "diag";

#define LOG_LINE_MAX      160   /* dluzsze linie sa przycinane */
#define QUEUE_LEN      32
#define SEND_GAP_MS    50   /* najwyzej ~20 linii/s, zeby petla bledow nie zalala brokera */

typedef struct {
    char line[LOG_LINE_MAX];
} log_line_t;

static QueueHandle_t   s_queue;
static TaskHandle_t    s_task;
static vprintf_like_t  s_orig_vprintf;
static volatile uint32_t s_dropped;

static esp_reset_reason_t s_reset;
static uint32_t           s_boots;

/* ================= PRZECHWYTYWANIE ================= */

static int log_hook(const char *fmt, va_list args)
{
    /* W formacie logow v1 linia zaczyna sie od litery poziomu ("E (123) tag:"),
     * a ta jest juz w samym formacie - poziom I/D/V odpada bez formatowania,
     * wiec zwykle logi nie kosztuja nic ponad to, co dotad. */
    if ((fmt[0] == 'E' || fmt[0] == 'W') && s_queue &&
        xTaskGetCurrentTaskHandle() != s_task && !xPortInIsrContext()) {
        log_line_t l;
        va_list copy;
        va_copy(copy, args);
        vsnprintf(l.line, sizeof l.line, fmt, copy);
        va_end(copy);
        if (xQueueSend(s_queue, &l, 0) != pdTRUE) {
            s_dropped++;
        }
    }
    return s_orig_vprintf(fmt, args);
}

/* ================= PUBLIKACJA ================= */

/** Kopiuje tekst do JSON-a, z ucieczka cudzyslowow i znakow sterujacych. */
static size_t json_escape(char *out, size_t cap, const char *in, size_t len)
{
    size_t n = 0;
    for (size_t i = 0; i < len && n + 7 < cap; i++) {
        unsigned char c = (unsigned char)in[i];
        if (c == '"' || c == '\\') {
            out[n++] = '\\';
            out[n++] = (char)c;
        } else if (c < 0x20) {
            n += (size_t)snprintf(out + n, cap - n, "\\u%04x", c);
        } else {
            out[n++] = (char)c;
        }
    }
    out[n] = 0;
    return n;
}

/**
 * Rozbiera "E (12345) tag: tresc\n" na pola JSON-a. Linia w innym formacie
 * (np. przycieta) idzie w calosci jako msg - lepiej brzydko niz wcale.
 */
static void publish_line(const char *line)
{
    char lvl = line[0];
    unsigned long ms = 0;
    const char *tag = "";
    size_t tag_len = 0;
    const char *msg = line;

    const char *close = strstr(line, ") ");
    if (sscanf(line + 1, " (%lu)", &ms) == 1 && close) {
        tag = close + 2;
        const char *colon = strstr(tag, ": ");
        if (colon) {
            tag_len = (size_t)(colon - tag);
            msg = colon + 2;
        } else {
            tag = "";
        }
    }
    size_t msg_len = strlen(msg);
    while (msg_len > 0 && (msg[msg_len - 1] == '\n' || msg[msg_len - 1] == '\r')) {
        msg_len--;
    }

    char etag[48], emsg[LOG_LINE_MAX * 2];
    json_escape(etag, sizeof etag, tag, tag_len);
    json_escape(emsg, sizeof emsg, msg, msg_len);

    char buf[LOG_LINE_MAX * 2 + 96];
    snprintf(buf, sizeof buf, "{\"lvl\":\"%c\",\"t_ms\":%lu,\"tag\":\"%s\",\"msg\":\"%s\"}",
             (lvl == 'E' || lvl == 'W') ? lvl : '?', ms, etag, emsg);
    flora_mqtt_publish_str("tele/log", buf, 0, false);
}

static void log_task(void *arg)
{
    log_line_t l;
    for (;;) {
        /* Bez brokera linie zostaja w kolejce i czekaja. */
        if (!flora_mqtt_is_connected()) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        if (xQueueReceive(s_queue, &l, pdMS_TO_TICKS(500)) != pdTRUE) {
            continue;
        }
        publish_line(l.line);

        uint32_t dropped = s_dropped;
        if (dropped) {
            s_dropped = 0;
            char note[64];
            snprintf(note, sizeof note, "W (0) diag: pominieto %" PRIu32 " linii logu", dropped);
            publish_line(note);
        }
        vTaskDelay(pdMS_TO_TICKS(SEND_GAP_MS));
    }
}

/* ================= START ================= */

static const char *reset_name(esp_reset_reason_t r)
{
    switch (r) {
        case ESP_RST_POWERON:   return "power_on";
        case ESP_RST_EXT:       return "external";
        case ESP_RST_SW:        return "software";    /* esp_restart(), w tym cmd/reboot */
        case ESP_RST_PANIC:     return "panic";       /* awaria: wyjatek, assert, abort */
        case ESP_RST_INT_WDT:   return "int_wdt";
        case ESP_RST_TASK_WDT:  return "task_wdt";    /* zadanie zawieszone */
        case ESP_RST_WDT:       return "wdt";
        case ESP_RST_DEEPSLEEP: return "deep_sleep";
        case ESP_RST_BROWNOUT:  return "brownout";    /* spadek napiecia - zasilanie! */
        case ESP_RST_SDIO:      return "sdio";
        case ESP_RST_USB:       return "usb";
        case ESP_RST_JTAG:      return "jtag";
        default:                return "unknown";
    }
}

esp_err_t hub_diag_init(void)
{
    s_reset = esp_reset_reason();

    /* Licznik startow w NVS: szybko rosnacy przy "online" w panelu to
     * znak petli restartow, ktorej sam status by nie pokazal. */
    flora_store_t h;
    if (flora_store_open(FLORA_NVS_NS_SYS, &h) == ESP_OK) {
        flora_store_get_u32(h, FLORA_NVS_BOOT_COUNT, &s_boots, 0);
        s_boots++;
        flora_store_set_u32(h, FLORA_NVS_BOOT_COUNT, s_boots);
        flora_store_close(h);
    }

    const esp_app_desc_t *app = esp_app_get_description();
    ESP_LOGI(TAG, "firmware %s (%s %s), start nr %" PRIu32 ", powod restartu: %s",
             app->version, app->date, app->time, s_boots, reset_name(s_reset));
    if (s_reset == ESP_RST_PANIC || s_reset == ESP_RST_TASK_WDT ||
        s_reset == ESP_RST_INT_WDT || s_reset == ESP_RST_BROWNOUT) {
        /* Ostrzezenie, wiec trafi tez na tele/log. */
        ESP_LOGW(TAG, "poprzednie uruchomienie skonczylo sie awaryjnie: %s", reset_name(s_reset));
    }

    s_queue = xQueueCreate(QUEUE_LEN, sizeof(log_line_t));
    if (!s_queue) {
        return ESP_ERR_NO_MEM;
    }
    /* Stos na dwa bufory JSON-a w publish_line i vsnprintf w flora_mqtt. */
    if (xTaskCreate(log_task, "log_mqtt", 4096, NULL, 2, &s_task) != pdPASS) {
        vQueueDelete(s_queue);
        s_queue = NULL;
        return ESP_ERR_NO_MEM;
    }
    s_orig_vprintf = esp_log_set_vprintf(log_hook);
    return ESP_OK;
}

void hub_diag_publish_boot(void)
{
    const esp_app_desc_t *app = esp_app_get_description();
    flora_mqtt_publish_fmt("tele/boot", 1, true,
                           "{\"fw\":\"%s\",\"built\":\"%s %s\",\"idf\":\"%s\","
                           "\"reset\":\"%s\",\"boots\":%" PRIu32 "}",
                           app->version, app->date, app->time, app->idf_ver,
                           reset_name(s_reset), s_boots);
}
