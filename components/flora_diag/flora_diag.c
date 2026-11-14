/**
 * @file flora_diag.c
 * @brief Logi przez MQTT i informacja o starcie.
 *
 * LOGI: esp_log_set_vprintf() przechwytuje kazda linie logu. Konsola USB
 * dostaje ja bez zmian, a wybrane linie trafiaja dodatkowo do kolejki, z
 * ktorej osobne zadanie publikuje je na temat logow (bryla: tele/log,
 * glowica: head/log - podaje je flora_diag_cfg_t). Samo przechwycenie nigdy
 * nie czeka na siec: logi pisze tez zadanie lacza z glowica czy timer
 * pompki, a im nie wolno blokowac sie na brokerze.
 *
 * KTORE LINIE IDA NA MQTT:
 *   - zawsze W i E,
 *   - wiecej na zadanie: cmd/log "<tag|*> <E|W|I|D|V>" (flora_diag_set_level),
 *     np. "flora_sonar D" na czas szukania bledu. Nie zapisywane w NVS -
 *     po restarcie wraca W, zeby zapomniane D nie zalewalo bazy,
 *   - START: przez pierwsze BOOT_WINDOW_MS od wlaczenia wszystko od poziomu
 *     I w gore laduje w buforze w PSRAM (BOOT_LINES linii) i wychodzi, gdy
 *     jest polaczenie. Okno jest czasowe, nie "do polaczenia": sprzet
 *     (naped, czujniki) startuje dopiero PO polaczeniu z brokerem, wiec
 *     zamkniecie bufora przy polaczeniu gubilo akurat to, co najwazniejsze.
 *     Widac caly start, np. ktore czujniki wstaly. t_ms w
 *     kazdej linii to czas od startu, bo w Grafanie te linie maja czas
 *     wyslania, nie zapisu.
 *
 * Pelna kolejka gubi nowe linie i liczy je - pomijanie jest jawne.
 *
 * PETLA LOGOW: publikacja, ktora sie nie uda, sama loguje blad (flora_mqtt).
 * Linie z wlasnego zadania sa wiec pomijane - inaczej kazdy blad publikacji
 * rodzilby kolejny.
 */
#include <inttypes.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_app_desc.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_timer.h"

#include "flora_diag.h"
#include "flora_board.h"
#include "flora_mqtt.h"
#include "flora_store.h"

static const char *TAG = "diag";

/* Tematy wzgledne wobec bazy MQTT. Kopie, bo wywolujacy moze podac stos. */
static char s_log_topic[32]  = "tele/log";
static char s_boot_topic[32] = "tele/boot";

#define LOG_LINE_MAX   240   /* dluzsze linie sa przycinane */
#define QUEUE_LEN      64
#define SEND_GAP_MS    20    /* najwyzej ~50 linii/s, zeby petla bledow nie zalala brokera */
#define BOOT_LINES     300   /* bufor startu w PSRAM, ~72 kB */
#define BOOT_WINDOW_MS 30000 /* jak dlugo od wlaczenia zbierac linie I */
#define TAG_RULES      8     /* ile tagow moze miec wlasny poziom naraz */

typedef struct {
    char line[LOG_LINE_MAX];
} log_line_t;

static QueueHandle_t   s_queue;
static bool            s_queue_caps;   /* kolejka z xQueueCreateWithCaps */
static TaskHandle_t    s_task;
static vprintf_like_t  s_orig_vprintf;
static volatile uint32_t s_dropped;

/* ---- poziomy zdalne ---- */
typedef struct {
    char            tag[24];
    esp_log_level_t level;
} tag_rule_t;

static portMUX_TYPE     s_lock = portMUX_INITIALIZER_UNLOCKED;
static esp_log_level_t  s_global = ESP_LOG_WARN;
static tag_rule_t       s_rules[TAG_RULES];
static volatile esp_log_level_t s_rules_max = ESP_LOG_NONE;  /* najwyzszy poziom w regulach */

/* ---- bufor startu ---- */
typedef struct {
    volatile bool ready;
    char          line[LOG_LINE_MAX];
} boot_slot_t;

static boot_slot_t     *s_boot;
static volatile bool    s_boot_capture;
static volatile int     s_boot_n;           /* zarezerwowane sloty */
static int              s_boot_sent;        /* juz wyslane (tylko zadanie logow) */
static volatile uint32_t s_boot_dropped;

static esp_reset_reason_t s_reset;
static uint32_t           s_boots;

/* ================= PRZECHWYTYWANIE ================= */

static esp_log_level_t level_of(char c)
{
    switch (c) {
    case 'E': return ESP_LOG_ERROR;
    case 'W': return ESP_LOG_WARN;
    case 'I': return ESP_LOG_INFO;
    case 'D': return ESP_LOG_DEBUG;
    case 'V': return ESP_LOG_VERBOSE;
    default:  return ESP_LOG_NONE;
    }
}

/** Tag z linii "X (123) tag: tresc" do out; false gdy format inny. */
static bool tag_of(const char *line, char *out, size_t cap)
{
    const char *close = strstr(line, ") ");
    if (!close) {
        return false;
    }
    const char *tag = close + 2;
    const char *colon = strstr(tag, ": ");
    if (!colon || (size_t)(colon - tag) >= cap) {
        return false;
    }
    memcpy(out, tag, (size_t)(colon - tag));
    out[colon - tag] = 0;
    return true;
}

/** Czy linie z tym tagiem i poziomem wysylac (poza buforem startu). */
static bool wanted(esp_log_level_t lvl, const char *line)
{
    if (lvl <= ESP_LOG_WARN || lvl <= s_global) {
        return true;
    }
    char tag[24];
    if (!tag_of(line, tag, sizeof tag)) {
        return false;
    }
    bool yes = false;
    taskENTER_CRITICAL(&s_lock);
    for (int i = 0; i < TAG_RULES; i++) {
        if (s_rules[i].tag[0] && strcmp(s_rules[i].tag, tag) == 0) {
            yes = lvl <= s_rules[i].level;
            break;
        }
    }
    taskEXIT_CRITICAL(&s_lock);
    return yes;
}

static int log_hook(const char *fmt, va_list args)
{
    /* W formacie logow v1 linia zaczyna sie od litery poziomu ("E (123) tag:"),
     * a ta jest juz w samym formacie. Linie, ktorych na pewno nie wyslemy,
     * odpadaja bez formatowania - zwykle logi nie kosztuja wiecej niz dotad. */
    esp_log_level_t lvl = level_of(fmt[0]);
    bool capture = s_boot_capture && s_boot && lvl != ESP_LOG_NONE && lvl <= ESP_LOG_INFO;
    bool maybe = lvl != ESP_LOG_NONE &&
                 (lvl <= ESP_LOG_WARN || lvl <= s_global || lvl <= s_rules_max);

    if ((capture || maybe) && s_queue &&
        xTaskGetCurrentTaskHandle() != s_task && !xPortInIsrContext()) {
        bool stored = false;
        if (capture) {
            /* Formatujemy wprost do slotu w PSRAM, nie na stos - przy starcie
             * loguja tez zadania z malym stosem (timery, sterowniki). */
            int slot = -1;
            taskENTER_CRITICAL(&s_lock);
            if (s_boot_capture && s_boot_n < BOOT_LINES) {
                slot = s_boot_n++;
            }
            taskEXIT_CRITICAL(&s_lock);
            if (slot >= 0) {
                va_list copy;
                va_copy(copy, args);
                vsnprintf(s_boot[slot].line, LOG_LINE_MAX, fmt, copy);
                va_end(copy);
                s_boot[slot].ready = true;
                stored = true;
            } else if (s_boot_capture) {
                s_boot_dropped++;
            }
        }
        if (!stored && maybe) {
            log_line_t l;
            va_list copy;
            va_copy(copy, args);
            vsnprintf(l.line, sizeof l.line, fmt, copy);
            va_end(copy);
            if (wanted(lvl, l.line) && xQueueSend(s_queue, &l, 0) != pdTRUE) {
                s_dropped++;
            }
        }
    }
    return s_orig_vprintf(fmt, args);
}

/* ================= POZIOMY NA ZADANIE ================= */

esp_err_t flora_diag_set_level(const char *arg)
{
    char tag[24];
    char c;
    if (sscanf(arg, "%23s %c", tag, &c) != 2) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_log_level_t lvl = level_of(c);
    if (lvl == ESP_LOG_NONE) {
        return ESP_ERR_INVALID_ARG;
    }
    /* Ponizej W na MQTT i tak ida zawsze - nizszy prog nic by nie zmienil. */
    if (lvl < ESP_LOG_WARN) {
        lvl = ESP_LOG_WARN;
    }
    if (lvl > (esp_log_level_t)CONFIG_LOG_MAXIMUM_LEVEL) {
        /* Poziom wyciety przy kompilacji - linii i tak by nie bylo. */
        return ESP_ERR_NOT_SUPPORTED;
    }

    if (strcmp(tag, "*") == 0) {
        taskENTER_CRITICAL(&s_lock);
        s_global = lvl;
        memset(s_rules, 0, sizeof s_rules);
        s_rules_max = ESP_LOG_NONE;
        taskEXIT_CRITICAL(&s_lock);
        /* Konsola zostaje przynajmniej na I, jak przed zmiana. */
        esp_log_level_set("*", lvl > ESP_LOG_INFO ? lvl : ESP_LOG_INFO);
        ESP_LOGW(TAG, "logi na MQTT: wszystko od poziomu %c", c);
        return ESP_OK;
    }

    esp_err_t ret = ESP_OK;
    taskENTER_CRITICAL(&s_lock);
    int free_i = -1, hit = -1;
    for (int i = 0; i < TAG_RULES; i++) {
        if (s_rules[i].tag[0] == 0 && free_i < 0) free_i = i;
        if (strcmp(s_rules[i].tag, tag) == 0) hit = i;
    }
    int i = hit >= 0 ? hit : free_i;
    if (i < 0) {
        ret = ESP_ERR_NO_MEM;
    } else if (lvl <= s_global) {
        s_rules[i].tag[0] = 0;          /* nic ponad globalny - regula zbedna */
    } else {
        strcpy(s_rules[i].tag, tag);
        s_rules[i].level = lvl;
    }
    esp_log_level_t mx = ESP_LOG_NONE;
    for (int k = 0; k < TAG_RULES; k++) {
        if (s_rules[k].tag[0] && s_rules[k].level > mx) mx = s_rules[k].level;
    }
    s_rules_max = mx;
    taskEXIT_CRITICAL(&s_lock);
    if (ret != ESP_OK) {
        return ret;
    }
    esp_log_level_set(tag, lvl > ESP_LOG_INFO ? lvl : ESP_LOG_INFO);
    ESP_LOGW(TAG, "logi na MQTT: %s od poziomu %c", tag, c);
    return ESP_OK;
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
static void publish_line(const char *line, bool boot)
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

    char buf[LOG_LINE_MAX * 2 + 112];
    snprintf(buf, sizeof buf,
             "{\"lvl\":\"%c\",\"t_ms\":%lu,\"tag\":\"%s\",\"msg\":\"%s\",\"boot\":%s}",
             level_of(lvl) != ESP_LOG_NONE ? lvl : '?', ms, etag, emsg,
             boot ? "true" : "false");
    flora_mqtt_publish_str(s_log_topic, buf, 0, false);
}

/**
 * Wysyla, co przybylo w buforze startu. Po uplywie okna zamyka zbieranie,
 * a gdy wszystko wyszlo - zwalnia bufor.
 */
static void flush_boot(void)
{
    bool closing = esp_timer_get_time() / 1000 >= BOOT_WINDOW_MS;
    taskENTER_CRITICAL(&s_lock);
    if (closing) {
        s_boot_capture = false;
    }
    int n = s_boot_n;
    taskEXIT_CRITICAL(&s_lock);

    for (; s_boot_sent < n; s_boot_sent++) {
        /* Slot zarezerwowany przed chwila moze byc jeszcze pisany. */
        for (int t = 0; t < 10 && !s_boot[s_boot_sent].ready; t++) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        if (s_boot[s_boot_sent].ready) {
            publish_line(s_boot[s_boot_sent].line, true);
        }
        vTaskDelay(pdMS_TO_TICKS(SEND_GAP_MS));
        if (!flora_mqtt_is_connected()) {
            return;   /* reszta przy nastepnym polaczeniu */
        }
    }
    if (!closing) {
        return;
    }
    if (s_boot_dropped) {
        char note[96];
        snprintf(note, sizeof note, "W (0) diag: bufor startu pelny, pominieto %" PRIu32 " linii",
                 s_boot_dropped);
        publish_line(note, true);
    }
    heap_caps_free(s_boot);
    s_boot = NULL;
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
        if (s_boot) {
            flush_boot();
        }
        if (xQueueReceive(s_queue, &l, pdMS_TO_TICKS(s_boot ? 100 : 500)) != pdTRUE) {
            continue;
        }
        publish_line(l.line, false);

        uint32_t dropped = s_dropped;
        if (dropped) {
            s_dropped = 0;
            char note[64];
            snprintf(note, sizeof note, "W (0) diag: pominieto %" PRIu32 " linii logu", dropped);
            publish_line(note, false);
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

esp_err_t flora_diag_init(const flora_diag_cfg_t *cfg)
{
    if (cfg && cfg->log_topic) {
        strncpy(s_log_topic, cfg->log_topic, sizeof s_log_topic - 1);
    }
    if (cfg && cfg->boot_topic) {
        strncpy(s_boot_topic, cfg->boot_topic, sizeof s_boot_topic - 1);
    }
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

    /* Kolejka i bufor startu w PSRAM - w sumie ~90 kB, ktorych szkoda
     * w pamieci wewnetrznej. Bez PSRAM kolejka idzie do zwyklej, a bufora
     * startu po prostu nie ma. */
    s_queue = xQueueCreateWithCaps(QUEUE_LEN, sizeof(log_line_t), MALLOC_CAP_SPIRAM);
    s_queue_caps = s_queue != NULL;
    if (!s_queue) {
        s_queue = xQueueCreate(QUEUE_LEN, sizeof(log_line_t));
    }
    if (!s_queue) {
        return ESP_ERR_NO_MEM;
    }
    s_boot = heap_caps_calloc(BOOT_LINES, sizeof(boot_slot_t), MALLOC_CAP_SPIRAM);
    s_boot_capture = s_boot != NULL;
    /* Stos na bufory JSON-a w publish_line i vsnprintf w flora_mqtt. */
    if (xTaskCreate(log_task, "log_mqtt", 5120, NULL, 2, &s_task) != pdPASS) {
        if (s_queue_caps) {
            vQueueDeleteWithCaps(s_queue);
        } else {
            vQueueDelete(s_queue);
        }
        s_queue = NULL;
        heap_caps_free(s_boot);
        s_boot = NULL;
        s_boot_capture = false;
        return ESP_ERR_NO_MEM;
    }
    s_orig_vprintf = esp_log_set_vprintf(log_hook);

    /* Po podpieciu, zeby trafilo tez do bufora startu. */
    const esp_app_desc_t *app = esp_app_get_description();
    ESP_LOGI(TAG, "firmware %s (%s %s), start nr %" PRIu32 ", powod restartu: %s",
             app->version, app->date, app->time, s_boots, reset_name(s_reset));
    if (s_reset == ESP_RST_PANIC || s_reset == ESP_RST_TASK_WDT ||
        s_reset == ESP_RST_INT_WDT || s_reset == ESP_RST_BROWNOUT) {
        /* Ostrzezenie, wiec trafi tez na temat logow. */
        ESP_LOGW(TAG, "poprzednie uruchomienie skonczylo sie awaryjnie: %s", reset_name(s_reset));
    }
    return ESP_OK;
}

void flora_diag_publish_boot(void)
{
    const esp_app_desc_t *app = esp_app_get_description();
    const esp_partition_t *running = esp_ota_get_running_partition();

    /* Wersja, ktora bootloader odrzucil (nie potwierdzila sie po OTA albo
     * padla). Zostaje oznaczona do nastepnej udanej aktualizacji. */
    char rolled_back[33] = "";
    const esp_partition_t *bad = esp_ota_get_last_invalid_partition();
    esp_app_desc_t bad_desc;
    if (bad && esp_ota_get_partition_description(bad, &bad_desc) == ESP_OK) {
        strncpy(rolled_back, bad_desc.version, sizeof rolled_back - 1);
    }

    flora_mqtt_publish_fmt(s_boot_topic, 1, true,
                           "{\"fw\":\"%s\",\"built\":\"%s %s\",\"idf\":\"%s\","
                           "\"reset\":\"%s\",\"boots\":%" PRIu32 ","
                           "\"partition\":\"%s\",\"rolled_back\":\"%s\"}",
                           app->version, app->date, app->time, app->idf_ver,
                           reset_name(s_reset), s_boots,
                           running ? running->label : "?", rolled_back);
}
