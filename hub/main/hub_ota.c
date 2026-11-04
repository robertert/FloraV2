/**
 * @file hub_ota.c
 * @brief Aktualizacja firmware przez siec (OTA) z serwera robota.
 *
 * SKAD: wylacznie z serwera, z ktorym robot rozmawia - komenda podaje tylko
 * wersje, nigdy adres. Host bierzemy z adresu brokera (mqtts://<host>:8883),
 * pliki leza pod https://<host>/ota/hub/. Logowanie tym samym loginem
 * i haslem co do brokera: binarka zawiera domyslne haslo WiFi, wiec nie
 * moze byc publiczna.
 *
 * KOLEJNOSC:
 *   1. manifest <wersja>.json (albo latest.json): wersja, SHA-256, rozmiar
 *   2. esp_https_ota zapisuje obraz do wolnego slotu, ESP sprawdza naglowek
 *   3. liczymy SHA-256 zapisanych bajtow i porownujemy z manifestem -
 *      niezgodnosc przerywa, start zostaje na starej wersji
 *   4. ustawienie nowego slotu do startu i restart
 *
 * Suma liczona NA SERWERZE przy wgraniu (flora-deploy), nie w CI - TLS
 * gwarantuje, ze manifest przyszedl z naszego serwera.
 *
 * PROBA I POWROT: z CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE nowa wersja startuje
 * w stanie "do potwierdzenia". hub_ota_check_pending() potwierdza ja dopiero
 * po minucie stabilnego polaczenia - bo jedyne, czego nie da sie naprawic
 * zdalnie, to wersja, ktora nie umie sie polaczyc.
 */
#include <ctype.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_app_desc.h"
#include "esp_crt_bundle.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "psa/crypto.h"

#include "hub_ota.h"
#include "flora_board.h"
#include "flora_motors.h"
#include "flora_mqtt.h"
#include "flora_pump.h"
#include "flora_store.h"

static const char *TAG = "ota";

#define CONFIRM_STABLE_S   60    /* tyle sekund ciaglego polaczenia potwierdza wersje */
#define CONFIRM_DEADLINE_S 300   /* bez potwierdzenia do tej chwili - powrot */
#define VERSION_MAX        32    /* esp_app_desc_t.version */

typedef struct {
    char     version[VERSION_MAX];
    uint8_t  sha256[32];
    uint32_t size;
} manifest_t;

typedef struct {
    char want[VERSION_MAX];   /* "latest" albo "hub-vX.Y.Z" */
    bool force;
} job_t;

static volatile bool s_busy;
static bool          s_pending;   /* biezaca wersja czeka na potwierdzenie */

bool hub_ota_in_progress(void) { return s_busy; }

/* ================= POMOCNICZE ================= */

/** "hub-v1.2.3" - nic innego nie trafi do adresu pliku. */
static bool valid_version(const char *v)
{
    if (strncmp(v, "hub-v", 5) != 0) {
        return false;
    }
    const char *p = v + 5;
    for (int part = 0; part < 3; part++) {
        if (!isdigit((unsigned char)*p)) {
            return false;
        }
        while (isdigit((unsigned char)*p)) {
            p++;
        }
        if (part < 2 && *p++ != '.') {
            return false;
        }
    }
    return *p == 0 && strlen(v) < VERSION_MAX;
}

/** tele/ota: stan aktualizacji. Stany koncowe z retain, postep bez. */
static void report(const char *state, const char *to, int pct, const char *err, bool final)
{
    char buf[200];
    snprintf(buf, sizeof buf,
             "{\"state\":\"%s\",\"to\":\"%s\",\"pct\":%d,\"err\":\"%s\",\"current\":\"%s\"}",
             state, to ? to : "", pct, err ? err : "", esp_app_get_description()->version);
    flora_mqtt_publish_str("tele/ota", buf, final ? 1 : 0, final);
}

/**
 * Host serwera i dane logowania. Host z adresu brokera w NVS, jesli to
 * serwer (mqtts://), inaczej z adresu wkompilowanego - robot podlaczony
 * awaryjnie do brokera na Macu dalej moze sie zaktualizowac z serwera.
 */
static esp_err_t server_access(char *host, size_t host_cap, char *user, size_t user_cap,
                               char *pass, size_t pass_cap)
{
    char uri[96];
    strncpy(uri, FLORA_MQTT_URI_DEFAULT, sizeof uri - 1); uri[sizeof uri - 1] = 0;
    strncpy(user, FLORA_MQTT_USER_DEFAULT, user_cap - 1); user[user_cap - 1] = 0;
    pass[0] = 0;

    flora_store_t h;
    if (flora_store_open(FLORA_NVS_NS_NET, &h) == ESP_OK) {
        char nvs_uri[96];
        flora_store_get_str(h, FLORA_NVS_MQTT_URI, nvs_uri, sizeof nvs_uri, "");
        if (strncmp(nvs_uri, "mqtts://", 8) == 0) {
            strcpy(uri, nvs_uri);
        }
        flora_store_get_str(h, FLORA_NVS_MQTT_USER, user, user_cap, FLORA_MQTT_USER_DEFAULT);
        flora_store_get_str(h, FLORA_NVS_MQTT_PASS, pass, pass_cap, "");
        flora_store_close(h);
    }
    if (pass[0] == 0) {
        ESP_LOGE(TAG, "brak hasla serwera w NVS (cmd/config/mqtt_pass)");
        return ESP_ERR_INVALID_STATE;
    }

    /* "mqtts://host:8883" -> "host" */
    const char *start = strstr(uri, "://");
    start = start ? start + 3 : uri;
    size_t n = strcspn(start, ":/");
    if (n == 0 || n >= host_cap) {
        return ESP_ERR_INVALID_ARG;
    }
    memcpy(host, start, n);
    host[n] = 0;
    return ESP_OK;
}

static void http_common(esp_http_client_config_t *c, const char *url,
                        const char *user, const char *pass)
{
    memset(c, 0, sizeof *c);
    c->url               = url;
    c->crt_bundle_attach = esp_crt_bundle_attach;
    c->username          = user;
    c->password          = pass;
    c->auth_type         = HTTP_AUTH_TYPE_BASIC;
    c->timeout_ms        = 15000;
    c->keep_alive_enable = true;
}

/** Wyciaga "klucz":"wartosc" albo "klucz":liczba z plaskiego JSON-a. */
static bool json_field(const char *json, const char *key, char *out, size_t cap)
{
    char pat[24];
    snprintf(pat, sizeof pat, "\"%s\"", key);
    const char *p = strstr(json, pat);
    if (!p) {
        return false;
    }
    p += strlen(pat);
    while (*p == ' ' || *p == ':') {
        p++;
    }
    bool quoted = (*p == '"');
    if (quoted) {
        p++;
    }
    size_t n = 0;
    while (*p && n + 1 < cap && (quoted ? *p != '"' : (isalnum((unsigned char)*p) != 0))) {
        out[n++] = *p++;
    }
    out[n] = 0;
    return n > 0;
}

static esp_err_t fetch_manifest(const char *host, const char *name, const char *user,
                                const char *pass, manifest_t *m, char *why, size_t why_cap)
{
    char url[160];
    snprintf(url, sizeof url, "https://%s/ota/hub/%s.json", host, name);

    esp_http_client_config_t cfg;
    http_common(&cfg, url, user, pass);
    esp_http_client_handle_t c = esp_http_client_init(&cfg);
    if (!c) {
        return ESP_ERR_NO_MEM;
    }

    char body[384] = "";
    esp_err_t err = esp_http_client_open(c, 0);
    if (err == ESP_OK) {
        esp_http_client_fetch_headers(c);
        int status = esp_http_client_get_status_code(c);
        int len = esp_http_client_read_response(c, body, sizeof body - 1);
        if (status != 200) {
            /* 401 = zle haslo robota (NVS vs .env na serwerze),
             * 404 = nie ma takiej wersji na serwerze. */
            snprintf(why, why_cap, "manifest %s: HTTP %d%s", name, status,
                     status == 401 ? " (zle haslo?)" : status == 404 ? " (brak wersji)" : "");
            err = (status == 401) ? ESP_ERR_INVALID_STATE : ESP_ERR_NOT_FOUND;
        } else if (len <= 0) {
            snprintf(why, why_cap, "manifest %s: pusta odpowiedz", name);
            err = ESP_ERR_INVALID_RESPONSE;
        } else {
            body[len] = 0;
        }
    } else {
        snprintf(why, why_cap, "polaczenie z %s: %s", host, esp_err_to_name(err));
    }
    esp_http_client_cleanup(c);
    if (err != ESP_OK) {
        return err;
    }

    char sha_hex[72], size_s[16];
    if (!json_field(body, "version", m->version, sizeof m->version) ||
        !json_field(body, "sha256", sha_hex, sizeof sha_hex) ||
        !json_field(body, "size", size_s, sizeof size_s) ||
        !valid_version(m->version) || strlen(sha_hex) != 64) {
        snprintf(why, why_cap, "manifest %s nieczytelny", name);
        return ESP_ERR_INVALID_RESPONSE;
    }
    for (int i = 0; i < 32; i++) {
        char byte[3] = { sha_hex[2 * i], sha_hex[2 * i + 1], 0 };
        if (!isxdigit((unsigned char)byte[0]) || !isxdigit((unsigned char)byte[1])) {
            snprintf(why, why_cap, "manifest %s: zla suma", name);
            return ESP_ERR_INVALID_RESPONSE;
        }
        m->sha256[i] = (uint8_t)strtoul(byte, NULL, 16);
    }
    m->size = (uint32_t)strtoul(size_s, NULL, 10);
    return ESP_OK;
}

/** SHA-256 pierwszych `size` bajtow partycji - to samo co sha256sum pliku .bin. */
static esp_err_t partition_sha256(const esp_partition_t *p, uint32_t size, uint8_t out[32])
{
    if (psa_crypto_init() != PSA_SUCCESS) {
        return ESP_FAIL;
    }
    psa_hash_operation_t op = PSA_HASH_OPERATION_INIT;
    if (psa_hash_setup(&op, PSA_ALG_SHA_256) != PSA_SUCCESS) {
        return ESP_FAIL;
    }
    static uint8_t chunk[4096];   /* statyczny: stos zadania nie musi go miescic */
    esp_err_t err = ESP_OK;
    for (uint32_t off = 0; off < size && err == ESP_OK; off += sizeof chunk) {
        uint32_t n = (size - off) < sizeof chunk ? (size - off) : sizeof chunk;
        err = esp_partition_read(p, off, chunk, n);
        if (err == ESP_OK && psa_hash_update(&op, chunk, n) != PSA_SUCCESS) {
            err = ESP_FAIL;
        }
    }
    size_t len = 0;
    if (err != ESP_OK) {
        psa_hash_abort(&op);
        return err;
    }
    return psa_hash_finish(&op, out, 32, &len) == PSA_SUCCESS && len == 32 ? ESP_OK : ESP_FAIL;
}

/* ================= AKTUALIZACJA ================= */

static esp_err_t run_update(const job_t *job, char *err_msg, size_t err_cap, char *to, size_t to_cap)
{
    char host[64], user[33], pass[65];
    esp_err_t err = server_access(host, sizeof host, user, sizeof user, pass, sizeof pass);
    if (err != ESP_OK) {
        snprintf(err_msg, err_cap, "brak hasla serwera w NVS");
        return err;
    }

    manifest_t m;
    err = fetch_manifest(host, job->want, user, pass, &m, err_msg, err_cap);
    if (err != ESP_OK) {
        return err;
    }
    strncpy(to, m.version, to_cap - 1);
    to[to_cap - 1] = 0;

    const char *running = esp_app_get_description()->version;
    if (!job->force && strcmp(m.version, running) == 0) {
        snprintf(err_msg, err_cap, "%s jest juz zainstalowana", m.version);
        return ESP_ERR_INVALID_STATE;
    }
    const esp_partition_t *target = esp_ota_get_next_update_partition(NULL);
    if (!target || m.size == 0 || m.size > target->size) {
        snprintf(err_msg, err_cap, "obraz %" PRIu32 " B nie miesci sie w slocie", m.size);
        return ESP_ERR_INVALID_SIZE;
    }
    ESP_LOGW(TAG, "aktualizacja %s -> %s (%" PRIu32 " B) do %s",
             running, m.version, m.size, target->label);

    char url[160];
    snprintf(url, sizeof url, "https://%s/ota/hub/%s.bin", host, m.version);
    esp_http_client_config_t http;
    http_common(&http, url, user, pass);
    esp_https_ota_config_t ota = { .http_config = &http };

    esp_https_ota_handle_t h = NULL;
    err = esp_https_ota_begin(&ota, &h);
    if (err != ESP_OK) {
        snprintf(err_msg, err_cap, "start pobierania: %s", esp_err_to_name(err));
        return err;
    }

    int last_pct = -10;
    while ((err = esp_https_ota_perform(h)) == ESP_ERR_HTTPS_OTA_IN_PROGRESS) {
        int pct = (int)(100LL * esp_https_ota_get_image_len_read(h) / m.size);
        if (pct >= last_pct + 10) {
            last_pct = pct;
            report("downloading", m.version, pct, NULL, false);
        }
    }
    if (err != ESP_OK || !esp_https_ota_is_complete_data_received(h)) {
        snprintf(err_msg, err_cap, "pobieranie: %s", esp_err_to_name(err));
        esp_https_ota_abort(h);
        return err != ESP_OK ? err : ESP_ERR_INVALID_SIZE;
    }

    /* Suma przed ustawieniem slotu do startu - przy niezgodnosci abort
     * zostawia start na starej wersji. */
    report("verifying", m.version, 100, NULL, false);
    uint8_t sha[32];
    if (esp_https_ota_get_image_len_read(h) != (int)m.size ||
        partition_sha256(target, m.size, sha) != ESP_OK || memcmp(sha, m.sha256, 32) != 0) {
        snprintf(err_msg, err_cap, "suma SHA-256 nie zgadza sie z manifestem");
        esp_https_ota_abort(h);
        return ESP_ERR_INVALID_CRC;
    }

    /* finish sprawdza obraz (naglowek, wlasna suma ESP) i ustawia slot do startu. */
    err = esp_https_ota_finish(h);
    if (err != ESP_OK) {
        snprintf(err_msg, err_cap, "obraz odrzucony: %s", esp_err_to_name(err));
        return err;
    }
    return ESP_OK;
}

static void ota_task(void *arg)
{
    job_t job = *(job_t *)arg;
    free(arg);

    char err_msg[96] = "", to[VERSION_MAX] = "";
    esp_err_t err = run_update(&job, err_msg, sizeof err_msg, to, sizeof to);
    if (err == ESP_OK) {
        ESP_LOGW(TAG, "zainstalowano %s - restart", to);
        report("rebooting", to, 100, NULL, true);
        vTaskDelay(pdMS_TO_TICKS(1000));   /* niech wiadomosc wyjdzie */
        esp_restart();
    }
    ESP_LOGE(TAG, "aktualizacja nieudana: %s", err_msg);
    report("failed", to, 0, err_msg, true);
    s_busy = false;
    vTaskDelete(NULL);
}

esp_err_t hub_ota_start(const char *arg)
{
    job_t *job = calloc(1, sizeof *job);
    if (!job) {
        return ESP_ERR_NO_MEM;
    }
    /* "<wersja>" albo "<wersja> force" */
    const char *sp = strchr(arg, ' ');
    size_t n = sp ? (size_t)(sp - arg) : strlen(arg);
    job->force = sp && strcmp(sp + 1, "force") == 0;
    if (n == 0 || n >= sizeof job->want || (sp && !job->force)) {
        free(job);
        return ESP_ERR_INVALID_ARG;
    }
    memcpy(job->want, arg, n);
    job->want[n] = 0;
    if (strcmp(job->want, "latest") != 0 && !valid_version(job->want)) {
        free(job);
        return ESP_ERR_INVALID_ARG;
    }

    /* Aktualizacja trwa okolo minuty i konczy sie restartem - nie w trakcie
     * jazdy ani podlewania. Druga z rzedu przed potwierdzeniem pierwszej
     * odebralaby mozliwosc powrotu (nadpisalaby jedyna dzialajaca wersje). */
    float l = 0, r = 0;
    flora_motors_get_last(&l, &r);
    if (s_busy || s_pending || l != 0.0f || r != 0.0f || flora_pump_is_running()) {
        free(job);
        return ESP_ERR_INVALID_STATE;
    }
    s_busy = true;
    flora_motors_stop();
    flora_pump_stop();

    /* TLS i pobieranie potrzebuja wiecej stosu niz zadanie MQTT. */
    if (xTaskCreate(ota_task, "ota", 8192, job, 3, NULL) != pdPASS) {
        free(job);
        s_busy = false;
        return ESP_ERR_NO_MEM;
    }
    report("starting", job->want, 0, NULL, false);
    return ESP_OK;
}

/* ================= POTWIERDZENIE NOWEJ WERSJI ================= */

static void confirm_tick(void *arg)
{
    static int stable_s, elapsed_s;
    elapsed_s++;
    stable_s = flora_mqtt_is_connected() ? stable_s + 1 : 0;

    if (stable_s >= CONFIRM_STABLE_S) {
        esp_ota_mark_app_valid_cancel_rollback();
        s_pending = false;
        ESP_LOGW(TAG, "wersja %s potwierdzona", esp_app_get_description()->version);
        report("confirmed", esp_app_get_description()->version, 100, NULL, true);
        esp_timer_stop(*(esp_timer_handle_t *)arg);
        return;
    }
    if (elapsed_s >= CONFIRM_DEADLINE_S) {
        /* Log zostaje w kolejce i nie wyjdzie - zostaje po nim tele/boot
         * z rolled_back po powrocie do starej wersji. */
        ESP_LOGE(TAG, "brak stabilnego polaczenia przez %d s - powrot do poprzedniej wersji",
                 CONFIRM_DEADLINE_S);
        esp_ota_mark_app_invalid_rollback_and_reboot();
    }
}

void hub_ota_check_pending(void)
{
    esp_ota_img_states_t state;
    const esp_partition_t *running = esp_ota_get_running_partition();
    if (esp_ota_get_state_partition(running, &state) != ESP_OK ||
        state != ESP_OTA_IMG_PENDING_VERIFY) {
        return;
    }
    s_pending = true;
    ESP_LOGW(TAG, "pierwszy start %s na %s - czekam na %d s stabilnego polaczenia",
             esp_app_get_description()->version, running->label, CONFIRM_STABLE_S);

    static esp_timer_handle_t timer;
    const esp_timer_create_args_t args = {
        .callback = confirm_tick,
        .arg      = &timer,
        .name     = "ota_confirm",
    };
    if (esp_timer_create(&args, &timer) != ESP_OK ||
        esp_timer_start_periodic(timer, 1000 * 1000) != ESP_OK) {
        /* Bez timera nie potwierdzimy - lepiej od razu wrocic niz utknac
         * na wersji, ktora przy nastepnym restarcie i tak sie cofnie. */
        esp_ota_mark_app_invalid_rollback_and_reboot();
    }
}
