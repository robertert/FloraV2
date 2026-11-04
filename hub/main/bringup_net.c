/**
 * @file bringup_net.c
 * @brief Test rozruchowy sieci: WiFi i MQTT.
 *
 * Dane dostepowe czytane z NVS, a stale z flora_board.h sluza jako domyslne -
 * ten sam wzorzec co kalibracja gleby. Po pierwszym zapisaniu do NVS zmiana
 * sieci nie wymaga juz rekompilacji.
 */
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "bringup_net.h"
#include "flora_board.h"
#include "flora_store.h"
#include "flora_wifi.h"
#include "flora_mqtt.h"

static const char *TAG = "siec";

static void on_wifi_state(const flora_wifi_info_t *info, void *ctx)
{
    switch (info->state) {
        case FLORA_WIFI_CONNECTED:
            ESP_LOGI(TAG, "  WiFi: adres " IPSTR, IP2STR(&info->ip));
            break;
        case FLORA_WIFI_CONNECTING:
            ESP_LOGI(TAG, "  WiFi: proba %" PRIu32, info->attempt);
            break;
        case FLORA_WIFI_DISCONNECTED:
            ESP_LOGW(TAG, "  WiFi: rozlaczone (%s), nastepna proba za %" PRIu32 " ms",
                     flora_wifi_reason_str(info->reason), info->next_retry_ms);
            break;
        case FLORA_WIFI_FAILED:
            ESP_LOGE(TAG, "  WiFi: poddaje sie");
            break;
    }
}

static void on_mqtt_state(bool connected, void *ctx)
{
    ESP_LOGI(TAG, "  MQTT: %s", connected ? "polaczony" : "rozlaczony");
    if (connected) {
        /* Nadpisujemy testament, zeby broker pokazal nas jako dostepnych. */
        flora_mqtt_publish_str("status", "online", 1, true);
    }
}

static void on_command(const char *topic, size_t topic_len,
                       const char *data, size_t data_len, void *ctx)
{
    ESP_LOGI(TAG, "  komenda z brokera: %.*s = %.*s",
             (int)topic_len, topic, (int)data_len, data);
}

/** Jedna proba: WiFi, potem MQTT. Zostawia dzialajacych klientow nawet
 *  przy ESP_ERR_TIMEOUT brokera, bo ESP-MQTT ponawia w tle.
 *
 *  @param[out] wifi_ok  false, gdy proba padla juz na WiFi i do brokera
 *                       w ogole nie doszlo. Wywolujacy potrzebuje tego
 *                       rozroznienia, bo sam kod bledu go nie niesie:
 *                       ESP_ERR_TIMEOUT wraca i z WiFi, i z brokera. */
static esp_err_t try_connect(const char *ssid, const char *pass, const char *uri,
                             const char *mqtt_user, const char *mqtt_pass,
                             bool *wifi_ok)
{
    *wifi_ok = false;

    /* --- WiFi --- */
    flora_wifi_cfg_t wcfg = FLORA_WIFI_DEFAULT_CFG();
    strncpy(wcfg.ssid, ssid, sizeof(wcfg.ssid) - 1);
    strncpy(wcfg.password, pass, sizeof(wcfg.password) - 1);
    wcfg.on_state = on_wifi_state;

    if (flora_wifi_init(&wcfg) != ESP_OK) {
        ESP_LOGE(TAG, "  nie udalo sie podniesc WiFi");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "  czekam na adres (do 20 s)...");
    esp_err_t r = flora_wifi_wait_connected(20000);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "  brak polaczenia z \"%s\": %s", ssid, esp_err_to_name(r));
        return r;
    }

    *wifi_ok = true;

    /* --- MQTT --- */
    flora_mqtt_cfg_t mcfg = FLORA_MQTT_DEFAULT_CFG();
    mcfg.uri        = uri;
    mcfg.base_topic = FLORA_MQTT_BASE_TOPIC;
    mcfg.client_id  = "flora-hub";
    /* Pusty login = broker bez uwierzytelniania. */
    mcfg.username   = mqtt_user[0] ? mqtt_user : NULL;
    mcfg.password   = mqtt_user[0] ? mqtt_pass : NULL;
    mcfg.on_state   = on_mqtt_state;

    if (flora_mqtt_init(&mcfg) != ESP_OK) {
        ESP_LOGE(TAG, "  nie udalo sie uruchomic klienta MQTT");
        return ESP_FAIL;
    }

    /* Nazwa .local rozwiazuje sie zapytaniem mDNS wbudowanym w lwIP
     * (CONFIG_LWIP_DNS_SUPPORT_MDNS_QUERIES), bez osobnego komponentu. */
    ESP_LOGI(TAG, "  czekam na brokera %s (do 10 s)...", uri);
    for (int i = 0; i < 100 && !flora_mqtt_is_connected(); i++) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    if (!flora_mqtt_is_connected()) {
        ESP_LOGE(TAG, "  broker nieosiagalny pod %s", uri);
        ESP_LOGE(TAG, "  sprawdz adres, port i czy broker nasluchuje poza localhostem");
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

/** Jeden zestaw ustawien polaczenia. */
typedef struct {
    const char *label;
    const char *ssid, *pass, *uri, *user, *mpass;
} net_attempt_t;

/** Czy obie proby siegaja po te sama siec WiFi. */
static bool same_wifi(const net_attempt_t *a, const net_attempt_t *b)
{
    return strcmp(a->ssid, b->ssid) == 0 && strcmp(a->pass, b->pass) == 0;
}

static bool same_attempt(const net_attempt_t *a, const net_attempt_t *b)
{
    return same_wifi(a, b) &&
           strcmp(a->uri, b->uri) == 0 && strcmp(a->user, b->user) == 0 &&
           strcmp(a->mpass, b->mpass) == 0;
}

esp_err_t bringup_net_connect(void)
{
    /* --- dane dostepowe --- */
    char ssid[33], pass[65], uri[96], muser[33], mpass[65] = "";
    flora_store_t store = NULL;

    strncpy(ssid,  FLORA_WIFI_SSID_DEFAULT, sizeof(ssid) - 1);  ssid[sizeof(ssid) - 1] = 0;
    strncpy(pass,  FLORA_WIFI_PASS_DEFAULT, sizeof(pass) - 1);  pass[sizeof(pass) - 1] = 0;
    strncpy(uri,   FLORA_MQTT_URI_DEFAULT,  sizeof(uri) - 1);   uri[sizeof(uri) - 1]  = 0;
    strncpy(muser, FLORA_MQTT_USER_DEFAULT, sizeof(muser) - 1); muser[sizeof(muser) - 1] = 0;

    if (flora_store_open(FLORA_NVS_NS_NET, &store) == ESP_OK) {
        flora_store_get_str(store, FLORA_NVS_WIFI_SSID, ssid, sizeof(ssid),
                            FLORA_WIFI_SSID_DEFAULT);
        flora_store_get_str(store, FLORA_NVS_WIFI_PASS, pass, sizeof(pass),
                            FLORA_WIFI_PASS_DEFAULT);
        flora_store_get_str(store, FLORA_NVS_MQTT_URI, uri, sizeof(uri),
                            FLORA_MQTT_URI_DEFAULT);
        flora_store_get_str(store, FLORA_NVS_MQTT_USER, muser, sizeof(muser),
                            FLORA_MQTT_USER_DEFAULT);
        flora_store_get_str(store, FLORA_NVS_MQTT_PASS, mpass, sizeof(mpass), "");
        flora_store_close(store);
    }

    if (strcmp(ssid, "ZMIEN_MNIE") == 0) {
        ESP_LOGW(TAG, "  SSID to nadal wartosc zastepcza - uzupelnij");
        ESP_LOGW(TAG, "  flora_board.h albo zapisz do NVS pod \"%s\"", FLORA_NVS_WIFI_SSID);
        return ESP_ERR_INVALID_STATE;
    }
    if (mpass[0] == 0) {
        ESP_LOGW(TAG, "  brak hasla brokera w NVS - serwer odmowi, zostanie broker lokalny");
        ESP_LOGW(TAG, "  ustaw je raz: cmd/config/mqtt_pass (z lokalnego panelu)");
    }

    /* Kolejnosc prob. Ustawienia z NVS mozna zmienic zdalnie, wiec literowka
     * w adresie albo hasle nie moze odciac robota na dobre - naprawa bylaby
     * tylko kablem. Kazda kolejna proba to bezpieczniejszy wariant:
     *   1. NVS                    to, co ustawiono zdalnie
     *   2. serwer z firmware      wkompilowane WiFi, adres i login; haslo z NVS
     *   3. broker na Macu         bez hasel - w domu bez internetu i przy
     *                             zgubionym hasle; stad da sie robota naprawic
     * NVS zostaje nietkniete: po chwilowej awarii serwera nastepny restart
     * znow zacznie od ustawien zapisanych. Identyczne proby sa pomijane,
     * tak samo jak te, ktore roznia sie wylacznie brokerem, gdy poprzednia
     * padla juz na WiFi - do brokera i tak by nie doszlo. */
    const net_attempt_t attempts[] = {
        { "z NVS",           ssid, pass, uri, muser, mpass },
        { "serwer domyslny", FLORA_WIFI_SSID_DEFAULT, FLORA_WIFI_PASS_DEFAULT,
                             FLORA_MQTT_URI_DEFAULT, FLORA_MQTT_USER_DEFAULT, mpass },
        { "broker lokalny",  FLORA_WIFI_SSID_DEFAULT, FLORA_WIFI_PASS_DEFAULT,
                             FLORA_MQTT_URI_LOCAL, "", "" },
    };
    const size_t n = sizeof(attempts) / sizeof(attempts[0]);

    esp_err_t r = ESP_FAIL;
    const net_attempt_t *prev = NULL;
    bool wifi_ok = false;
    for (size_t i = 0; i < n; i++) {
        const net_attempt_t *a = &attempts[i];
        if (prev && same_attempt(a, prev)) {
            continue;
        }
        if (prev && !wifi_ok && same_wifi(a, prev)) {
            /* Ta sama siec, ktora przed chwila nie wstala. Kolejne 20 s
             * czekania na ten sam timeout niczego nie sprawdzi. */
            ESP_LOGW(TAG, "  pomijam \"%s\": ta sama siec \"%s\", a ona wlasnie nie wstala",
                     a->label, a->ssid);
            continue;
        }
        if (prev) {
            ESP_LOGW(TAG, "  nie wyszlo - probuje: %s", a->label);
            flora_mqtt_deinit();   /* ESP_ERR_INVALID_STATE, gdy nie doszlo do MQTT - bez znaczenia */
            flora_wifi_deinit();
        }
        ESP_LOGI(TAG, "  [%s] siec \"%s\", broker %s%s%s", a->label, a->ssid, a->uri,
                 a->user[0] ? " jako " : "", a->user);
        r = try_connect(a->ssid, a->pass, a->uri, a->user, a->mpass, &wifi_ok);
        if (r == ESP_OK) {
            return ESP_OK;
        }
        prev = a;
    }

    /* Klienci z ostatniej proby zostaja uruchomieni i ponawiaja w tle:
     * WiFi bez konca (max_retries = -1), a gdy siec wstanie - ESP-MQTT. */
    if (wifi_ok) {
        ESP_LOGE(TAG, "  klient MQTT ponawia w tle - polaczenie moze jeszcze dojsc");
    } else {
        ESP_LOGE(TAG, "  WiFi ponawia w tle - do brokera nie doszlo ani razu");
    }
    return r;
}

void bringup_net_run(void)
{
    ESP_LOGI(TAG, "=== SIEC ===");

    if (bringup_net_connect() != ESP_OK) {
        return;
    }

    /* Subskrypcja zaklada sie nawet przed polaczeniem - komponent odtworzy
     * ja po nawiazaniu i po kazdym ponownym polaczeniu. */
    flora_mqtt_subscribe("cmd/#", 1, on_command, NULL);

    /* --- probna telemetria --- */
    ESP_LOGI(TAG, "  publikuje probna telemetrie:");
    flora_mqtt_publish_fmt("telemetry/uptime", 0, false, "%" PRId64,
                           esp_timer_get_time() / 1000000);
    flora_mqtt_publish_fmt("telemetry/heap", 0, false, "%" PRIu32,
                           (uint32_t)esp_get_free_heap_size());
    flora_mqtt_publish_str("telemetry/hello", "jednostka dolna dziala", 0, false);
    ESP_LOGI(TAG, "    %s/telemetry/*", FLORA_MQTT_BASE_TOPIC);
    ESP_LOGI(TAG, "  podsluchaj: mosquitto_sub -h <broker> -t '%s/#' -v",
             FLORA_MQTT_BASE_TOPIC);
}
