#include <inttypes.h>
#include <stdlib.h>
#include <string.h>

#include "flora_wifi.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_random.h"

static const char *TAG = "flora_wifi";

#define BIT_CONNECTED  BIT0
#define BIT_FAILED     BIT1

static struct {
    flora_wifi_cfg_t  cfg;
    esp_netif_t      *netif;
    esp_event_handler_instance_t h_wifi;
    esp_event_handler_instance_t h_ip;
    esp_timer_handle_t retry_timer;
    EventGroupHandle_t events;

    flora_wifi_info_t info;
    bool running;        /**< false po flora_wifi_stop - nie ponawiamy */
    bool initialized;
} s_w;

const char *flora_wifi_reason_str(uint8_t reason)
{
    /* Tylko te powody, ktore faktycznie cos mowia o przyczynie. Reszta
     * trafia do galezi domyslnej razem z numerem. */
    switch (reason) {
        case 2:   return "uplynal czas uwierzytelnienia";
        case 15:  return "zle haslo (4-way handshake)";
        case 200: return "utrata ramek rozgloszeniowych";
        case 201: return "nie znaleziono sieci";
        case 202: return "uwierzytelnienie odrzucone";
        case 203: return "asocjacja odrzucona";
        case 204: return "uplynal czas uzgadniania";
        case 205: return "polaczenie nieudane";
        case 211: return "zadna siec nie spelnia progu zabezpieczen";
        case 212: return "zadna siec nie spelnia progu sygnalu";
        default:  return "inny";
    }
}

/** Wylicza odstep do nastepnej proby: wykladniczo, z sufitem i rozrzutem. */
static uint32_t backoff_ms(uint32_t attempt)
{
    uint32_t min_ms = s_w.cfg.backoff_min_ms ? s_w.cfg.backoff_min_ms : 1000;
    uint32_t max_ms = s_w.cfg.backoff_max_ms ? s_w.cfg.backoff_max_ms : 60000;

    uint32_t delay = min_ms;
    for (uint32_t i = 1; i < attempt && delay < max_ms; i++) {
        delay *= 2;
    }
    if (delay > max_ms) {
        delay = max_ms;
    }
    /* Do 25% rozrzutu. Po zaniku zasilania w domu wszystkie urzadzenia
     * startuja naraz; bez rozrzutu uderzylyby w router w tej samej chwili. */
    uint32_t jitter = delay / 4;
    return jitter ? delay - jitter / 2 + (esp_random() % jitter) : delay;
}

static void notify(void)
{
    if (s_w.cfg.on_state) {
        s_w.cfg.on_state(&s_w.info, s_w.cfg.state_ctx);
    }
}

static void retry_cb(void *arg)
{
    if (!s_w.running) {
        return;
    }
    s_w.info.state = FLORA_WIFI_CONNECTING;
    notify();
    esp_err_t ret = esp_wifi_connect();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "esp_wifi_connect: %s", esp_err_to_name(ret));
    }
}

static void schedule_retry(void)
{
    if (!s_w.running) {
        return;
    }
    if (s_w.cfg.max_retries >= 0 && s_w.info.attempt >= (uint32_t)s_w.cfg.max_retries) {
        s_w.info.state = FLORA_WIFI_FAILED;
        ESP_LOGE(TAG, "wyczerpano limit %d prob polaczenia", s_w.cfg.max_retries);
        xEventGroupSetBits(s_w.events, BIT_FAILED);
        notify();
        return;
    }

    s_w.info.attempt++;
    uint32_t delay = backoff_ms(s_w.info.attempt);
    s_w.info.next_retry_ms = delay;
    ESP_LOGI(TAG, "proba %" PRIu32 " za %" PRIu32 " ms", s_w.info.attempt, delay);

    esp_timer_stop(s_w.retry_timer);
    esp_timer_start_once(s_w.retry_timer, (uint64_t)delay * 1000);
}

static void wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (id == WIFI_EVENT_STA_START) {
        s_w.info.attempt = 0;
        retry_cb(NULL);                 /* pierwsza proba od razu, bez czekania */
        return;
    }
    if (id != WIFI_EVENT_STA_DISCONNECTED) {
        return;
    }

    const wifi_event_sta_disconnected_t *e = data;
    s_w.info.reason = e->reason;
    s_w.info.rssi   = e->rssi;
    s_w.info.state  = FLORA_WIFI_DISCONNECTED;
    xEventGroupClearBits(s_w.events, BIT_CONNECTED);

    ESP_LOGW(TAG, "rozlaczono: %s (powod %u, sygnal %d dBm)",
             flora_wifi_reason_str(e->reason), e->reason, e->rssi);
    notify();
    schedule_retry();
}

static void ip_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (id != IP_EVENT_STA_GOT_IP) {
        return;
    }
    const ip_event_got_ip_t *e = data;

    s_w.info.ip            = e->ip_info.ip;
    s_w.info.state         = FLORA_WIFI_CONNECTED;
    s_w.info.attempt       = 0;         /* udane polaczenie zeruje odstep */
    s_w.info.next_retry_ms = 0;

    ESP_LOGI(TAG, "polaczono, adres " IPSTR, IP2STR(&e->ip_info.ip));
    xEventGroupClearBits(s_w.events, BIT_FAILED);
    xEventGroupSetBits(s_w.events, BIT_CONNECTED);
    notify();
}

esp_err_t flora_wifi_init(const flora_wifi_cfg_t *cfg)
{
    ESP_RETURN_ON_FALSE(cfg, ESP_ERR_INVALID_ARG, TAG, "cfg == NULL");
    ESP_RETURN_ON_FALSE(!s_w.initialized, ESP_ERR_INVALID_STATE, TAG, "juz zainicjalizowano");
    ESP_RETURN_ON_FALSE(cfg->ssid[0], ESP_ERR_INVALID_ARG, TAG, "pusty SSID");

    memset(&s_w, 0, sizeof(s_w));
    s_w.cfg = *cfg;

    s_w.events = xEventGroupCreate();
    ESP_RETURN_ON_FALSE(s_w.events, ESP_ERR_NO_MEM, TAG, "brak pamieci");

    esp_err_t ret = ESP_OK;
    ESP_GOTO_ON_ERROR(esp_netif_init(), fail, TAG, "esp_netif_init");

    /* Petla zdarzen moze byc juz utworzona przez inny komponent. */
    ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_GOTO_ON_ERROR(ret, fail, TAG, "esp_event_loop_create_default");
    }

    /* Ta funkcja uzywa assert() w srodku, wiec wymaga juz istniejacej
     * domyslnej petli zdarzen - stad kolejnosc powyzej. */
    s_w.netif = esp_netif_create_default_wifi_sta();
    ESP_GOTO_ON_FALSE(s_w.netif, ESP_FAIL, fail, TAG, "esp_netif_create_default_wifi_sta");

    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_GOTO_ON_ERROR(esp_wifi_init(&init), fail, TAG, "esp_wifi_init");

    ESP_GOTO_ON_ERROR(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                          wifi_event, NULL, &s_w.h_wifi),
                      fail, TAG, "rejestracja WIFI_EVENT");
    ESP_GOTO_ON_ERROR(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                          ip_event, NULL, &s_w.h_ip),
                      fail, TAG, "rejestracja IP_EVENT");

    wifi_config_t wc = { 0 };
    strncpy((char *)wc.sta.ssid, cfg->ssid, sizeof(wc.sta.ssid) - 1);
    strncpy((char *)wc.sta.password, cfg->password, sizeof(wc.sta.password) - 1);
    /* Przeszukanie wszystkich kanalow i wybor po sile sygnalu. Przy robocie,
     * ktory jezdzi po trawniku, punkt dostepowy bywa na granicy zasiegu. */
    wc.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
    wc.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
    wc.sta.threshold.authmode = cfg->password[0] ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;

    ESP_GOTO_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), fail, TAG, "esp_wifi_set_mode");
    ESP_GOTO_ON_ERROR(esp_wifi_set_config(WIFI_IF_STA, &wc), fail, TAG, "esp_wifi_set_config");

    const esp_timer_create_args_t targs = { .callback = retry_cb, .name = "wifi_retry" };
    ESP_GOTO_ON_ERROR(esp_timer_create(&targs, &s_w.retry_timer), fail, TAG, "timer");

    s_w.running     = true;
    s_w.initialized = true;
    ESP_GOTO_ON_ERROR(esp_wifi_start(), fail_init, TAG, "esp_wifi_start");

    ESP_LOGI(TAG, "laczenie z \"%s\", odstep %" PRIu32 "-%" PRIu32 " ms",
             cfg->ssid,
             cfg->backoff_min_ms ? cfg->backoff_min_ms : 1000,
             cfg->backoff_max_ms ? cfg->backoff_max_ms : 60000);
    return ESP_OK;

fail_init:
    s_w.initialized = false;
fail:
    if (s_w.retry_timer) { esp_timer_delete(s_w.retry_timer); }
    if (s_w.h_ip)   { esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, s_w.h_ip); }
    if (s_w.h_wifi) { esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, s_w.h_wifi); }
    if (s_w.netif)  { esp_wifi_deinit(); esp_netif_destroy_default_wifi(s_w.netif); }
    if (s_w.events) { vEventGroupDelete(s_w.events); }
    memset(&s_w, 0, sizeof(s_w));
    return ret;
}

esp_err_t flora_wifi_deinit(void)
{
    ESP_RETURN_ON_FALSE(s_w.initialized, ESP_ERR_INVALID_STATE, TAG, "nie zainicjalizowano");

    s_w.running = false;
    esp_timer_stop(s_w.retry_timer);
    esp_timer_delete(s_w.retry_timer);
    esp_wifi_stop();
    esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, s_w.h_ip);
    esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, s_w.h_wifi);
    esp_wifi_deinit();
    esp_netif_destroy_default_wifi(s_w.netif);
    vEventGroupDelete(s_w.events);
    memset(&s_w, 0, sizeof(s_w));
    return ESP_OK;
}

esp_err_t flora_wifi_stop(void)
{
    ESP_RETURN_ON_FALSE(s_w.initialized, ESP_ERR_INVALID_STATE, TAG, "nie zainicjalizowano");
    s_w.running = false;
    esp_timer_stop(s_w.retry_timer);
    return esp_wifi_disconnect();
}

esp_err_t flora_wifi_start(void)
{
    ESP_RETURN_ON_FALSE(s_w.initialized, ESP_ERR_INVALID_STATE, TAG, "nie zainicjalizowano");
    s_w.running      = true;
    s_w.info.attempt = 0;
    xEventGroupClearBits(s_w.events, BIT_FAILED);
    retry_cb(NULL);
    return ESP_OK;
}

esp_err_t flora_wifi_wait_connected(uint32_t timeout_ms)
{
    ESP_RETURN_ON_FALSE(s_w.initialized, ESP_ERR_INVALID_STATE, TAG, "nie zainicjalizowano");

    EventBits_t bits = xEventGroupWaitBits(s_w.events, BIT_CONNECTED | BIT_FAILED,
                                           pdFALSE, pdFALSE, pdMS_TO_TICKS(timeout_ms));
    if (bits & BIT_CONNECTED) { return ESP_OK; }
    if (bits & BIT_FAILED)    { return ESP_FAIL; }
    return ESP_ERR_TIMEOUT;
}

bool flora_wifi_is_connected(void)
{
    return s_w.initialized && s_w.info.state == FLORA_WIFI_CONNECTED;
}

void flora_wifi_get_info(flora_wifi_info_t *out)
{
    if (out && s_w.initialized) {
        *out = s_w.info;
    }
}
