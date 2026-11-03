#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "flora_mqtt.h"

#include "esp_check.h"
#include "esp_crt_bundle.h"
#include "esp_log.h"
#include "mqtt_client.h"

static const char *TAG = "flora_mqtt";

#define TOPIC_MAX 128

typedef struct {
    char                 topic[TOPIC_MAX];   /**< pelny temat, z prefiksem */
    int                  qos;
    flora_mqtt_data_cb_t cb;
    void                *ctx;
    bool                 used;
} sub_t;

static struct {
    esp_mqtt_client_handle_t client;
    char base[TOPIC_MAX];
    flora_mqtt_state_cb_t on_state;
    void *state_ctx;
    sub_t subs[FLORA_MQTT_MAX_SUBS];
    bool connected;
    bool initialized;
} s_m;

/** Skleja temat bazowy ze sciezka wzgledna. */
static esp_err_t full_topic(char *out, size_t cap, const char *subtopic)
{
    int n;
    if (s_m.base[0] && subtopic && subtopic[0]) {
        n = snprintf(out, cap, "%s/%s", s_m.base, subtopic);
    } else if (subtopic && subtopic[0]) {
        n = snprintf(out, cap, "%s", subtopic);
    } else {
        n = snprintf(out, cap, "%s", s_m.base);
    }
    ESP_RETURN_ON_FALSE(n > 0 && (size_t)n < cap, ESP_ERR_INVALID_ARG, TAG,
                        "temat nie miesci sie w %u bajtach", (unsigned)cap);
    return ESP_OK;
}

/**
 * Dopasowuje odebrany temat do subskrypcji.
 *
 * Obsluguje zgodnosc doslowna oraz prefiks zakonczony "/#". Pelna gramatyka
 * wieloznacznikow MQTT nie jest tu potrzebna - broker i tak filtruje, a to
 * sluzy tylko wyborowi handlera po naszej stronie.
 */
static bool topic_matches(const char *pattern, const char *topic, size_t topic_len)
{
    size_t plen = strlen(pattern);

    if (plen >= 2 && pattern[plen - 1] == '#' && pattern[plen - 2] == '/') {
        size_t prefix = plen - 2;   /* bez "/#" */
        return topic_len >= prefix && strncmp(pattern, topic, prefix) == 0;
    }
    const char *plus = strchr(pattern, '+');
    if (plus) {
        size_t prefix = (size_t)(plus - pattern);
        return topic_len >= prefix && strncmp(pattern, topic, prefix) == 0;
    }
    return topic_len == plen && strncmp(pattern, topic, topic_len) == 0;
}

/** Po ponownym polaczeniu broker nie pamieta naszych subskrypcji. */
static void resubscribe_all(void)
{
    for (int i = 0; i < FLORA_MQTT_MAX_SUBS; i++) {
        if (!s_m.subs[i].used) {
            continue;
        }
        int id = esp_mqtt_client_subscribe(s_m.client, s_m.subs[i].topic, s_m.subs[i].qos);
        if (id < 0) {
            ESP_LOGW(TAG, "nie udalo sie odtworzyc subskrypcji \"%s\"", s_m.subs[i].topic);
        } else {
            ESP_LOGI(TAG, "odtworzona subskrypcja \"%s\"", s_m.subs[i].topic);
        }
    }
}

static void dispatch(const char *topic, size_t topic_len, const char *data, size_t data_len)
{
    for (int i = 0; i < FLORA_MQTT_MAX_SUBS; i++) {
        if (s_m.subs[i].used && topic_matches(s_m.subs[i].topic, topic, topic_len)) {
            s_m.subs[i].cb(topic, topic_len, data, data_len, s_m.subs[i].ctx);
            return;
        }
    }
}

static void mqtt_event(void *handler_args, esp_event_base_t base, int32_t id, void *data)
{
    esp_mqtt_event_handle_t ev = data;

    switch ((esp_mqtt_event_id_t)id) {
        case MQTT_EVENT_CONNECTED:
            s_m.connected = true;
            ESP_LOGI(TAG, "polaczono z brokerem");
            resubscribe_all();
            if (s_m.on_state) { s_m.on_state(true, s_m.state_ctx); }
            break;

        case MQTT_EVENT_DISCONNECTED:
            s_m.connected = false;
            ESP_LOGW(TAG, "rozlaczono z brokerem");
            if (s_m.on_state) { s_m.on_state(false, s_m.state_ctx); }
            break;

        case MQTT_EVENT_DATA:
            dispatch(ev->topic, (size_t)ev->topic_len, ev->data, (size_t)ev->data_len);
            break;

        case MQTT_EVENT_ERROR:
            /* Rozroznienie warstwy bledu oszczedza duzo czasu: problem
             * z gniazdem to zla sieci albo adres, a odrzucenie polaczenia
             * to zle dane dostepowe albo zajety identyfikator klienta. */
            if (ev->error_handle) {
                if (ev->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
                    ESP_LOGE(TAG, "blad transportu: errno %d - sprawdz adres brokera i siec",
                             ev->error_handle->esp_transport_sock_errno);
                } else if (ev->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED) {
                    ESP_LOGE(TAG, "broker odrzucil polaczenie (kod %d) - sprawdz dane "
                                  "dostepowe i czy identyfikator klienta nie jest zajety",
                             ev->error_handle->connect_return_code);
                } else {
                    ESP_LOGE(TAG, "blad MQTT typu %d", ev->error_handle->error_type);
                }
            }
            break;

        default:
            break;
    }
}

esp_err_t flora_mqtt_init(const flora_mqtt_cfg_t *cfg)
{
    ESP_RETURN_ON_FALSE(cfg && cfg->uri, ESP_ERR_INVALID_ARG, TAG, "brak adresu brokera");
    ESP_RETURN_ON_FALSE(!s_m.initialized, ESP_ERR_INVALID_STATE, TAG, "juz zainicjalizowano");

    memset(&s_m, 0, sizeof(s_m));
    if (cfg->base_topic) {
        strncpy(s_m.base, cfg->base_topic, sizeof(s_m.base) - 1);
    }
    s_m.on_state  = cfg->on_state;
    s_m.state_ctx = cfg->state_ctx;

    /* Temat testamentu musi byc pelny - broker nie zna naszego prefiksu. */
    static char lwt_full[TOPIC_MAX];
    esp_mqtt_client_config_t mc = {
        .broker.address.uri        = cfg->uri,
        .credentials.client_id     = cfg->client_id,
        .credentials.username      = cfg->username,
        .credentials.authentication.password = cfg->password,
        .session.keepalive         = cfg->keepalive_s ? cfg->keepalive_s : 30,
    };

    /* Polaczenie szyfrowane weryfikujemy pakietem certyfikatow glownych
     * z ESP-IDF (te same, co w przegladarkach). Certyfikat z Let's Encrypt
     * przechodzi bez wkompilowywania czegokolwiek, a odnowienie co 90 dni
     * nie wymaga zmian w robocie - zmienia sie tylko certyfikat serwera,
     * nie urzad, ktory go podpisal. */
    bool tls = strncmp(cfg->uri, "mqtts://", 8) == 0 || strncmp(cfg->uri, "wss://", 6) == 0;
    if (tls) {
        mc.broker.verification.crt_bundle_attach = esp_crt_bundle_attach;
    }

    if (cfg->lwt_topic && cfg->lwt_message) {
        ESP_RETURN_ON_ERROR(full_topic(lwt_full, sizeof(lwt_full), cfg->lwt_topic),
                            TAG, "temat testamentu");
        mc.session.last_will.topic  = lwt_full;
        mc.session.last_will.msg    = cfg->lwt_message;
        mc.session.last_will.msg_len = (int)strlen(cfg->lwt_message);
        mc.session.last_will.qos    = 1;
        mc.session.last_will.retain = 1;
    }

    s_m.client = esp_mqtt_client_init(&mc);
    ESP_RETURN_ON_FALSE(s_m.client, ESP_FAIL, TAG, "esp_mqtt_client_init");

    esp_err_t ret = ESP_OK;
    ESP_GOTO_ON_ERROR(esp_mqtt_client_register_event(s_m.client, ESP_EVENT_ANY_ID,
                                                     mqtt_event, NULL),
                      fail, TAG, "rejestracja zdarzen");
    ESP_GOTO_ON_ERROR(esp_mqtt_client_start(s_m.client), fail, TAG, "esp_mqtt_client_start");

    s_m.initialized = true;
    ESP_LOGI(TAG, "klient uruchomiony: %s%s%s, prefiks tematow \"%s\"",
             cfg->uri, tls ? " (TLS)" : "", cfg->username ? ", z logowaniem" : "",
             s_m.base[0] ? s_m.base : "(brak)");
    return ESP_OK;

fail:
    esp_mqtt_client_destroy(s_m.client);
    memset(&s_m, 0, sizeof(s_m));
    return ret;
}

esp_err_t flora_mqtt_deinit(void)
{
    ESP_RETURN_ON_FALSE(s_m.initialized, ESP_ERR_INVALID_STATE, TAG, "nie zainicjalizowano");
    esp_mqtt_client_stop(s_m.client);
    esp_mqtt_client_destroy(s_m.client);
    memset(&s_m, 0, sizeof(s_m));
    return ESP_OK;
}

esp_err_t flora_mqtt_publish(const char *subtopic, const void *data, size_t len,
                             int qos, bool retain)
{
    ESP_RETURN_ON_FALSE(s_m.initialized, ESP_ERR_INVALID_STATE, TAG, "nie zainicjalizowano");
    ESP_RETURN_ON_FALSE(s_m.connected, ESP_ERR_INVALID_STATE, TAG, "brak polaczenia z brokerem");

    char topic[TOPIC_MAX];
    ESP_RETURN_ON_ERROR(full_topic(topic, sizeof(topic), subtopic), TAG, "temat");

    int id = esp_mqtt_client_publish(s_m.client, topic, (const char *)data,
                                     (int)len, qos, retain ? 1 : 0);
    ESP_RETURN_ON_FALSE(id >= 0, ESP_FAIL, TAG, "publikacja \"%s\" nie powiodla sie", topic);
    return ESP_OK;
}

esp_err_t flora_mqtt_publish_str(const char *subtopic, const char *text, int qos, bool retain)
{
    ESP_RETURN_ON_FALSE(text, ESP_ERR_INVALID_ARG, TAG, "text == NULL");
    return flora_mqtt_publish(subtopic, text, strlen(text), qos, retain);
}

esp_err_t flora_mqtt_publish_fmt(const char *subtopic, int qos, bool retain,
                                 const char *fmt, ...)
{
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    ESP_RETURN_ON_FALSE(n > 0, ESP_ERR_INVALID_ARG, TAG, "blad formatowania");
    if ((size_t)n >= sizeof(buf)) {
        ESP_LOGW(TAG, "tresc przycieta do %u bajtow", (unsigned)sizeof(buf) - 1);
        n = (int)sizeof(buf) - 1;
    }
    return flora_mqtt_publish(subtopic, buf, (size_t)n, qos, retain);
}

esp_err_t flora_mqtt_subscribe(const char *subtopic, int qos,
                               flora_mqtt_data_cb_t cb, void *ctx)
{
    ESP_RETURN_ON_FALSE(s_m.initialized, ESP_ERR_INVALID_STATE, TAG, "nie zainicjalizowano");
    ESP_RETURN_ON_FALSE(cb, ESP_ERR_INVALID_ARG, TAG, "cb == NULL");

    char topic[TOPIC_MAX];
    ESP_RETURN_ON_ERROR(full_topic(topic, sizeof(topic), subtopic), TAG, "temat");

    int slot = -1;
    for (int i = 0; i < FLORA_MQTT_MAX_SUBS; i++) {
        if (s_m.subs[i].used && strcmp(s_m.subs[i].topic, topic) == 0) { slot = i; break; }
        if (!s_m.subs[i].used && slot < 0) { slot = i; }
    }
    ESP_RETURN_ON_FALSE(slot >= 0, ESP_ERR_NO_MEM, TAG,
                        "brak miejsca na subskrypcje (limit %d)", FLORA_MQTT_MAX_SUBS);

    strncpy(s_m.subs[slot].topic, topic, sizeof(s_m.subs[slot].topic) - 1);
    s_m.subs[slot].qos  = qos;
    s_m.subs[slot].cb   = cb;
    s_m.subs[slot].ctx  = ctx;
    s_m.subs[slot].used = true;

    /* Przy braku polaczenia zapamietujemy - odtworzymy po nawiazaniu. */
    if (s_m.connected) {
        int id = esp_mqtt_client_subscribe(s_m.client, topic, qos);
        ESP_RETURN_ON_FALSE(id >= 0, ESP_FAIL, TAG, "subskrypcja \"%s\"", topic);
    }
    ESP_LOGI(TAG, "subskrypcja \"%s\" (qos %d)%s", topic, qos,
             s_m.connected ? "" : " - zostanie zalozona po polaczeniu");
    return ESP_OK;
}

bool flora_mqtt_is_connected(void)
{
    return s_m.initialized && s_m.connected;
}
