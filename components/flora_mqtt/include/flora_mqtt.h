/**
 * @file flora_mqtt.h
 * @brief Klient MQTT do telemetrii.
 *
 * Opakowanie nad ESP-MQTT, ktory od ESP-IDF 6.0 nie jest juz czescia
 * frameworka tylko komponentem z rejestru (`espressif/mqtt`) - zaleznosc
 * jest zadeklarowana w idf_component.yml tego komponentu, wiec aplikacja
 * nie musi o niej wiedziec.
 *
 * Ponawianie polaczenia robi sam ESP-MQTT i nie duplikujemy go tutaj -
 * inaczej niz przy WiFi, gdzie esp_wifi_connect() probuje tylko raz.
 *
 * PREFIKS TEMATOW: konfiguracja przyjmuje temat bazowy, do ktorego doklejane
 * sa sciezki wzgledne. Dzieki temu zmiana identyfikatora robota nie wymaga
 * przepisywania kazdego wywolania publikacji.
 *
 * TESTAMENT (last will): broker rozglasza ustalona wiadomosc, gdy klient
 * zniknie bez pozegnania. Dla robota na trawniku to jedyny sposob, zeby
 * odroznic "wylaczony porzadnie" od "zniknal z sieci" - warto ustawic.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FLORA_MQTT_MAX_SUBS 8

/** Wiadomosc odebrana z brokera. Dane NIE sa zakonczone zerem. */
typedef void (*flora_mqtt_data_cb_t)(const char *topic, size_t topic_len,
                                     const char *data, size_t data_len, void *ctx);

typedef void (*flora_mqtt_state_cb_t)(bool connected, void *ctx);

typedef struct {
    const char *uri;         /**< np. "mqtt://192.168.1.10:1883" */
    const char *client_id;   /**< NULL generuje identyfikator z adresu MAC */
    const char *username;    /**< NULL gdy broker bez uwierzytelniania */
    const char *password;

    /** Doklejany przed sciezkami wzglednymi, bez konczacego ukosnika. */
    const char *base_topic;

    /* Testament rozglaszany przez brokera po nieoczekiwanym zaniku klienta.
     * Temat jest wzgledny wobec base_topic. */
    const char *lwt_topic;
    const char *lwt_message;

    int keepalive_s;         /**< 0 bierze domyslne 120 s */

    flora_mqtt_state_cb_t on_state;
    void                 *state_ctx;
} flora_mqtt_cfg_t;

#define FLORA_MQTT_DEFAULT_CFG()       \
    {                                  \
        .lwt_topic   = "status",       \
        .lwt_message = "offline",      \
        .keepalive_s = 30,             \
    }

/**
 * @brief Tworzy klienta i rozpoczyna laczenie. Nie blokuje.
 *
 * Wymaga dzialajacego polaczenia sieciowego - wywoluj po uzyskaniu adresu IP.
 */
esp_err_t flora_mqtt_init(const flora_mqtt_cfg_t *cfg);
esp_err_t flora_mqtt_deinit(void);

/**
 * @brief Publikuje pod tematem WZGLEDNYM wobec base_topic.
 *
 * @param qos     0 bez potwierdzenia, 1 co najmniej raz, 2 dokladnie raz
 * @param retain  broker zapamieta wiadomosc i poda nowym subskrybentom
 * @return ESP_ERR_INVALID_STATE gdy brak polaczenia z brokerem
 */
esp_err_t flora_mqtt_publish(const char *subtopic, const void *data, size_t len,
                             int qos, bool retain);

/** Wygodna wersja dla tekstu zakonczonego zerem. */
esp_err_t flora_mqtt_publish_str(const char *subtopic, const char *text,
                                 int qos, bool retain);

/** Publikuje z formatowaniem jak printf. Ograniczenie 256 bajtow. */
esp_err_t flora_mqtt_publish_fmt(const char *subtopic, int qos, bool retain,
                                 const char *fmt, ...) __attribute__((format(printf, 4, 5)));

/**
 * @brief Subskrybuje temat wzgledny i podpina handler.
 *
 * Dopasowanie obsluguje zgodnosc doslowna oraz prefiks zakonczony "/#".
 * Symbol "+" w srodku tematu jest przekazywany do brokera, ale handler
 * dopasuje sie tylko po prefiksie do pierwszego "+", wiec przy takich
 * wzorcach sprawdz temat ponownie wewnatrz handlera.
 *
 * Subskrypcje sa zapamietywane i odtwarzane po kazdym ponownym polaczeniu -
 * bez tego po zerwaniu sieci robot przestalby odbierac komendy.
 */
esp_err_t flora_mqtt_subscribe(const char *subtopic, int qos,
                               flora_mqtt_data_cb_t cb, void *ctx);

bool flora_mqtt_is_connected(void);

#ifdef __cplusplus
}
#endif
