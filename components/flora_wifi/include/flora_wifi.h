/**
 * @file flora_wifi.h
 * @brief Polaczenie WiFi w trybie stacji, z ponawianiem.
 *
 * DLACZEGO PONAWIANIE JEST TUTAJ: esp_wifi_connect() probuje polaczyc sie
 * DOKLADNIE RAZ. Nie ma zadnego wbudowanego ponawiania - po nieudanej probie
 * albo po utracie polaczenia uklad po prostu zostaje rozlaczony, dopoki
 * aplikacja sama nie zareaguje na zdarzenie. To najczestsze zrodlo
 * "dziala na biurku, w terenie przestaje".
 *
 * Odstep rosnie wykladniczo i ma SUFIT. Bez sufitu dostajesz jedna z dwoch
 * patologii: albo zalewasz router probami co kilkadziesiat milisekund, albo
 * po godzinie bez sieci nastepna proba wypada za osiem godzin. Do tego
 * losowy rozrzut, zeby po zaniku zasilania w domu wszystkie urzadzenia nie
 * uderzyly w router w tej samej sekundzie.
 *
 * Komponent jest generyczny - dane dostepowe przychodza w konfiguracji.
 * Skad je wziac (NVS, Kconfig, stale w kodzie) decyduje aplikacja.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_netif_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    FLORA_WIFI_DISCONNECTED,
    FLORA_WIFI_CONNECTING,
    FLORA_WIFI_CONNECTED,
    FLORA_WIFI_FAILED,       /**< wyczerpano limit prob */
} flora_wifi_state_t;

typedef struct {
    flora_wifi_state_t state;
    esp_ip4_addr_t     ip;        /**< wypelnione tylko w stanie CONNECTED */
    uint8_t            reason;    /**< wifi_err_reason_t przy rozlaczeniu */
    int8_t             rssi;      /**< sila sygnalu w chwili rozlaczenia */
    uint32_t           attempt;   /**< numer biezacej proby */
    uint32_t           next_retry_ms;
} flora_wifi_info_t;

typedef void (*flora_wifi_cb_t)(const flora_wifi_info_t *info, void *ctx);

typedef struct {
    char ssid[33];
    char password[65];

    uint32_t backoff_min_ms;   /**< pierwszy odstep, 0 bierze domyslny */
    uint32_t backoff_max_ms;   /**< sufit odstepu */
    int      max_retries;      /**< ujemne znaczy bez konca */

    flora_wifi_cb_t on_state;
    void           *state_ctx;
} flora_wifi_cfg_t;

/** Sufit minuty jest kompromisem: po awarii routera robot wraca do sieci
 *  w ciagu minuty, a przy trwalym braku sieci nie zajmuje radia w kolko. */
#define FLORA_WIFI_DEFAULT_CFG()      \
    {                                 \
        .backoff_min_ms = 1000,       \
        .backoff_max_ms = 60000,      \
        .max_retries    = -1,         \
    }

/**
 * @brief Podnosi stos sieciowy i laczy sie.
 *
 * Wymaga zainicjalizowanego NVS - WiFi trzyma tam kalibracje toru radiowego.
 *
 * @return ESP_ERR_INVALID_STATE przy powtornym wywolaniu. W ESP-IDF 6
 *         samo esp_wifi_init() tez zwraca ten blad przy drugim wywolaniu
 *         (w wersji 5 zwracalo ESP_OK), wiec komponent pilnuje wlasnego
 *         stanu, zeby blad byl czytelny.
 */
esp_err_t flora_wifi_init(const flora_wifi_cfg_t *cfg);
esp_err_t flora_wifi_deinit(void);

/** Rozlacza i zatrzymuje ponawianie. Stos zostaje podniesiony. */
esp_err_t flora_wifi_stop(void);

/** Wznawia laczenie po flora_wifi_stop(). Zeruje licznik prob. */
esp_err_t flora_wifi_start(void);

/**
 * @brief Czeka na polaczenie. Do prostych sekwencji startowych.
 *
 * @return ESP_OK gdy polaczono, ESP_ERR_TIMEOUT gdy minal czas,
 *         ESP_FAIL gdy wyczerpano limit prob
 */
esp_err_t flora_wifi_wait_connected(uint32_t timeout_ms);

bool flora_wifi_is_connected(void);
void flora_wifi_get_info(flora_wifi_info_t *out);

/** Nazwa powodu rozlaczenia, do logu. */
const char *flora_wifi_reason_str(uint8_t reason);

#ifdef __cplusplus
}
#endif
