/**
 * @file flora_link.h
 * @brief Transport ramek po UART miedzy bryla a glowica masztu.
 *
 * Komponent jest GENERYCZNY: zna tylko typ wiadomosci (uint8_t) i surowy
 * bufor. Nie wie nic o robocie - znaczenie wiadomosci opisuje flora_msgs,
 * dolaczany przez aplikacje, nie przez ten komponent.
 *
 * Co bierze na siebie:
 *   - ramkowanie i CRC (patrz flora_link_frame.h)
 *   - zadanie odbiorcze na kolejce zdarzen UART
 *   - potwierdzanie i ponawianie komend
 *   - wykrywanie zycia i smierci drugiej strony
 *   - liczniki do diagnozy kabla
 *
 * DWA POZIOMY NIEZAWODNOSCI. Telemetria idzie bez potwierdzania, bo zgubiona
 * ramka odtwarza sie sama przy nastepnej - potwierdzanie 20 razy na sekunde
 * to czysty narzut. Komendy sa rzadkie i musza dojsc, wiec dostaja numer
 * sekwencyjny, potwierdzenie i ponowienie.
 *
 * DOSTARCZANIE "CO NAJMNIEJ RAZ". Jesli odbiorca wykona komende i straci
 * zasilanie przed wyslaniem potwierdzenia, nadawca ponowi, a odbiorca
 * wykona ja drugi raz. Dokladnie-raz wymagaloby zapisu stanu we flashu.
 * Dlatego KAZDA KOMENDA MUSI BYC IDEMPOTENTNA - formy absolutne, nie
 * przyrostowe. Tlumienie duplikatow jest warstwa dodatkowa, nie gwarancja.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "driver/uart.h"
#include "flora_link_frame.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Ile roznych typow wiadomosci moze miec zarejestrowany handler. */
#define FLORA_LINK_MAX_HANDLERS 16

/**
 * Handler wiadomosci. Wolany z zadania odbiorczego, wiec moze logowac
 * i wolac API - ale musi byc KROTKI, bo blokuje odbior kolejnych ramek.
 * Ciezka prace oddaj do wlasnego zadania przez kolejke.
 */
typedef void (*flora_link_msg_cb_t)(uint8_t type, const void *payload,
                                    uint8_t len, void *ctx);

/** Zmiana stanu polaczenia z druga strona. */
typedef void (*flora_link_state_cb_t)(bool up, void *ctx);

/** Zgloszenie niezgodnej wersji protokolu - prawie zawsze znaczy, ze
 *  przeflashowano tylko jedna plytke. */
typedef void (*flora_link_version_cb_t)(uint8_t got, uint8_t expected, void *ctx);

typedef struct {
    uart_port_t port;
    int         tx_gpio;
    int         rx_gpio;
    uint32_t    baud_rate;

    uint8_t     proto_version;   /**< zwykle FLORA_MSGS_VERSION */
    flora_frame_size_fn size_of; /**< zwykle flora_msgs_payload_size */

    /**
     * Wartosc typu uzywana dla potwierdzen. Przychodzi z konfiguracji, bo
     * komponent celowo nie zna flora_msgs - ustaw na FLORA_MSG_ACK.
     * Funkcja size_of musi dla tej wartosci zwracac 0.
     */
    uint8_t     ack_type;

    size_t      rx_ring_bytes;
    uint32_t    task_stack_bytes;
    int         task_priority;

    /* --- potwierdzanie komend --- */
    uint32_t    ack_timeout_ms;  /**< ile czekac na potwierdzenie jednej proby */
    uint8_t     max_retries;

    /**
     * Po tylu ms bez zadnej odebranej ramki uznajemy polaczenie za zerwane.
     * Ma byc WIELOKROTNOSCIA najwolniejszego istotnego okresu - przy 20 Hz
     * telemetrii pojedyncza zgubiona ramka nie moze zrywac polaczenia.
     */
    uint32_t    peer_timeout_ms;

    flora_link_state_cb_t   on_state;
    void                   *state_ctx;
    flora_link_version_cb_t on_version_mismatch;
    void                   *version_ctx;
} flora_link_cfg_t;

#define FLORA_LINK_DEFAULT_CFG()        \
    {                                   \
        .port             = UART_NUM_1, \
        .baud_rate        = 115200,     \
        .rx_ring_bytes    = 1024,       \
        .task_stack_bytes = 4096,       \
        .task_priority    = 10,         \
        .ack_timeout_ms   = 100,        \
        .max_retries      = 3,          \
        .peer_timeout_ms  = 1000,       \
    }

typedef struct {
    uint32_t frames_ok;
    uint32_t frames_sent;
    uint32_t err_crc;
    uint32_t err_version;
    uint32_t err_type;
    uint32_t err_length;
    uint32_t err_flags;
    uint32_t uart_overflow;     /**< FIFO albo bufor pierscieniowy przepelniony */
    uint32_t acks_sent;
    uint32_t acks_received;
    uint32_t retransmits;
    uint32_t cmd_timeouts;
    uint32_t duplicates;        /**< powtorki stlumione przed handlerem */
    uint32_t bytes_rx;
} flora_link_stats_t;

esp_err_t flora_link_init(const flora_link_cfg_t *cfg);
esp_err_t flora_link_deinit(void);

/** Wysyla bez potwierdzania. Do telemetrii. */
esp_err_t flora_link_send(uint8_t type, const void *payload, uint8_t len);

/**
 * @brief Wysyla z potwierdzeniem i ponawianiem. Blokuje.
 *
 * Wywolania z roznych zadan sa szeregowane - w danej chwili w locie jest
 * jedna komenda. Calkowity czas blokady to czas oczekiwania na szeregowanie
 * plus `max_retries` razy `ack_timeout_ms`.
 *
 * @return ESP_ERR_TIMEOUT gdy wyczerpano ponowienia,
 *         ESP_ERR_INVALID_STATE gdy polaczenie zerwano w trakcie
 */
esp_err_t flora_link_send_reliable(uint8_t type, const void *payload, uint8_t len,
                                   uint32_t acquire_timeout_ms);

/**
 * @brief Przerywa zalegla komende natychmiast.
 *
 * Do wywolania, zanim bryla odetnie zasilanie glowicy kluczem masztu -
 * inaczej komenda wypali wszystkie ponowienia do niezasilanego ukladu.
 * Komponent celowo nic nie wie o tym kluczu; to aplikacja wie, kiedy
 * zamierza wylaczyc druga strone.
 */
esp_err_t flora_link_abort_pending(void);

esp_err_t flora_link_on(uint8_t type, flora_link_msg_cb_t cb, void *ctx);

bool flora_link_is_up(void);
void flora_link_get_stats(flora_link_stats_t *out);
void flora_link_reset_stats(void);

/**
 * @brief Zwiera nadawanie z odbiorem wewnatrz ukladu.
 *
 * Pozwala przejsc caly tor - ramkowanie, CRC, potwierdzanie, ponawianie -
 * na jednej plytce, bez drugiej strony i bez kabla.
 */
esp_err_t flora_link_set_loopback(bool enable);

#ifdef __cplusplus
}
#endif
