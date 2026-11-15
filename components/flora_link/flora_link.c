#include <inttypes.h>
#include <stdlib.h>
#include <string.h>

#include "flora_link.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "flora_link";

typedef struct {
    uint8_t             type;
    flora_link_msg_cb_t cb;
    void               *ctx;
    bool                used;
} handler_t;

struct flora_link_s {
    flora_link_cfg_t cfg;

    QueueHandle_t     uart_queue;
    TaskHandle_t      rx_task;
    flora_parser_t    parser;

    /* Nadawanie. Dwa niezalezne zrodla bajtow - telemetria i komendy -
     * wiec zlozenie i zapis ramki musi byc sekcja krytyczna, inaczej
     * bajty obu ramek przeplotlyby sie na kablu. */
    SemaphoreHandle_t tx_lock;
    uint8_t           tx_buf[FLORA_FRAME_MAX_LEN];

    /* Komendy z potwierdzeniem. Osobna blokada: szereguje wywolujacych,
     * zeby w locie byla jedna komenda naraz. */
    SemaphoreHandle_t cmd_lock;
    SemaphoreHandle_t ack_sig;
    bool              pending_active;
    uint8_t           pending_seq;
    uint8_t           pending_type;
    uint8_t           next_seq;

    /* Tlumienie duplikatow po stronie odbiorcy. `have_last` jest
     * konieczne: bez niego swiezo zrestartowana glowica ma zapamietany
     * numer 0 i pierwsza komenda o numerze 0 zostalaby uznana za powtorke,
     * odeslano by na nia potwierdzenie, a handler by jej nie zobaczyl. */
    uint8_t           last_acked_seq;
    bool              have_last_acked;

    /* Zycie drugiej strony. */
    esp_timer_handle_t liveness_timer;
    int64_t            last_rx_us;
    bool               link_up;

    handler_t          handlers[FLORA_LINK_MAX_HANDLERS];
    flora_link_stats_t stats;
    bool               initialized;
};

/* ================= NADAWANIE ================= */

/** Sklada i wysyla jedna ramke. Jedyna droga bajtow na kabel. */
static esp_err_t tx_frame(flora_link_handle_t h, uint8_t type, uint8_t flags,
                          uint8_t seq, const void *payload, uint8_t len)
{
    xSemaphoreTake(h->tx_lock, portMAX_DELAY);

    size_t n = flora_frame_build(h->tx_buf, h->cfg.proto_version,
                                 type, flags, seq, payload, len);
    esp_err_t ret = ESP_OK;
    if (n == 0) {
        ret = ESP_ERR_INVALID_ARG;
    } else if (uart_write_bytes(h->cfg.port, h->tx_buf, n) != (int)n) {
        ret = ESP_FAIL;
    } else {
        h->stats.frames_sent++;
    }

    xSemaphoreGive(h->tx_lock);
    return ret;
}

esp_err_t flora_link_send(flora_link_handle_t h, uint8_t type,
                          const void *payload, uint8_t len)
{
    ESP_RETURN_ON_FALSE(h, ESP_ERR_INVALID_ARG, TAG, "h == NULL");
    ESP_RETURN_ON_FALSE(h->initialized, ESP_ERR_INVALID_STATE, TAG, "nie zainicjalizowano");
    return tx_frame(h, type, 0, 0, payload, len);
}

esp_err_t flora_link_send_reliable(flora_link_handle_t h, uint8_t type,
                                   const void *payload, uint8_t len,
                                   uint32_t acquire_timeout_ms)
{
    ESP_RETURN_ON_FALSE(h, ESP_ERR_INVALID_ARG, TAG, "h == NULL");
    ESP_RETURN_ON_FALSE(h->initialized, ESP_ERR_INVALID_STATE, TAG, "nie zainicjalizowano");

    if (xSemaphoreTake(h->cmd_lock, pdMS_TO_TICKS(acquire_timeout_ms)) != pdTRUE) {
        ESP_LOGW(TAG, "nie udalo sie zajac toru komend w %" PRIu32 " ms", acquire_timeout_ms);
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t ret = ESP_ERR_TIMEOUT;
    uint8_t seq = h->next_seq++;

    xSemaphoreTake(h->ack_sig, 0);      /* kasujemy sygnal po poprzedniej komendzie */
    h->pending_seq    = seq;
    h->pending_type   = type;
    h->pending_active = true;

    for (int attempt = 0; attempt <= h->cfg.max_retries; attempt++) {
        if (attempt > 0) {
            h->stats.retransmits++;
            ESP_LOGW(TAG, "ponowienie %d dla typu 0x%02X, numer %u", attempt, type, seq);
        }
        ret = tx_frame(h, type, FLORA_FLAG_ACK_REQ, seq, payload, len);
        if (ret != ESP_OK) {
            break;
        }
        if (xSemaphoreTake(h->ack_sig, pdMS_TO_TICKS(h->cfg.ack_timeout_ms)) == pdTRUE) {
            ret = ESP_OK;
            goto done;
        }
        /* Polaczenie zerwane w trakcie - dalsze ponawianie nie ma sensu. */
        if (!h->pending_active) {
            ret = ESP_ERR_INVALID_STATE;
            goto done;
        }
    }

    if (ret == ESP_ERR_TIMEOUT) {
        h->stats.cmd_timeouts++;
        ESP_LOGE(TAG, "brak potwierdzenia typu 0x%02X po %u probach", type,
                 h->cfg.max_retries + 1);
    }

done:
    h->pending_active = false;
    xSemaphoreGive(h->cmd_lock);
    return ret;
}

esp_err_t flora_link_abort_pending(flora_link_handle_t h)
{
    ESP_RETURN_ON_FALSE(h, ESP_ERR_INVALID_ARG, TAG, "h == NULL");
    ESP_RETURN_ON_FALSE(h->initialized, ESP_ERR_INVALID_STATE, TAG, "nie zainicjalizowano");
    if (h->pending_active) {
        h->pending_active = false;
        xSemaphoreGive(h->ack_sig);   /* budzimy czekajacego, zobaczy brak pending */
    }
    return ESP_OK;
}

/* ================= ZYCIE DRUGIEJ STRONY ================= */

static void set_link_state(flora_link_handle_t h, bool up)
{
    if (h->link_up == up) {
        return;
    }
    h->link_up = up;
    ESP_LOGI(TAG, "[%s] polaczenie %s", h->cfg.name ? h->cfg.name : "?",
             up ? "nawiazane" : "zerwane");

    if (!up) {
        /* Druga strona zniknela. Zalegla komenda nie ma do kogo dojsc,
         * a stan tlumienia duplikatow traci waznosc - po restarcie
         * glowica zacznie numerowac od nowa. */
        flora_link_abort_pending(h);
        h->have_last_acked = false;
    }
    if (h->cfg.on_state) {
        h->cfg.on_state(up, h->cfg.state_ctx);
    }
}

static void liveness_cb(void *arg)
{
    flora_link_handle_t h = (flora_link_handle_t)arg;
    int64_t silence_ms = (esp_timer_get_time() - h->last_rx_us) / 1000;
    if (h->link_up && silence_ms > (int64_t)h->cfg.peer_timeout_ms) {
        set_link_state(h, false);
    }
}

/* ================= ODBIOR ================= */

static void dispatch(flora_link_handle_t h, const flora_frame_t *f)
{
    for (int i = 0; i < FLORA_LINK_MAX_HANDLERS; i++) {
        if (h->handlers[i].used && h->handlers[i].type == f->type) {
            h->handlers[i].cb(f->type, f->payload, f->len, h->handlers[i].ctx);
            return;
        }
    }
}

static void handle_frame(flora_link_handle_t h, const flora_frame_t *f)
{
    h->stats.frames_ok++;
    h->last_rx_us = esp_timer_get_time();
    set_link_state(h, true);

    /* Potwierdzenie zaleglej komendy. Dopasowanie po numerze ORAZ typie -
     * spozniony ACK na komende, z ktorej juz zrezygnowalismy, nie moze
     * przypadkiem zamknac nowej. */
    if (f->flags & FLORA_FLAG_IS_ACK) {
        h->stats.acks_received++;
        if (h->pending_active && f->seq == h->pending_seq) {
            xSemaphoreGive(h->ack_sig);
        }
        return;
    }

    if (f->flags & FLORA_FLAG_ACK_REQ) {
        /* Potwierdzamy PRZED przekazaniem do handlera, zeby wolny handler
         * nie wywolal niepotrzebnej retransmisji po drugiej stronie. */
        tx_frame(h, h->cfg.ack_type, FLORA_FLAG_IS_ACK, f->seq, NULL, 0);
        h->stats.acks_sent++;

        if (h->have_last_acked && f->seq == h->last_acked_seq) {
            /* Powtorka po zgubionym potwierdzeniu. Potwierdzamy ponownie,
             * ale NIE wykonujemy drugi raz. */
            h->stats.duplicates++;
            return;
        }
        h->last_acked_seq  = f->seq;
        h->have_last_acked = true;
    }

    dispatch(h, f);
}

static void rx_task(void *arg)
{
    flora_link_handle_t h = (flora_link_handle_t)arg;
    uart_event_t ev;
    uint8_t *chunk = malloc(h->cfg.rx_ring_bytes);
    if (!chunk) {
        ESP_LOGE(TAG, "brak pamieci na bufor odbiorczy");
        vTaskDelete(NULL);
        return;
    }

    while (1) {
        if (xQueueReceive(h->uart_queue, &ev, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        switch (ev.type) {
            case UART_DATA: {
                /* Jedno zdarzenie moze zawierac kilka ramek albo kawalek
                 * jednej - sterownik wyzwala je na progu FIFO albo na
                 * przerwie w transmisji. Parser bajt po bajcie jest na to
                 * odporny z definicji. */
                size_t n = ev.size > h->cfg.rx_ring_bytes ? h->cfg.rx_ring_bytes : ev.size;
                int got = uart_read_bytes(h->cfg.port, chunk, n, 0);
                for (int i = 0; i < got; i++) {
                    h->stats.bytes_rx++;
                    flora_parse_result_t r = flora_parser_feed(&h->parser, chunk[i]);
                    switch (r) {
                        case FLORA_PARSE_FRAME:       handle_frame(h, &h->parser.frame); break;
                        case FLORA_PARSE_ERR_CRC:     h->stats.err_crc++;     break;
                        case FLORA_PARSE_ERR_TYPE:    h->stats.err_type++;    break;
                        case FLORA_PARSE_ERR_LENGTH:  h->stats.err_length++;  break;
                        case FLORA_PARSE_ERR_FLAGS:   h->stats.err_flags++;   break;
                        case FLORA_PARSE_ERR_VERSION:
                            h->stats.err_version++;
                            if (h->cfg.on_version_mismatch) {
                                h->cfg.on_version_mismatch(h->parser.frame.version,
                                                            h->cfg.proto_version,
                                                            h->cfg.version_ctx);
                            }
                            break;
                        default: break;
                    }
                }
                break;
            }

            case UART_FIFO_OVF:
            case UART_BUFFER_FULL:
                /* Bez obslugi tych zdarzen sterownik po cichu gubilby dane,
                 * a parser zostawalby w polowie ramki. */
                h->stats.uart_overflow++;
                ESP_LOGW(TAG, "[%s] przepelnienie bufora odbiorczego",
                         h->cfg.name ? h->cfg.name : "?");
                uart_flush_input(h->cfg.port);
                xQueueReset(h->uart_queue);
                flora_parser_init(&h->parser, h->cfg.proto_version, h->cfg.size_of);
                break;

            default:
                break;
        }
    }
}

/* ================= HANDLERY ================= */

esp_err_t flora_link_on(flora_link_handle_t h, uint8_t type,
                        flora_link_msg_cb_t cb, void *ctx)
{
    ESP_RETURN_ON_FALSE(h, ESP_ERR_INVALID_ARG, TAG, "h == NULL");
    ESP_RETURN_ON_FALSE(h->initialized, ESP_ERR_INVALID_STATE, TAG, "nie zainicjalizowano");
    ESP_RETURN_ON_FALSE(cb, ESP_ERR_INVALID_ARG, TAG, "cb == NULL");

    for (int i = 0; i < FLORA_LINK_MAX_HANDLERS; i++) {
        if (h->handlers[i].used && h->handlers[i].type == type) {
            h->handlers[i].cb  = cb;      /* nadpisanie istniejacego */
            h->handlers[i].ctx = ctx;
            return ESP_OK;
        }
    }
    for (int i = 0; i < FLORA_LINK_MAX_HANDLERS; i++) {
        if (!h->handlers[i].used) {
            h->handlers[i] = (handler_t){ .type = type, .cb = cb, .ctx = ctx, .used = true };
            return ESP_OK;
        }
    }
    ESP_RETURN_ON_FALSE(false, ESP_ERR_NO_MEM, TAG,
                        "brak miejsca na handler (limit %d)", FLORA_LINK_MAX_HANDLERS);
}

/** Handler przekazujacy: wysyla odebrany payload na drugie lacze. */
static void relay_cb(uint8_t type, const void *payload, uint8_t len, void *ctx)
{
    flora_link_handle_t to = (flora_link_handle_t)ctx;
    esp_err_t ret = flora_link_send(to, type, payload, len);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "[%s] nie udalo sie przekazac typu 0x%02X: %s",
                 to->cfg.name ? to->cfg.name : "?", type, esp_err_to_name(ret));
    }
}

esp_err_t flora_link_relay(flora_link_handle_t from, flora_link_handle_t to, uint8_t type)
{
    ESP_RETURN_ON_FALSE(from && to, ESP_ERR_INVALID_ARG, TAG, "zly argument");
    ESP_RETURN_ON_FALSE(from != to, ESP_ERR_INVALID_ARG, TAG,
                        "przekazywanie na to samo lacze zapetliloby ramki");

    ESP_RETURN_ON_ERROR(flora_link_on(from, type, relay_cb, to), TAG,
                        "rejestracja przekazywania typu 0x%02X", type);
    ESP_LOGI(TAG, "przekazywanie 0x%02X: %s -> %s", type,
             from->cfg.name ? from->cfg.name : "?",
             to->cfg.name ? to->cfg.name : "?");
    return ESP_OK;
}

bool flora_link_is_up(flora_link_handle_t h) { return h && h->initialized && h->link_up; }

void flora_link_get_stats(flora_link_handle_t h, flora_link_stats_t *out)
{
    if (h && out && h->initialized) {
        *out = h->stats;
    }
}

void flora_link_reset_stats(flora_link_handle_t h)
{
    if (h && h->initialized) {
        memset(&h->stats, 0, sizeof(h->stats));
    }
}

esp_err_t flora_link_set_loopback(flora_link_handle_t h, bool enable)
{
    ESP_RETURN_ON_FALSE(h, ESP_ERR_INVALID_ARG, TAG, "h == NULL");
    ESP_RETURN_ON_FALSE(h->initialized, ESP_ERR_INVALID_STATE, TAG, "nie zainicjalizowano");
    return uart_set_loop_back(h->cfg.port, enable);
}

/* ================= INICJALIZACJA ================= */

esp_err_t flora_link_init(const flora_link_cfg_t *cfg, flora_link_handle_t *out)
{
    ESP_RETURN_ON_FALSE(cfg && out, ESP_ERR_INVALID_ARG, TAG, "zly argument");
    ESP_RETURN_ON_FALSE(cfg->peer_timeout_ms >= cfg->ack_timeout_ms, ESP_ERR_INVALID_ARG, TAG,
                        "peer_timeout_ms musi byc nie mniejsze niz ack_timeout_ms");
    /* Gdyby size_of nie znal typu ACK albo przypisal mu niezerowa dlugosc,
     * parser odrzucalby wlasne potwierdzenia i kazda komenda konczylaby sie
     * timeoutem - blad trudny do zdiagnozowania, wiec lapiemy go tutaj. */
    ESP_RETURN_ON_FALSE(!cfg->size_of || cfg->size_of(cfg->ack_type) == 0,
                        ESP_ERR_INVALID_ARG, TAG,
                        "typ ACK (0x%02X) musi miec zerowy payload wg size_of",
                        cfg->ack_type);

    flora_link_handle_t h = calloc(1, sizeof(struct flora_link_s));
    ESP_RETURN_ON_FALSE(h, ESP_ERR_NO_MEM, TAG, "brak pamieci na uchwyt lacza");
    h->cfg = *cfg;

    esp_err_t ret = ESP_OK;
    h->tx_lock  = xSemaphoreCreateMutex();
    h->cmd_lock = xSemaphoreCreateMutex();
    h->ack_sig  = xSemaphoreCreateBinary();
    ESP_GOTO_ON_FALSE(h->tx_lock && h->cmd_lock && h->ack_sig,
                      ESP_ERR_NO_MEM, fail, TAG, "brak pamieci na blokady");

    uart_config_t uc = {
        .baud_rate = (int)cfg->baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_GOTO_ON_ERROR(uart_driver_install(cfg->port, (int)cfg->rx_ring_bytes,
                                          (int)cfg->rx_ring_bytes, 20,
                                          &h->uart_queue, 0),
                      fail, TAG, "uart_driver_install");
    ESP_GOTO_ON_ERROR(uart_param_config(cfg->port, &uc), fail, TAG, "uart_param_config");
    ESP_GOTO_ON_ERROR(uart_set_pin(cfg->port, cfg->tx_gpio, cfg->rx_gpio,
                                   UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE),
                      fail, TAG, "uart_set_pin");

    flora_parser_init(&h->parser, cfg->proto_version, cfg->size_of);
    h->last_rx_us = esp_timer_get_time();

    const esp_timer_create_args_t args = { .callback = liveness_cb, .arg = h,
                                          .name = "link_alive" };
    ESP_GOTO_ON_ERROR(esp_timer_create(&args, &h->liveness_timer), fail, TAG, "timer");
    ESP_GOTO_ON_ERROR(esp_timer_start_periodic(h->liveness_timer,
                                               (uint64_t)cfg->peer_timeout_ms * 1000 / 4),
                      fail, TAG, "start timera");

    h->initialized = true;
    ESP_GOTO_ON_FALSE(xTaskCreate(rx_task, "flora_link_rx", cfg->task_stack_bytes,
                                  h, cfg->task_priority, &h->rx_task) == pdPASS,
                      ESP_ERR_NO_MEM, fail_started, TAG, "zadanie odbiorcze");

    ESP_LOGI(TAG, "[%s] UART%d gotowy: TX=%d RX=%d, %" PRIu32 " baud, wersja protokolu %u",
             cfg->name ? cfg->name : "?", cfg->port, cfg->tx_gpio, cfg->rx_gpio,
             cfg->baud_rate, cfg->proto_version);
    *out = h;
    return ESP_OK;

fail_started:
    h->initialized = false;
fail:
    if (h->liveness_timer) { esp_timer_delete(h->liveness_timer); }
    if (h->uart_queue)     { uart_driver_delete(cfg->port); }
    if (h->ack_sig)        { vSemaphoreDelete(h->ack_sig); }
    if (h->cmd_lock)       { vSemaphoreDelete(h->cmd_lock); }
    if (h->tx_lock)        { vSemaphoreDelete(h->tx_lock); }
    free(h);
    return ret;
}

esp_err_t flora_link_deinit(flora_link_handle_t h)
{
    ESP_RETURN_ON_FALSE(h && h->initialized, ESP_ERR_INVALID_STATE, TAG, "nie zainicjalizowano");

    h->initialized = false;
    if (h->rx_task) { vTaskDelete(h->rx_task); }
    esp_timer_stop(h->liveness_timer);
    esp_timer_delete(h->liveness_timer);
    uart_driver_delete(h->cfg.port);
    vSemaphoreDelete(h->ack_sig);
    vSemaphoreDelete(h->cmd_lock);
    vSemaphoreDelete(h->tx_lock);
    free(h);
    return ESP_OK;
}
