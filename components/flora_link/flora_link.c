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

static struct {
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
} s_l;

/* ================= NADAWANIE ================= */

/** Sklada i wysyla jedna ramke. Jedyna droga bajtow na kabel. */
static esp_err_t tx_frame(uint8_t type, uint8_t flags, uint8_t seq,
                          const void *payload, uint8_t len)
{
    xSemaphoreTake(s_l.tx_lock, portMAX_DELAY);

    size_t n = flora_frame_build(s_l.tx_buf, s_l.cfg.proto_version,
                                 type, flags, seq, payload, len);
    esp_err_t ret = ESP_OK;
    if (n == 0) {
        ret = ESP_ERR_INVALID_ARG;
    } else if (uart_write_bytes(s_l.cfg.port, s_l.tx_buf, n) != (int)n) {
        ret = ESP_FAIL;
    } else {
        s_l.stats.frames_sent++;
    }

    xSemaphoreGive(s_l.tx_lock);
    return ret;
}

esp_err_t flora_link_send(uint8_t type, const void *payload, uint8_t len)
{
    ESP_RETURN_ON_FALSE(s_l.initialized, ESP_ERR_INVALID_STATE, TAG, "nie zainicjalizowano");
    return tx_frame(type, 0, 0, payload, len);
}

esp_err_t flora_link_send_reliable(uint8_t type, const void *payload, uint8_t len,
                                   uint32_t acquire_timeout_ms)
{
    ESP_RETURN_ON_FALSE(s_l.initialized, ESP_ERR_INVALID_STATE, TAG, "nie zainicjalizowano");

    if (xSemaphoreTake(s_l.cmd_lock, pdMS_TO_TICKS(acquire_timeout_ms)) != pdTRUE) {
        ESP_LOGW(TAG, "nie udalo sie zajac toru komend w %" PRIu32 " ms", acquire_timeout_ms);
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t ret = ESP_ERR_TIMEOUT;
    uint8_t seq = s_l.next_seq++;

    xSemaphoreTake(s_l.ack_sig, 0);      /* kasujemy sygnal po poprzedniej komendzie */
    s_l.pending_seq    = seq;
    s_l.pending_type   = type;
    s_l.pending_active = true;

    for (int attempt = 0; attempt <= s_l.cfg.max_retries; attempt++) {
        if (attempt > 0) {
            s_l.stats.retransmits++;
            ESP_LOGW(TAG, "ponowienie %d dla typu 0x%02X, numer %u", attempt, type, seq);
        }
        ret = tx_frame(type, FLORA_FLAG_ACK_REQ, seq, payload, len);
        if (ret != ESP_OK) {
            break;
        }
        if (xSemaphoreTake(s_l.ack_sig, pdMS_TO_TICKS(s_l.cfg.ack_timeout_ms)) == pdTRUE) {
            ret = ESP_OK;
            goto done;
        }
        /* Polaczenie zerwane w trakcie - dalsze ponawianie nie ma sensu. */
        if (!s_l.pending_active) {
            ret = ESP_ERR_INVALID_STATE;
            goto done;
        }
    }

    if (ret == ESP_ERR_TIMEOUT) {
        s_l.stats.cmd_timeouts++;
        ESP_LOGE(TAG, "brak potwierdzenia typu 0x%02X po %u probach", type,
                 s_l.cfg.max_retries + 1);
    }

done:
    s_l.pending_active = false;
    xSemaphoreGive(s_l.cmd_lock);
    return ret;
}

esp_err_t flora_link_abort_pending(void)
{
    ESP_RETURN_ON_FALSE(s_l.initialized, ESP_ERR_INVALID_STATE, TAG, "nie zainicjalizowano");
    if (s_l.pending_active) {
        s_l.pending_active = false;
        xSemaphoreGive(s_l.ack_sig);   /* budzimy czekajacego, zobaczy brak pending */
    }
    return ESP_OK;
}

/* ================= ZYCIE DRUGIEJ STRONY ================= */

static void set_link_state(bool up)
{
    if (s_l.link_up == up) {
        return;
    }
    s_l.link_up = up;
    ESP_LOGI(TAG, "polaczenie %s", up ? "nawiazane" : "zerwane");

    if (!up) {
        /* Druga strona zniknela. Zalegla komenda nie ma do kogo dojsc,
         * a stan tlumienia duplikatow traci waznosc - po restarcie
         * glowica zacznie numerowac od nowa. */
        flora_link_abort_pending();
        s_l.have_last_acked = false;
    }
    if (s_l.cfg.on_state) {
        s_l.cfg.on_state(up, s_l.cfg.state_ctx);
    }
}

static void liveness_cb(void *arg)
{
    int64_t silence_ms = (esp_timer_get_time() - s_l.last_rx_us) / 1000;
    if (s_l.link_up && silence_ms > (int64_t)s_l.cfg.peer_timeout_ms) {
        set_link_state(false);
    }
}

/* ================= ODBIOR ================= */

static void dispatch(const flora_frame_t *f)
{
    for (int i = 0; i < FLORA_LINK_MAX_HANDLERS; i++) {
        if (s_l.handlers[i].used && s_l.handlers[i].type == f->type) {
            s_l.handlers[i].cb(f->type, f->payload, f->len, s_l.handlers[i].ctx);
            return;
        }
    }
}

static void handle_frame(const flora_frame_t *f)
{
    s_l.stats.frames_ok++;
    s_l.last_rx_us = esp_timer_get_time();
    set_link_state(true);

    /* Potwierdzenie zaleglej komendy. Dopasowanie po numerze ORAZ typie -
     * spozniony ACK na komende, z ktorej juz zrezygnowalismy, nie moze
     * przypadkiem zamknac nowej. */
    if (f->flags & FLORA_FLAG_IS_ACK) {
        s_l.stats.acks_received++;
        if (s_l.pending_active && f->seq == s_l.pending_seq) {
            xSemaphoreGive(s_l.ack_sig);
        }
        return;
    }

    if (f->flags & FLORA_FLAG_ACK_REQ) {
        /* Potwierdzamy PRZED przekazaniem do handlera, zeby wolny handler
         * nie wywolal niepotrzebnej retransmisji po drugiej stronie. */
        tx_frame(s_l.cfg.ack_type, FLORA_FLAG_IS_ACK, f->seq, NULL, 0);
        s_l.stats.acks_sent++;

        if (s_l.have_last_acked && f->seq == s_l.last_acked_seq) {
            /* Powtorka po zgubionym potwierdzeniu. Potwierdzamy ponownie,
             * ale NIE wykonujemy drugi raz. */
            s_l.stats.duplicates++;
            return;
        }
        s_l.last_acked_seq  = f->seq;
        s_l.have_last_acked = true;
    }

    dispatch(f);
}

static void rx_task(void *arg)
{
    uart_event_t ev;
    uint8_t *chunk = malloc(s_l.cfg.rx_ring_bytes);
    if (!chunk) {
        ESP_LOGE(TAG, "brak pamieci na bufor odbiorczy");
        vTaskDelete(NULL);
        return;
    }

    while (1) {
        if (xQueueReceive(s_l.uart_queue, &ev, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        switch (ev.type) {
            case UART_DATA: {
                /* Jedno zdarzenie moze zawierac kilka ramek albo kawalek
                 * jednej - sterownik wyzwala je na progu FIFO albo na
                 * przerwie w transmisji. Parser bajt po bajcie jest na to
                 * odporny z definicji. */
                size_t n = ev.size > s_l.cfg.rx_ring_bytes ? s_l.cfg.rx_ring_bytes : ev.size;
                int got = uart_read_bytes(s_l.cfg.port, chunk, n, 0);
                for (int i = 0; i < got; i++) {
                    s_l.stats.bytes_rx++;
                    flora_parse_result_t r = flora_parser_feed(&s_l.parser, chunk[i]);
                    switch (r) {
                        case FLORA_PARSE_FRAME:       handle_frame(&s_l.parser.frame); break;
                        case FLORA_PARSE_ERR_CRC:     s_l.stats.err_crc++;     break;
                        case FLORA_PARSE_ERR_TYPE:    s_l.stats.err_type++;    break;
                        case FLORA_PARSE_ERR_LENGTH:  s_l.stats.err_length++;  break;
                        case FLORA_PARSE_ERR_FLAGS:   s_l.stats.err_flags++;   break;
                        case FLORA_PARSE_ERR_VERSION:
                            s_l.stats.err_version++;
                            if (s_l.cfg.on_version_mismatch) {
                                s_l.cfg.on_version_mismatch(s_l.parser.frame.version,
                                                            s_l.cfg.proto_version,
                                                            s_l.cfg.version_ctx);
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
                s_l.stats.uart_overflow++;
                ESP_LOGW(TAG, "przepelnienie bufora odbiorczego");
                uart_flush_input(s_l.cfg.port);
                xQueueReset(s_l.uart_queue);
                flora_parser_init(&s_l.parser, s_l.cfg.proto_version, s_l.cfg.size_of);
                break;

            default:
                break;
        }
    }
}

/* ================= HANDLERY ================= */

esp_err_t flora_link_on(uint8_t type, flora_link_msg_cb_t cb, void *ctx)
{
    ESP_RETURN_ON_FALSE(s_l.initialized, ESP_ERR_INVALID_STATE, TAG, "nie zainicjalizowano");
    ESP_RETURN_ON_FALSE(cb, ESP_ERR_INVALID_ARG, TAG, "cb == NULL");

    for (int i = 0; i < FLORA_LINK_MAX_HANDLERS; i++) {
        if (s_l.handlers[i].used && s_l.handlers[i].type == type) {
            s_l.handlers[i].cb  = cb;      /* nadpisanie istniejacego */
            s_l.handlers[i].ctx = ctx;
            return ESP_OK;
        }
    }
    for (int i = 0; i < FLORA_LINK_MAX_HANDLERS; i++) {
        if (!s_l.handlers[i].used) {
            s_l.handlers[i] = (handler_t){ .type = type, .cb = cb, .ctx = ctx, .used = true };
            return ESP_OK;
        }
    }
    ESP_RETURN_ON_FALSE(false, ESP_ERR_NO_MEM, TAG,
                        "brak miejsca na handler (limit %d)", FLORA_LINK_MAX_HANDLERS);
}

bool flora_link_is_up(void) { return s_l.initialized && s_l.link_up; }

void flora_link_get_stats(flora_link_stats_t *out)
{
    if (out && s_l.initialized) {
        *out = s_l.stats;
    }
}

void flora_link_reset_stats(void)
{
    if (s_l.initialized) {
        memset(&s_l.stats, 0, sizeof(s_l.stats));
    }
}

esp_err_t flora_link_set_loopback(bool enable)
{
    ESP_RETURN_ON_FALSE(s_l.initialized, ESP_ERR_INVALID_STATE, TAG, "nie zainicjalizowano");
    return uart_set_loop_back(s_l.cfg.port, enable);
}

/* ================= INICJALIZACJA ================= */

esp_err_t flora_link_init(const flora_link_cfg_t *cfg)
{
    ESP_RETURN_ON_FALSE(cfg, ESP_ERR_INVALID_ARG, TAG, "cfg == NULL");
    ESP_RETURN_ON_FALSE(!s_l.initialized, ESP_ERR_INVALID_STATE, TAG, "juz zainicjalizowano");
    ESP_RETURN_ON_FALSE(cfg->peer_timeout_ms >= cfg->ack_timeout_ms, ESP_ERR_INVALID_ARG, TAG,
                        "peer_timeout_ms musi byc nie mniejsze niz ack_timeout_ms");
    /* Gdyby size_of nie znal typu ACK albo przypisal mu niezerowa dlugosc,
     * parser odrzucalby wlasne potwierdzenia i kazda komenda konczylaby sie
     * timeoutem - blad trudny do zdiagnozowania, wiec lapiemy go tutaj. */
    ESP_RETURN_ON_FALSE(!cfg->size_of || cfg->size_of(cfg->ack_type) == 0,
                        ESP_ERR_INVALID_ARG, TAG,
                        "typ ACK (0x%02X) musi miec zerowy payload wg size_of",
                        cfg->ack_type);

    memset(&s_l, 0, sizeof(s_l));
    s_l.cfg = *cfg;

    esp_err_t ret = ESP_OK;
    s_l.tx_lock  = xSemaphoreCreateMutex();
    s_l.cmd_lock = xSemaphoreCreateMutex();
    s_l.ack_sig  = xSemaphoreCreateBinary();
    ESP_GOTO_ON_FALSE(s_l.tx_lock && s_l.cmd_lock && s_l.ack_sig,
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
                                          &s_l.uart_queue, 0),
                      fail, TAG, "uart_driver_install");
    ESP_GOTO_ON_ERROR(uart_param_config(cfg->port, &uc), fail, TAG, "uart_param_config");
    ESP_GOTO_ON_ERROR(uart_set_pin(cfg->port, cfg->tx_gpio, cfg->rx_gpio,
                                   UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE),
                      fail, TAG, "uart_set_pin");

    flora_parser_init(&s_l.parser, cfg->proto_version, cfg->size_of);
    s_l.last_rx_us = esp_timer_get_time();

    const esp_timer_create_args_t args = { .callback = liveness_cb, .name = "link_alive" };
    ESP_GOTO_ON_ERROR(esp_timer_create(&args, &s_l.liveness_timer), fail, TAG, "timer");
    ESP_GOTO_ON_ERROR(esp_timer_start_periodic(s_l.liveness_timer,
                                               (uint64_t)cfg->peer_timeout_ms * 1000 / 4),
                      fail, TAG, "start timera");

    s_l.initialized = true;
    ESP_GOTO_ON_FALSE(xTaskCreate(rx_task, "flora_link_rx", cfg->task_stack_bytes,
                                  NULL, cfg->task_priority, &s_l.rx_task) == pdPASS,
                      ESP_ERR_NO_MEM, fail_started, TAG, "zadanie odbiorcze");

    ESP_LOGI(TAG, "UART%d gotowy: TX=%d RX=%d, %" PRIu32 " baud, wersja protokolu %u",
             cfg->port, cfg->tx_gpio, cfg->rx_gpio, cfg->baud_rate, cfg->proto_version);
    return ESP_OK;

fail_started:
    s_l.initialized = false;
fail:
    if (s_l.liveness_timer) { esp_timer_delete(s_l.liveness_timer); }
    if (s_l.uart_queue)     { uart_driver_delete(cfg->port); }
    if (s_l.ack_sig)        { vSemaphoreDelete(s_l.ack_sig); }
    if (s_l.cmd_lock)       { vSemaphoreDelete(s_l.cmd_lock); }
    if (s_l.tx_lock)        { vSemaphoreDelete(s_l.tx_lock); }
    memset(&s_l, 0, sizeof(s_l));
    return ret;
}

esp_err_t flora_link_deinit(void)
{
    ESP_RETURN_ON_FALSE(s_l.initialized, ESP_ERR_INVALID_STATE, TAG, "nie zainicjalizowano");

    s_l.initialized = false;
    if (s_l.rx_task) { vTaskDelete(s_l.rx_task); }
    esp_timer_stop(s_l.liveness_timer);
    esp_timer_delete(s_l.liveness_timer);
    uart_driver_delete(s_l.cfg.port);
    vSemaphoreDelete(s_l.ack_sig);
    vSemaphoreDelete(s_l.cmd_lock);
    vSemaphoreDelete(s_l.tx_lock);
    memset(&s_l, 0, sizeof(s_l));
    return ESP_OK;
}
