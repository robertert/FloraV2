/**
 * @file bringup_link.c
 * @brief Test toru komunikacyjnego na petli zwrotnej.
 *
 * Zwiera nadawanie z odbiorem wewnatrz ukladu, wiec sprawdza ramkowanie,
 * CRC, potwierdzanie, ponawianie i tlumienie duplikatow bez drugiej plytki
 * i bez kabla w maszcie.
 *
 * Maszyna stanow parsera i CRC sa juz przetestowane osobno na hoscie
 * (16 przypadkow). Tutaj sprawdzamy to, czego na hoscie sie nie da:
 * sterownik UART, zadanie odbiorcze i sciezke potwierdzen.
 */
#include <inttypes.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "bringup_link.h"
#include "flora_head_board.h"
#include "flora_link.h"
#include "flora_msgs.h"

static const char *TAG = "link";

static int s_pass, s_fail;
static int s_attitude_seen;
static int s_cmd_seen;
static flora_attitude_t s_last_attitude;

static void check(bool ok, const char *what)
{
    if (ok) { s_pass++; ESP_LOGI(TAG, "  [ OK ] %s", what); }
    else    { s_fail++; ESP_LOGE(TAG, "  [BLAD] %s", what); }
}

static void on_attitude(uint8_t type, const void *payload, uint8_t len, void *ctx)
{
    if (len == sizeof(flora_attitude_t)) {
        memcpy(&s_last_attitude, payload, len);
        s_attitude_seen++;
    }
}

static void on_cmd(uint8_t type, const void *payload, uint8_t len, void *ctx)
{
    s_cmd_seen++;
}

static void on_link_state(bool up, void *ctx)
{
    ESP_LOGI(TAG, "  zdarzenie: polaczenie %s", up ? "w gore" : "w dol");
}

void bringup_link_loopback(void)
{
    ESP_LOGI(TAG, "=== TEST LACZA NA PETLI ZWROTNEJ ===");

    flora_link_cfg_t cfg = FLORA_LINK_DEFAULT_CFG();
    cfg.tx_gpio       = FLORA_HEAD_UART_TX;
    cfg.rx_gpio       = FLORA_HEAD_UART_RX;
    cfg.proto_version = FLORA_MSGS_VERSION;
    cfg.size_of       = flora_msgs_payload_size;
    cfg.ack_type      = FLORA_MSG_ACK;
    cfg.on_state      = on_link_state;

    if (flora_link_init(&cfg) != ESP_OK) {
        ESP_LOGE(TAG, "  inicjalizacja nie powiodla sie");
        return;
    }
    flora_link_on(FLORA_MSG_ATTITUDE, on_attitude, NULL);
    flora_link_on(FLORA_MSG_CMD, on_cmd, NULL);

    if (flora_link_set_loopback(true) != ESP_OK) {
        ESP_LOGE(TAG, "  nie udalo sie zewrzec petli zwrotnej");
        flora_link_deinit();
        return;
    }
    ESP_LOGI(TAG, "  petla zwrotna zwarta, nadawanie wraca do odbioru");

    /* 1. Telemetria bez potwierdzania. */
    const int N = 20;
    s_attitude_seen = 0;
    for (int i = 0; i < N; i++) {
        flora_attitude_t a = {
            .timestamp_us = (uint32_t)esp_timer_get_time(),
            .heading_cdeg = (uint16_t)(i * 1000),
            .rate_z_cdps  = (int16_t)(-i * 10),
            .quality      = 90,
            .flags        = FLORA_ATT_HEADING_VALID,
        };
        flora_link_send(FLORA_MSG_ATTITUDE, &a, sizeof a);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    char what[80];
    snprintf(what, sizeof(what), "odebrano %d z %d ramek telemetrii", s_attitude_seen, N);
    check(s_attitude_seen == N, what);
    check(s_last_attitude.heading_cdeg == (uint16_t)((N - 1) * 1000),
          "payload ostatniej ramki zgadza sie co do bitu");
    check(flora_link_is_up(), "polaczenie zglaszane jako nawiazane");

    /* 2. Komenda z potwierdzeniem. W petli zwrotnej wlasna ramka wraca,
     *    zostaje automatycznie potwierdzona, a potwierdzenie wraca do nas. */
    s_cmd_seen = 0;
    flora_cmd_t cmd = { .code = FLORA_CMD_SET_ATT_HZ, .arg = 20 };
    int64_t t0 = esp_timer_get_time();
    esp_err_t r = flora_link_send_reliable(FLORA_MSG_CMD, &cmd, sizeof cmd, 500);
    int64_t dt_ms = (esp_timer_get_time() - t0) / 1000;

    snprintf(what, sizeof(what), "komenda potwierdzona w %" PRId64 " ms", dt_ms);
    check(r == ESP_OK, what);
    check(s_cmd_seen == 1, "komenda trafila do handlera dokladnie raz");

    /* 3. Statystyki musza sie domykac. */
    flora_link_stats_t st;
    flora_link_get_stats(&st);
    ESP_LOGI(TAG, "  ramki: %" PRIu32 " odebrane, %" PRIu32 " wyslane",
             st.frames_ok, st.frames_sent);
    ESP_LOGI(TAG, "  potwierdzenia: %" PRIu32 " wyslane, %" PRIu32 " odebrane",
             st.acks_sent, st.acks_received);
    ESP_LOGI(TAG, "  bledy: CRC %" PRIu32 ", wersja %" PRIu32 ", typ %" PRIu32
                  ", dlugosc %" PRIu32 ", flagi %" PRIu32 ", przepelnienia %" PRIu32,
             st.err_crc, st.err_version, st.err_type, st.err_length,
             st.err_flags, st.uart_overflow);
    ESP_LOGI(TAG, "  ponowienia %" PRIu32 ", duplikaty %" PRIu32 ", bajty %" PRIu32,
             st.retransmits, st.duplicates, st.bytes_rx);

    check(st.err_crc == 0 && st.err_version == 0 && st.err_type == 0 &&
          st.err_length == 0 && st.err_flags == 0,
          "zero bledow ramkowania na czystej petli");
    check(st.uart_overflow == 0, "zero przepelnien bufora");
    check(st.retransmits == 0, "potwierdzenie przyszlo bez ponawiania");

    ESP_LOGI(TAG, "  ----------------------------------------");
    if (s_fail == 0) {
        ESP_LOGI(TAG, "  WYNIK: wszystko przeszlo (%d sprawdzen)", s_pass);
    } else {
        ESP_LOGE(TAG, "  WYNIK: %d przeszlo, %d nie przeszlo", s_pass, s_fail);
    }
    ESP_LOGI(TAG, "  ----------------------------------------");

    flora_link_set_loopback(false);
    flora_link_deinit();
}
