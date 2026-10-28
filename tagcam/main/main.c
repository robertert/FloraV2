/**
 * Doniczka-robot, modul kamery znacznikow (trzeci modul, w maszcie).
 *
 * Zadanie: szukac znacznikow AprilTag na palikach w rogach obszaru
 * roboczego i wysylac SUROWE detekcje - identyfikator, odleglosc, azymut
 * i pewnosc. Pozycji robota nie liczy; robi to filtr Kalmana w bryle,
 * ktory sam wazy kazda detekcje.
 *
 * W maszcie jest tylko jeden kabel w dol, wiec modul nie ma wlasnego
 * polaczenia z bryla - gada z glowica, a ta przekazuje ramki dalej.
 *
 * STAN: szkielet lacza z atrapa detekcji. Samo przetwarzanie obrazu
 * i wykrywanie AprilTag jeszcze nie istnieje.
 */
#include <inttypes.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "flora_cam_board.h"
#include "flora_link.h"
#include "flora_msgs.h"

static const char *TAG = "tagcam";

static flora_link_handle_t s_link;

static void on_link_state(bool up, void *ctx)
{
    ESP_LOGI(TAG, "lacze do glowicy %s", up ? "nawiazane" : "zerwane");
    if (up) {
        /* Glowica (albo bryla) musi wiedziec, z czym rozmawia i od kiedy. */
        flora_hello_t hello = {
            .msgs_version    = FLORA_MSGS_VERSION,
            .sensors_present = 0,          /* modul nie ma czujnikow I2C */
            .fw_build        = 0,
        };
        flora_link_send(s_link, FLORA_MSG_HELLO, &hello, sizeof hello);
    }
}

static void report_task(void *arg)
{
    uint32_t frames = 0;

    while (1) {
        /* TODO: tu wejdzie wykrywanie AprilTag. Na razie atrapa, zeby
         * przetestowac caly tor: modul -> glowica -> bryla. */
        flora_tags_t msg = {
            .timestamp_us = (uint32_t)esp_timer_get_time(),
            .count        = 0,
        };
        flora_link_send(s_link, FLORA_MSG_TAGS, &msg, sizeof msg);
        frames++;

        /* Stan modulu rzadziej niz detekcje. */
        if (frames % 50 == 0) {
            flora_cam_status_t st = {
                .uptime_ms        = (uint32_t)(esp_timer_get_time() / 1000),
                .frames_processed = 50,
                .error_count      = 0,
                .camera_ok        = 0,   /* kamera jeszcze nieobslugiwana */
            };
            flora_link_send(s_link, FLORA_MSG_CAM_STATUS, &st, sizeof st);
        }

        vTaskDelay(pdMS_TO_TICKS(200));   /* 5 Hz - znaczniki nie uciekaja */
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "doniczka-robot, modul kamery znacznikow");

    flora_link_cfg_t cfg = FLORA_LINK_DEFAULT_CFG();
    cfg.name          = "glowica";
    cfg.port          = FLORA_CAM_UART_PORT;
    cfg.tx_gpio       = FLORA_CAM_UART_TX;
    cfg.rx_gpio       = FLORA_CAM_UART_RX;
    cfg.baud_rate     = FLORA_CAM_BAUD;
    cfg.proto_version = FLORA_MSGS_VERSION;
    cfg.size_of       = flora_msgs_payload_size;
    cfg.ack_type      = FLORA_MSG_ACK;
    cfg.on_state      = on_link_state;

    if (flora_link_init(&cfg, &s_link) != ESP_OK) {
        ESP_LOGE(TAG, "nie udalo sie uruchomic lacza");
        return;
    }

    xTaskCreate(report_task, "tags", 4096, NULL, 5, NULL);
}
