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
 * WYDAJNOSC: na ESP32-S3 przy 320x240 detektor AprilTag potrzebuje setek
 * milisekund na klatke, wiec realne tempo to pojedyncze klatki na sekunde.
 * Dla tego zastosowania wystarcza: znaczniki stoja na palikach, robot jedzie
 * wolno, a jedna poprawka pozycji na sekunde z nadmiarem pokrywa dryf
 * odometrii miedzy korektami.
 */
#include <inttypes.h>
#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "flora_cam_board.h"
#include "flora_camera.h"
#include "flora_link.h"
#include "flora_msgs.h"
#include "flora_tags.h"

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

/** Ile klatek pod rzad bez znacznika, zanim przestaniemy o tym pisac. */
#define QUIET_AFTER_EMPTY 10

static void detect_task(void *arg)
{
    uint32_t frames = 0, empty_streak = 0, errors = 0;
    uint32_t sum_ms = 0, measured = 0;

    while (1) {
        flora_camera_frame_t fr;
        if (flora_camera_grab(&fr, 1000) != ESP_OK) {
            errors++;
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        flora_tags_result_t res;
        esp_err_t r = flora_tags_detect(fr.data, fr.width, fr.height, &res);
        /* Bufor oddajemy NATYCHMIAST po przetworzeniu. Przy dwoch buforach
         * kamera moze juz lapac nastepna klatke, gdy my skladamy ramke. */
        flora_camera_release(&fr);

        if (r != ESP_OK) {
            errors++;
            continue;
        }
        frames++;
        sum_ms += res.process_ms;
        measured++;

        flora_tags_t msg = {
            .timestamp_us = (uint32_t)fr.timestamp_us,
            .count        = res.count > FLORA_TAGS_MAX ? FLORA_TAGS_MAX : res.count,
        };
        for (int i = 0; i < msg.count; i++) {
            const flora_tag_detection_t *d = &res.items[i];
            msg.tags[i].id           = d->id;
            msg.tags[i].distance_mm  = (uint16_t)(d->distance_mm > 65535.0f
                                                  ? 65535.0f : d->distance_mm);
            msg.tags[i].bearing_cdeg = (int16_t)lroundf(d->bearing_deg * 100.0f);
            msg.tags[i].quality      = d->quality;
        }
        flora_link_send(s_link, FLORA_MSG_TAGS, &msg, sizeof msg);

        if (res.count > 0) {
            empty_streak = 0;
            for (int i = 0; i < msg.count; i++) {
                ESP_LOGI(TAG, "znacznik %u: %.0f mm, %+.1f st, jakosc %u "
                              "(srodek %.0f,%.0f, ostrosc %.0f)",
                         res.items[i].id, res.items[i].distance_mm,
                         res.items[i].bearing_deg, res.items[i].quality,
                         res.items[i].center_x, res.items[i].center_y,
                         res.items[i].decision_margin);
            }
        } else if (++empty_streak == QUIET_AFTER_EMPTY) {
            /* Raz, nie w kazdej klatce - inaczej log zalewa sie informacja,
             * ze nic nie widac. */
            ESP_LOGI(TAG, "brak znacznikow w polu widzenia");
        }

        /* Co 50 klatek podsumowanie: realne tempo jest tu najwazniejsza
         * liczba, bo decyduje, czy modul nadaje sie do nawigacji. */
        if (frames % 50 == 0) {
            uint32_t avg = measured ? sum_ms / measured : 0;
            ESP_LOGI(TAG, "%" PRIu32 " klatek, srednio %" PRIu32 " ms na klatke "
                          "(%.1f kl/s), bledow %" PRIu32,
                     frames, avg, avg ? 1000.0f / (float)avg : 0.0f, errors);
            flora_cam_status_t st = {
                .uptime_ms        = (uint32_t)(esp_timer_get_time() / 1000),
                .frames_processed = 50,
                .error_count      = (uint16_t)errors,
                .camera_ok        = 1,
            };
            flora_link_send(s_link, FLORA_MSG_CAM_STATUS, &st, sizeof st);
            sum_ms = 0;
            measured = 0;
        }
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

    /* Kamera i detektor. Brak ktoregokolwiek nie blokuje lacza - modul
     * bedzie widoczny dla bryly i powie w HEAD_STATUS, ze kamera nie dziala. */
    flora_camera_cfg_t ccfg = FLORA_CAMERA_DEFAULT_CFG();
    ccfg.pins = (flora_camera_pins_t)FLORA_CAM_PINS();
    if (flora_camera_init(&ccfg) != ESP_OK) {
        ESP_LOGE(TAG, "kamera nie wystartowala - sprawdz pinout DVP i tasme");
        return;
    }

    flora_tags_cfg_t tcfg = FLORA_TAGS_DEFAULT_CFG();
    tcfg.tag_size_mm   = FLORA_CAM_TAG_SIZE_MM;
    tcfg.focal_px      = FLORA_CAM_FOCAL_PX;
    /* Pelna rozdzielczosc: zasieg 11 m zamiast 5,6, kosztem czterokrotnie
     * dluzszego przetwarzania. Wybor swiadomy - paliki w rogach obszaru
     * trzeba widziec z daleka. */
    tcfg.quad_decimate = FLORA_CAM_QUAD_DECIMATE;
    if (flora_tags_init(&tcfg) != ESP_OK) {
        ESP_LOGE(TAG, "detektor znacznikow nie wystartowal");
        return;
    }

    /* Niski priorytet: detekcja zajmuje setki milisekund i nie moze
     * zaglodzic zadania odbiorczego lacza. */
    xTaskCreate(detect_task, "tags", 8192, NULL, 4, NULL);
}
