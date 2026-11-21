/**
 * Doniczka-robot, modul kamery przeszkod (trzeci modul, w maszcie).
 *
 * Zadanie: kamera z obiektywem 160 stopni patrzy w dol, detektor
 * flora_ground szuka miejsc, w ktorych przestaje byc podloze, i wysyla
 * 16 sektorow z odlegloscia (FLORA_MSG_OBSTACLES). Weto jazdy liczy
 * straznik w bryle.
 *
 * Od 21.11.2026. Wczesniej ten modul szukal znacznikow AprilTag, a
 * przeszkody liczyla glowica - role kamer zostaly zamienione (znaczniki sa
 * teraz w node/main/head_tags.c). Algorytm jest ten sam, przeniesiony
 * z node/main/head_obstacles.c bez zmian.
 *
 * W maszcie jest tylko jeden kabel w dol, wiec modul nie ma wlasnego
 * polaczenia z bryla - gada z glowica, a ta przekazuje ramki dalej.
 *
 * Bez WiFi i bez podgladu obrazu: do wycelowania kamery i zbierania klatek
 * trzeba chwilowo wpiac ja z powrotem w glowice (bringup_cam.c).
 */
#include <inttypes.h>
#include <stdbool.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "flora_cam_board.h"
#include "flora_camera.h"
#include "flora_ground.h"
#include "flora_link.h"
#include "flora_msgs.h"

static const char *TAG = "przeszkody";

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

/* Liczniki dla zadania statusu. Pisze je tylko obstacles_task, czyta tylko
 * status_task - 32-bitowe zapisy na S3 sa niepodzielne, mutex zbedny. */
static volatile uint32_t s_frames_total;
static volatile uint32_t s_errors_total;
static volatile bool     s_camera_ok;

/** Co ile wysylamy CAM_STATUS. Musi byc wyraznie krotsze niz peer_timeout
 *  lacza (1 s), bo flora_link nie ma wlasnego heartbeatu. */
#define STATUS_PERIOD_MS 500

static void status_task(void *arg)
{
    uint32_t last_frames = 0;
    while (1) {
        const uint32_t frames = s_frames_total;
        flora_cam_status_t st = {
            .uptime_ms        = (uint32_t)(esp_timer_get_time() / 1000),
            .frames_processed = (uint16_t)(frames - last_frames),
            .error_count      = (uint16_t)s_errors_total,
            .camera_ok        = s_camera_ok ? 1 : 0,
        };
        last_frames = frames;
        flora_link_send(s_link, FLORA_MSG_CAM_STATUS, &st, sizeof st);
        vTaskDelay(pdMS_TO_TICKS(STATUS_PERIOD_MS));
    }
}

/**
 * Takt. Przy 0,2 m/s to 5 cm na klatke przy zasiegu okolo metra. Tempo
 * ograniczamy swiadomie: szybciej nic nie wnosi, a grzeje modul kamery.
 */
#define PERIOD_MS       250
#define MIN_SLEEP_MS    20

static void obstacles_task(void *arg)
{
    uint32_t frames = 0, errors = 0, sent = 0;
    uint32_t sum_ms = 0, sum_grab = 0, sum_proc = 0, measured = 0;

    while (1) {
        const int64_t t0 = esp_timer_get_time();

        flora_camera_frame_t fr;
        if (flora_camera_grab(&fr, 1000) != ESP_OK) {
            errors++;
            s_errors_total = errors;
            vTaskDelay(pdMS_TO_TICKS(PERIOD_MS));
            continue;
        }
        /* Czas pobrania i liczenia osobno: przy obnizonym zegarze kamera
         * oddaje klatke co kilkaset ms i to ona ogranicza takt. */
        const int64_t t_grabbed = esp_timer_get_time();

        flora_ground_result_t res;
        const int r = flora_ground_process(fr.data, &res);
        const int64_t t_done = esp_timer_get_time();
        const uint32_t ts_us = (uint32_t)fr.timestamp_us;
        flora_camera_release(&fr);

        if (r != 0) {
            errors++;
            s_errors_total = errors;
            continue;
        }
        frames++;
        s_frames_total = frames;

        sum_ms   += (uint32_t)((t_done - t0) / 1000);
        sum_grab += (uint32_t)((t_grabbed - t0) / 1000);
        sum_proc += (uint32_t)((t_done - t_grabbed) / 1000);
        measured++;

        flora_obstacles_t msg = {
            /* Znacznik czasu z chwili POBRANIA klatki, jak w ATTITUDE. */
            .timestamp_us = ts_us,
            .fov_cdeg     = 16000,     /* wachlarz 160 stopni */
            .valid        = res.valid,
        };
        memcpy(msg.sectors, res.sectors, FLORA_OBSTACLE_SECTORS);
        /* Bez ACK: nastepna klatka i tak uniewaznia poprzednia. */
        if (flora_link_send(s_link, FLORA_MSG_OBSTACLES, &msg, sizeof msg) == ESP_OK) {
            sent++;
        }

        /* Co 40 klatek, czyli co 10 s: tempo i najblizsza przeszkoda. */
        if (frames % 40 == 0) {
            /* Szukamy po first_ring, NIE po sectors[]: bez wysokosci kamery
             * kazda przeszkoda ma w sectors[] zero ("nie wiem"), a log
             * szukajacy minimum w cm drukowalby "czysto" nad przeszkoda. */
            uint8_t min_ring = 255;
            int min_sec = -1, blind = 0;
            for (int i = 0; i < FLORA_OBSTACLE_SECTORS; i++) {
                if (res.first_ring[i] < min_ring) {
                    min_ring = res.first_ring[i];
                    min_sec = i;
                }
                if (res.first_ring[i] == 255 &&
                    res.sectors[i] == FLORA_OBSTACLE_UNKNOWN) {
                    blind++;
                }
            }
            const uint32_t avg = measured ? sum_ms / measured : 0;
            const uint32_t avg_grab = measured ? sum_grab / measured : 0;
            const uint32_t avg_proc = measured ? sum_proc / measured : 0;
            ESP_LOGI(TAG, "%" PRIu32 " klatek, srednio %" PRIu32 " ms, wyslanych %"
                          PRIu32 ", bledow %" PRIu32 ", zapas stosu %u B",
                     frames, avg, sent, errors,
                     (unsigned)uxTaskGetStackHighWaterMark(NULL));
            ESP_LOGI(TAG, "  z tego czekanie na kamere %" PRIu32 " ms, liczenie %"
                          PRIu32 " ms", avg_grab, avg_proc);
            if (min_sec >= 0 && min_ring != 255) {
                const uint8_t cm = res.sectors[min_sec];
                if (cm == FLORA_OBSTACLE_UNKNOWN) {
                    ESP_LOGI(TAG, "  najblizej: wycinek %d, pierscien %u "
                                  "(cm nieznane - brak FLORA_CAM_HEIGHT_MM)",
                             min_sec, min_ring);
                } else {
                    ESP_LOGI(TAG, "  najblizej: wycinek %d, pierscien %u, %u cm",
                             min_sec, min_ring, cm);
                }
            } else {
                ESP_LOGI(TAG, "  czysto (slepych wycinkow %d/%d)",
                         blind, FLORA_OBSTACLE_SECTORS);
            }
            sum_ms = sum_grab = sum_proc = 0;
            measured = 0;
        }

        /* Spimy RESZTE okresu: przy przekroczeniu budzetu zostaje minimalny
         * oddech dla zadan lacza. */
        const uint32_t elapsed = (uint32_t)((esp_timer_get_time() - t0) / 1000);
        const uint32_t sleep_ms = (elapsed < PERIOD_MS - MIN_SLEEP_MS)
                                ? (PERIOD_MS - elapsed) : MIN_SLEEP_MS;
        vTaskDelay(pdMS_TO_TICKS(sleep_ms));
    }
}

static esp_err_t camera_up(void)
{
    flora_camera_cfg_t cfg = FLORA_CAMERA_DEFAULT_CFG();
    cfg.pins = (flora_camera_pins_t)FLORA_CAM_PINS();
    cfg.pins.xclk_freq_hz = FLORA_CAM_XCLK_HZ;
    cfg.format = FLORA_CAMERA_RGB565;     /* flora_ground odrzuca cien po barwie */
    cfg.width  = 320;
    cfg.height = 240;

    esp_err_t r = flora_camera_init(&cfg);
    /* Obnizony zegar to wybor termiczny - gdyby czujnik przy nim nie ruszyl,
     * lepiej ciepla kamera niz zadna. */
    if (r != ESP_OK && cfg.pins.xclk_freq_hz != 20000000) {
        ESP_LOGW(TAG, "czujnik nie ruszyl przy %d MHz, probuje 20 MHz",
                 cfg.pins.xclk_freq_hz / 1000000);
        cfg.pins.xclk_freq_hz = 20000000;
        r = flora_camera_init(&cfg);
    }
    return r;
}

void app_main(void)
{
    ESP_LOGI(TAG, "doniczka-robot, modul kamery przeszkod");

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
    xTaskCreate(status_task, "status", 3072, NULL, 5, NULL);

    /* Kamera i detektor. Brak ktoregokolwiek nie blokuje lacza - glowica
     * widzi modul i dostaje camera_ok = 0 zamiast ciszy. */
    if (camera_up() != ESP_OK) {
        ESP_LOGE(TAG, "kamera nie wystartowala - sprawdz tasme (styki do dolu) "
                      "i wlaczony PSRAM");
        return;
    }

    flora_ground_cfg_t gcfg = FLORA_GROUND_DEFAULT_CFG();
    gcfg.cam_height_mm = (float)FLORA_CAM_HEIGHT_MM;
    if (gcfg.cam_height_mm <= 0.0f) {
        ESP_LOGW(TAG, "WYSOKOSC KAMERY NIE JEST ZNANA - sektory pojada jako");
        ESP_LOGW(TAG, "nieznane zamiast w centymetrach, czyli bryla ich nie uzyje.");
        ESP_LOGW(TAG, "Zmierz miarka i wpisz w FLORA_CAM_HEIGHT_MM (flora_cam_board.h).");
    }

    const size_t need = flora_ground_arena_size(&gcfg);
    /* PSRAM: same tablice przegladowe to ponad 200 kB. */
    void *arena = heap_caps_malloc(need, MALLOC_CAP_SPIRAM);
    if (!arena) {
        ESP_LOGE(TAG, "brak %u kB w PSRAM na tablice", (unsigned)(need / 1024));
        return;
    }
    if (flora_ground_init(&gcfg, arena, need) != 0) {
        ESP_LOGE(TAG, "konfiguracja detektora bez sensu");
        heap_caps_free(arena);
        return;
    }
    ESP_LOGI(TAG, "detektor gotowy: %u kB w PSRAM, %d wycinkow x %d pierscieni, "
                  "wysokosc %.0f mm", (unsigned)(need / 1024),
             FLORA_GROUND_SECTORS, gcfg.rings, gcfg.cam_height_mm);

    s_camera_ok = true;
    /* Priorytet 4, ponizej zadan odbiorczych lacza; rdzen 1. */
    xTaskCreatePinnedToCore(obstacles_task, "przeszkody", 4096, NULL, 4, NULL, 1);
}
