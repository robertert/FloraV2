/**
 * @file head_tags.c
 * @brief Zadanie znacznikow: klatka RGB565 -> szarosc -> AprilTag -> lacze w dol.
 *
 * Przeniesione z tagcam/main/main.c przy zamianie rol kamer (21.11.2026).
 *
 * WYDAJNOSC: przy 320x240 i pelnej rozdzielczosci (FLORA_HEAD_TAG_QUAD_DECIMATE
 * 1) detektor liczy setki milisekund do kilku sekund na klatke. Zadanie ma
 * niski priorytet i siedzi na rdzeniu 1, wiec fuzja kursu (priorytet 10)
 * i zadania lacza zawsze je wywlaszczaja.
 */
#include <inttypes.h>
#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "esp_log.h"

#include "bringup_cam.h"
#include "head_tags.h"

#include "flora_head_board.h"
#include "flora_msgs.h"
#include "flora_tags.h"

static const char *TAG = "znaczniki";

/** Ile klatek pod rzad bez znacznika, zanim napiszemy o tym raz. */
#define QUIET_AFTER_EMPTY 10
/** Oddech miedzy klatkami: kamera ma czas, a lacza i fuzja procesor. */
#define GAP_MS            50

static flora_link_handle_t s_down;
static uint8_t            *s_gray;       /**< 320x240, PSRAM */

/**
 * RGB565 big-endian (jak dekoduje esp32-camera) -> jasnosc 0..255.
 * Przy FLORA_HEAD_TAG_MIRROR wiersze sa odwracane lewo-prawo, bo kamera
 * glowicy daje obraz lustrzany, a odbitego znacznika nie da sie odczytac.
 * Srodek i azymut znacznika licza sie wtedy w obrazie prostym.
 */
static void to_gray(const uint8_t *rgb, uint8_t *gray, uint16_t w, uint16_t h)
{
    for (uint16_t y = 0; y < h; y++) {
        const uint8_t *src = rgb + (size_t)y * w * 2;
        uint8_t *dst = gray + (size_t)y * w;
        for (uint16_t x = 0; x < w; x++) {
            const uint8_t hb = src[2 * x], lb = src[2 * x + 1];
            const uint32_t r = hb & 0xF8;
            const uint32_t g = ((hb & 0x07) << 5) | ((lb & 0xE0) >> 3);
            const uint32_t b = (lb & 0x1F) << 3;
            const uint8_t v = (uint8_t)((77 * r + 150 * g + 29 * b) >> 8);
            dst[FLORA_HEAD_TAG_MIRROR ? (w - 1 - x) : x] = v;
        }
    }
}

static void tags_task(void *arg)
{
    uint32_t frames = 0, empty_streak = 0, errors = 0;
    uint32_t sum_ms = 0, measured = 0;

    while (1) {
        flora_camera_frame_t fr;
        if (bringup_cam_frame_begin(&fr) != ESP_OK) {
            errors++;
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }
        const uint16_t w = fr.width, h = fr.height;
        const uint32_t ts_us = (uint32_t)fr.timestamp_us;
        const bool ok = fr.format == FLORA_CAMERA_RGB565 && (size_t)w * h <= 320u * 240u;
        if (ok) {
            to_gray(fr.data, s_gray, w, h);
        }
        /* Kamere oddajemy PRZED detekcja: ta trwa dlugo, a blokada kamery
         * trzyma serwer podgladu i timer usypiania. */
        bringup_cam_frame_end(&fr);
        if (!ok) {
            errors++;
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        flora_tags_result_t res;
        if (flora_tags_detect(s_gray, w, h, &res) != ESP_OK) {
            errors++;
            continue;
        }
        frames++;
        sum_ms += res.process_ms;
        measured++;

        flora_tags_t msg = {
            .timestamp_us = ts_us,
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
        if (s_down) {
            flora_link_send(s_down, FLORA_MSG_TAGS, &msg, sizeof msg);
        }

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
            ESP_LOGI(TAG, "brak znacznikow w polu widzenia");
        }

        /* Co 20 klatek podsumowanie: realne tempo decyduje, czy to nadaje
         * sie do nawigacji. */
        if (frames % 20 == 0) {
            const uint32_t avg = measured ? sum_ms / measured : 0;
            ESP_LOGI(TAG, "%" PRIu32 " klatek, srednio %" PRIu32 " ms na klatke "
                          "(%.1f kl/s), bledow %" PRIu32 ", zapas stosu %u B",
                     frames, avg, avg ? 1000.0f / (float)avg : 0.0f, errors,
                     (unsigned)uxTaskGetStackHighWaterMark(NULL));
            sum_ms = 0;
            measured = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(GAP_MS));
    }
}

esp_err_t head_tags_start(flora_link_handle_t down)
{
    if (!bringup_cam_is_raw_mode()) {
        ESP_LOGW(TAG, "kamera nie jest w trybie surowym - znaczniki nie startuja");
        ESP_LOGW(TAG, "przelacz przez http://<ip>/mode?v=raw");
        return ESP_ERR_INVALID_STATE;
    }
    s_gray = heap_caps_malloc(320 * 240, MALLOC_CAP_SPIRAM);
    if (!s_gray) {
        ESP_LOGE(TAG, "brak PSRAM na bufor szarosci");
        return ESP_ERR_NO_MEM;
    }

    flora_tags_cfg_t cfg = FLORA_TAGS_DEFAULT_CFG();
    cfg.tag_size_mm   = FLORA_HEAD_TAG_SIZE_MM;
    cfg.focal_px      = FLORA_HEAD_TAG_FOCAL_PX;
    /* Pelna rozdzielczosc: zasieg 11 m zamiast 5,6, kosztem tempa. */
    cfg.quad_decimate = FLORA_HEAD_TAG_QUAD_DECIMATE;
    if (flora_tags_init(&cfg) != ESP_OK) {
        ESP_LOGE(TAG, "detektor znacznikow nie wystartowal");
        heap_caps_free(s_gray);
        s_gray = NULL;
        return ESP_FAIL;
    }

    s_down = down;
    ESP_LOGI(TAG, "detektor gotowy: znacznik %d mm, ogniskowa %.0f px, decymacja %.1f",
             FLORA_HEAD_TAG_SIZE_MM, FLORA_HEAD_TAG_FOCAL_PX,
             (double)FLORA_HEAD_TAG_QUAD_DECIMATE);

    /* Priorytet 4, ponizej fuzji (10), lacza i czujnikow wolnych (5);
     * rdzen 1, z dala od WiFi. Stos jak w tagcam - AprilTag alokuje na
     * stercie, ale ma glebokie wywolania. */
    if (xTaskCreatePinnedToCore(tags_task, "znaczniki", 8192, NULL, 4, NULL, 1) != pdPASS) {
        ESP_LOGE(TAG, "nie udalo sie utworzyc zadania");
        return ESP_FAIL;
    }
    return ESP_OK;
}
