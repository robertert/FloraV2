/**
 * @file head_obstacles.c
 * @brief Zadanie wizji: klatka RGB565 -> 16 odleglosci -> lacze w dol.
 */
#include <inttypes.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "bringup_cam.h"
#include "head_obstacles.h"

#include "flora_ground.h"
#include "flora_head_board.h"
#include "flora_msgs.h"
#include "flora_store.h"

static const char *TAG = "przeszkody";

/**
 * Takt. Przy 0,2 m/s to 5 cm na klatke przy zasiegu okolo metra, wiec
 * zapasu jest duzo. Tempo ograniczamy swiadomie, nie sprzetowo: ciaglejsza
 * praca nic nie wnosi do bezpieczenstwa, a grzeje modul kamery.
 */
#define PERIOD_MS       250
/** Minimalny oddech dla innych zadan, gdyby przetwarzanie przekroczylo takt. */
#define MIN_SLEEP_MS    20

/** Klucz NVS z wysokoscia obiektywu nad ziemia. Bez niej nie ma centymetrow. */
#define NVS_CAM_H_MM    "cam_h_mm"

static flora_link_handle_t s_down;
static void              *s_arena;

static void obstacles_task(void *arg)
{
    uint32_t frames = 0, errors = 0, sent = 0;
    uint32_t sum_ms = 0, sum_grab = 0, sum_proc = 0, measured = 0;

    while (1) {
        const int64_t t0 = esp_timer_get_time();

        flora_camera_frame_t fr;
        if (bringup_cam_frame_begin(&fr) != ESP_OK) {
            errors++;
            vTaskDelay(pdMS_TO_TICKS(PERIOD_MS));
            continue;
        }
        /* Czas pobrania i czas liczenia MIERZYMY OSOBNO. Bez tego podzialu
         * nie da sie odpowiedziec na pytanie, czy takt ogranicza algorytm,
         * czy sam czujnik: przy obnizonym dla chlodzenia zegarze kamera
         * oddaje klatke co kilkaset ms, a grab() po prostu na nia czeka. */
        const int64_t t_grabbed = esp_timer_get_time();

        flora_ground_result_t res;
        const int r = flora_ground_process(fr.data, &res);
        const int64_t t_done = esp_timer_get_time();
        const uint32_t ts_us = (uint32_t)fr.timestamp_us;
        /* Bufor oddajemy NATYCHMIAST po przetworzeniu: trzymanie go blokuje
         * serwer podgladu i timer usypiania kamery. */
        bringup_cam_frame_end(&fr);

        if (r != 0) {
            errors++;
            continue;
        }
        frames++;

        sum_ms   += (uint32_t)((t_done - t0) / 1000);
        sum_grab += (uint32_t)((t_grabbed - t0) / 1000);
        sum_proc += (uint32_t)((t_done - t_grabbed) / 1000);
        measured++;

        if (s_down) {
            flora_obstacles_t msg = {
                /* Znacznik czasu z chwili POBRANIA klatki, nie wyslania -
                 * ta sama konwencja co w ATTITUDE. */
                .timestamp_us = ts_us,
                .fov_cdeg     = 16000,     /* wachlarz 160 stopni */
                .valid        = res.valid,
            };
            memcpy(msg.sectors, res.sectors, FLORA_OBSTACLE_SECTORS);
            /* Bez ACK, jak ATTITUDE: nastepna klatka i tak uniewaznia
             * poprzednia, a send_reliable blokuje do 400 ms i zdemolowalby
             * takt tego zadania. */
            if (flora_link_send(s_down, FLORA_MSG_OBSTACLES, &msg, sizeof msg) == ESP_OK) {
                sent++;
            }
        }

        /* Co 40 klatek, czyli co 10 s: tempo, obciazenie i najblizsza
         * przeszkoda. Bez tego nie wiadomo, czy zadanie zyje. */
        if (frames % 40 == 0) {
            /* Szukamy po first_ring, NIE po sectors[]. Bez wysokosci kamery
             * w NVS kazda wykryta przeszkoda dostaje w sectors[] zero, czyli
             * "nie wiem" - i tak ma byc, bo odleglosci naprawde nie znamy.
             * Ale log szukajacy minimum w centymetrach pomijal te zera i
             * drukowal "czysto" nad wykryta przeszkoda. Pierscien jest
             * wypelniony zawsze, wiec to on jest tu wlasciwa diagnostyka. */
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
            /* Zapas stosu w kazdym podsumowaniu. Pierwsza wersja tego
             * zadania trzymala histogramy pierscieni na stosie i wywalala
             * sie przez przepelnienie zaraz po starcie - takie rzeczy maja
             * byc widoczne jako malejaca liczba, a nie jako panic. */
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
                                  "(cm nieznane - brak wysokosci w NVS)",
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

        /* Spimy RESZTE okresu, a nie vTaskDelayUntil: przy przekroczeniu
         * budzetu tamto zaczyna isc klatka za klatka bez oddechu, a wizja
         * ma byc tym, co ustepuje laczu i fuzji, nie odwrotnie. */
        const uint32_t elapsed = (uint32_t)((esp_timer_get_time() - t0) / 1000);
        const uint32_t sleep_ms = (elapsed < PERIOD_MS - MIN_SLEEP_MS)
                                ? (PERIOD_MS - elapsed) : MIN_SLEEP_MS;
        vTaskDelay(pdMS_TO_TICKS(sleep_ms));
    }
}

esp_err_t head_obstacles_start(flora_link_handle_t down)
{
    if (!bringup_cam_is_raw_mode()) {
        ESP_LOGW(TAG, "kamera nie jest w trybie surowym - wizja nie startuje");
        ESP_LOGW(TAG, "przelacz przez http://<ip>/mode?v=raw");
        return ESP_ERR_INVALID_STATE;
    }

    flora_ground_cfg_t cfg = FLORA_GROUND_DEFAULT_CFG();

    /* Wysokosc obiektywu nad ziemia z NVS. Bez niej detektor dziala, ale
     * zamiast centymetrow raportuje "nie wiem", bo zamiana pierscienia na
     * odleglosc jest wprost proporcjonalna do tej liczby. Zmierz miarka i
     * zapisz, a nie zgaduj - blad wysokosci przeklada sie jeden do jednego
     * na blad odleglosci. */
    flora_store_t store = NULL;
    if (flora_store_open(FLORA_HEAD_NVS_NS, &store) == ESP_OK) {
        int32_t h = 0;
        flora_store_get_i32(store, NVS_CAM_H_MM, &h, 0);
        flora_store_close(store);
        cfg.cam_height_mm = (float)h;
    }
    if (cfg.cam_height_mm <= 0.0f) {
        ESP_LOGW(TAG, "brak wysokosci kamery w NVS (klucz \"%s\") - sektory beda",
                 NVS_CAM_H_MM);
        ESP_LOGW(TAG, "raportowane jako nieznane zamiast w centymetrach");
    }

    const size_t need = flora_ground_arena_size(&cfg);
    /* PSRAM, nie wewnetrzny SRAM: same tablice przegladowe to ponad 200 kB,
     * a wewnetrzna pamiec jest potrzebna stosom i stosowi sieciowemu. */
    s_arena = heap_caps_malloc(need, MALLOC_CAP_SPIRAM);
    if (!s_arena) {
        ESP_LOGE(TAG, "brak %u kB w PSRAM na tablice", (unsigned)(need / 1024));
        return ESP_ERR_NO_MEM;
    }
    if (flora_ground_init(&cfg, s_arena, need) != 0) {
        ESP_LOGE(TAG, "konfiguracja detektora bez sensu");
        heap_caps_free(s_arena);
        s_arena = NULL;
        return ESP_ERR_INVALID_ARG;
    }

    s_down = down;
    ESP_LOGI(TAG, "detektor gotowy: %u kB w PSRAM, %d wycinkow x %d pierscieni, "
                  "wysokosc %.0f mm", (unsigned)(need / 1024),
             FLORA_GROUND_SECTORS, cfg.rings, cfg.cam_height_mm);

    /* Priorytet 4, scisle ponizej fuzji i zadan odbiorczych lacza (10) oraz
     * czujnikow wolnych (5) - ten sam wybor i to samo uzasadnienie co przy
     * detekcji znacznikow w tagcam. Rdzen 1, zeby nie konkurowac z WiFi i
     * laczami, ktore siedza na rdzeniu 0. */
    if (xTaskCreatePinnedToCore(obstacles_task, "przeszkody", 4096, NULL, 4,
                                NULL, 1) != pdPASS) {
        ESP_LOGE(TAG, "nie udalo sie utworzyc zadania");
        return ESP_FAIL;
    }
    return ESP_OK;
}
