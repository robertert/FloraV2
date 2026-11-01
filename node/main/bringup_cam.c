/**
 * @file bringup_cam.c
 * @brief Podglad obrazu z kamery przez WiFi: serwer MJPEG.
 *
 * DLACZEGO JPEG, A NIE ODCIENIE SZAROSCI: czujnik OV2640 kompresuje klatke
 * sprzetowo, wiec po sieci leci kilkadziesiat kilobajtow zamiast 300 kB
 * surowych pikseli VGA, a przegladarka wyswietla je bez zadnej konwersji.
 * Gdybysmy nadawali szarosc z flora_camera, trzeba by ja na ESP pakowac
 * w BMP albo kodowac do JPEG programowo - niepotrzebna praca w tescie,
 * ktorego cala rola jest pokazac obraz.
 *
 * DLACZEGO MJPEG, A NIE STRONA Z JAVASCRIPTEM: multipart/x-mixed-replace
 * to strumien klatek w jednej odpowiedzi HTTP. Zwykly <img src="/stream">
 * odtwarza go sam, bez linii skryptu, i dziala tak samo w Safari i Chrome.
 *
 * DLACZEGO VGA: podglad ma pokazac OSTROSC i KADR obiektywu 160 stopni,
 * a przy 320x240 nie widac, czy obraz jest ostry, czy tylko maly. VGA
 * (640x480) to okolo 8-12 klatek na sekunde przez WiFi - do oceny montazu
 * z nadmiarem, a klatka nadal miesci sie w jednym buforze PSRAM.
 */
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_camera.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "bringup_cam.h"
#include "flora_head_board.h"
/* Tylko po dane dostepowe WiFi i klucze NVS - glowica trzyma je pod tymi
 * samymi nazwami co bryla, zeby nie bylo dwoch miejsc do poprawiania. */
#include "flora_board.h"
#include "flora_store.h"
#include "flora_wifi.h"

static const char *TAG = "podglad";

/** Rozmiar klatki. SVGA (800x600) tez dziala, ale tempo spada o polowe. */
#define PREVIEW_FRAMESIZE   FRAMESIZE_VGA
/** Jakosc JPEG: 10-12 to czysty obraz, 20 to wyraznie widoczne bloki. */
#define PREVIEW_JPEG_Q      12

static httpd_handle_t s_server;

/* ---------------------------------------------------------------- WiFi -- */

static void on_wifi_state(const flora_wifi_info_t *info, void *ctx)
{
    switch (info->state) {
        case FLORA_WIFI_CONNECTED:
            ESP_LOGI(TAG, "  WiFi: adres " IPSTR, IP2STR(&info->ip));
            break;
        case FLORA_WIFI_CONNECTING:
            ESP_LOGI(TAG, "  WiFi: proba %" PRIu32, info->attempt);
            break;
        case FLORA_WIFI_DISCONNECTED:
            ESP_LOGW(TAG, "  WiFi: rozlaczone (%s), nastepna proba za %" PRIu32 " ms",
                     flora_wifi_reason_str(info->reason), info->next_retry_ms);
            break;
        case FLORA_WIFI_FAILED:
            ESP_LOGE(TAG, "  WiFi: poddaje sie");
            break;
    }
}

static esp_err_t wifi_up(void)
{
    char ssid[33], pass[65];
    flora_store_t store = NULL;

    strncpy(ssid, FLORA_WIFI_SSID_DEFAULT, sizeof(ssid) - 1); ssid[sizeof(ssid) - 1] = 0;
    strncpy(pass, FLORA_WIFI_PASS_DEFAULT, sizeof(pass) - 1); pass[sizeof(pass) - 1] = 0;

    if (flora_store_open(FLORA_NVS_NS_NET, &store) == ESP_OK) {
        flora_store_get_str(store, FLORA_NVS_WIFI_SSID, ssid, sizeof(ssid),
                            FLORA_WIFI_SSID_DEFAULT);
        flora_store_get_str(store, FLORA_NVS_WIFI_PASS, pass, sizeof(pass),
                            FLORA_WIFI_PASS_DEFAULT);
        flora_store_close(store);
    }
    ESP_LOGI(TAG, "  siec \"%s\"", ssid);

    flora_wifi_cfg_t cfg = FLORA_WIFI_DEFAULT_CFG();
    strncpy(cfg.ssid, ssid, sizeof(cfg.ssid) - 1);
    strncpy(cfg.password, pass, sizeof(cfg.password) - 1);
    cfg.on_state = on_wifi_state;

    esp_err_t r = flora_wifi_init(&cfg);
    /* Glowica moze miec WiFi podniesione przez inny test - to nie blad. */
    if (r != ESP_OK && r != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "  nie udalo sie podniesc WiFi: %s", esp_err_to_name(r));
        return r;
    }

    ESP_LOGI(TAG, "  czekam na adres (do 20 s)...");
    r = flora_wifi_wait_connected(20000);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "  brak polaczenia: %s", esp_err_to_name(r));
        ESP_LOGE(TAG, "  ponawianie dziala dalej w tle - podglad wstanie dopiero po restarcie");
    }
    return r;
}

/* -------------------------------------------------------------- kamera -- */

static esp_err_t camera_up(void)
{
    camera_config_t cc = {
        .pin_pwdn     = FLORA_HEAD_DVP_PWDN,
        .pin_reset    = FLORA_HEAD_DVP_RESET,
        .pin_xclk     = FLORA_HEAD_DVP_XCLK,
        .pin_sccb_sda = FLORA_HEAD_DVP_SDA,
        .pin_sccb_scl = FLORA_HEAD_DVP_SCL,
        .pin_d7 = FLORA_HEAD_DVP_D7, .pin_d6 = FLORA_HEAD_DVP_D6,
        .pin_d5 = FLORA_HEAD_DVP_D5, .pin_d4 = FLORA_HEAD_DVP_D4,
        .pin_d3 = FLORA_HEAD_DVP_D3, .pin_d2 = FLORA_HEAD_DVP_D2,
        .pin_d1 = FLORA_HEAD_DVP_D1, .pin_d0 = FLORA_HEAD_DVP_D0,
        .pin_vsync = FLORA_HEAD_DVP_VSYNC,
        .pin_href  = FLORA_HEAD_DVP_HREF,
        .pin_pclk  = FLORA_HEAD_DVP_PCLK,

        .xclk_freq_hz = 20000000,
        /* Timery 0 i 1 zajmuje naped i pompka w bryle. Tu sa wolne, ale
         * trzymamy te same numery co flora_camera, zeby zamiana podgladu
         * na detekcje nie zmieniala niczego poza formatem. */
        .ledc_timer   = LEDC_TIMER_3,
        .ledc_channel = LEDC_CHANNEL_7,

        .pixel_format = PIXFORMAT_JPEG,
        .frame_size   = PREVIEW_FRAMESIZE,
        .jpeg_quality = PREVIEW_JPEG_Q,
        /* Dwa bufory: jeden leci do przegladarki, w drugi kamera juz lapie.
         * Przy jednym tempo spada do czasu wyslania klatki po WiFi. */
        .fb_count     = 2,
        .fb_location  = CAMERA_FB_IN_PSRAM,
        .grab_mode    = CAMERA_GRAB_LATEST,
    };

    esp_err_t r = esp_camera_init(&cc);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "  esp_camera_init: %s", esp_err_to_name(r));
        ESP_LOGE(TAG, "  sprawdz w tej kolejnosci: tasma w gniezdzie (styki do dolu),");
        ESP_LOGE(TAG, "  pinout DVP w flora_head_board.h, wlaczony PSRAM w sdkconfig");
        return r;
    }

    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        /* Podglad sluzy tez do ustalenia orientacji obrazu - jesli widok
         * jest do gory nogami albo odbity, popraw TUTAJ i te same wartosci
         * przenies do flora_camera. Lustrzane odbicie zmienia ZNAK azymutu
         * znacznika, wiec musi byc zgodne w obu trybach. */
        s->set_hmirror(s, 0);
        s->set_vflip(s, 0);
        ESP_LOGI(TAG, "  czujnik PID 0x%02X", s->id.PID);
    }
    return ESP_OK;
}

/* ---------------------------------------------------------- serwer HTTP -- */

#define BOUNDARY "flora_klatka"

static const char *STREAM_TYPE = "multipart/x-mixed-replace;boundary=" BOUNDARY;
static const char *PART_HDR    = "\r\n--" BOUNDARY "\r\n"
                                 "Content-Type: image/jpeg\r\n"
                                 "Content-Length: %u\r\n\r\n";

static const char PAGE[] =
    "<!doctype html><meta charset=utf-8><meta name=viewport "
    "content='width=device-width,initial-scale=1'>"
    "<title>Flora - podglad kamery</title>"
    "<style>body{margin:0;background:#111;color:#ddd;font:14px system-ui;"
    "text-align:center}img{max-width:100%;image-rendering:pixelated}"
    "p{margin:8px}a{color:#8cf}</style>"
    "<img src='/stream' alt='strumien z kamery'>"
    "<p>obiektyw 160&deg; &middot; <a href='/jpg'>zapisz klatke</a></p>";

static esp_err_t page_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, PAGE, sizeof(PAGE) - 1);
}

/** Jedna klatka. Do "curl -o kadr.jpg http://<ip>/jpg". */
static esp_err_t jpg_handler(httpd_req_t *req)
{
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "brak klatki");
        return ESP_FAIL;
    }
    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=flora.jpg");
    esp_err_t r = httpd_resp_send(req, (const char *)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    return r;
}

static esp_err_t stream_handler(httpd_req_t *req)
{
    esp_err_t r = httpd_resp_set_type(req, STREAM_TYPE);
    if (r != ESP_OK) {
        return r;
    }
    ESP_LOGI(TAG, "przegladarka podlaczona do strumienia");

    uint32_t frames = 0, bytes = 0;
    int64_t  t0 = esp_timer_get_time();

    while (1) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "brak klatki z kamery - koncze strumien");
            break;
        }

        /* 96 bajtow: naglowek czesci ma 75 znakow przy szescioznakowej
         * dlugosci klatki, reszta to zapas na dluzsze liczby. */
        char hdr[96];
        int n = snprintf(hdr, sizeof hdr, PART_HDR, (unsigned)fb->len);

        /* Przegladarka zamyka strumien zwyczajnie przez rozlaczenie, wiec
         * blad zapisu to NORMALNE zakonczenie, nie awaria. */
        r = httpd_resp_send_chunk(req, hdr, n);
        if (r == ESP_OK) {
            r = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
        }
        bytes += fb->len;
        esp_camera_fb_return(fb);
        if (r != ESP_OK) {
            break;
        }

        /* Co 100 klatek tempo i sredni rozmiar klatki: jedno i drugie mowi,
         * czy waskim gardlem jest WiFi, czy sama kamera. */
        if (++frames % 100 == 0) {
            int64_t dt = esp_timer_get_time() - t0;
            float   s  = (float)dt / 1000000.0f;
            ESP_LOGI(TAG, "%" PRIu32 " klatek, %.1f kl/s, srednio %" PRIu32 " kB/klatke",
                     frames, s > 0 ? 100.0f / s : 0.0f, bytes / 100 / 1024);
            bytes = 0;
            t0 = esp_timer_get_time();
        }
    }

    httpd_resp_send_chunk(req, NULL, 0);
    ESP_LOGI(TAG, "strumien zakonczony po %" PRIu32 " klatkach", frames);
    return ESP_OK;
}

static esp_err_t server_up(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    /* Domyslne 4 kB nie wystarcza: w tym samym zadaniu siedzi obsluga
     * kamery i sklejanie czesci multipart. */
    cfg.stack_size       = 8192;
    /* Zamknieta karta przegladarki zostawia gniazdo zajete do timeoutu.
     * Bez tego po kilku odswiezeniach serwer przestaje przyjmowac. */
    cfg.lru_purge_enable = true;

    esp_err_t r = httpd_start(&s_server, &cfg);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "  httpd_start: %s", esp_err_to_name(r));
        return r;
    }

    const httpd_uri_t uris[] = {
        { .uri = "/",       .method = HTTP_GET, .handler = page_handler   },
        { .uri = "/stream", .method = HTTP_GET, .handler = stream_handler },
        { .uri = "/jpg",    .method = HTTP_GET, .handler = jpg_handler    },
    };
    for (size_t i = 0; i < sizeof uris / sizeof uris[0]; i++) {
        httpd_register_uri_handler(s_server, &uris[i]);
    }
    return ESP_OK;
}

/* ----------------------------------------------------------------- test -- */

esp_err_t bringup_cam_preview(void)
{
    ESP_LOGI(TAG, "--- podglad kamery ---");

    esp_err_t r = wifi_up();
    if (r != ESP_OK) {
        return r;
    }
    if ((r = camera_up()) != ESP_OK) {
        return r;
    }
    if ((r = server_up()) != ESP_OK) {
        esp_camera_deinit();
        return r;
    }

    flora_wifi_info_t info;
    flora_wifi_get_info(&info);
    ESP_LOGI(TAG, "  obraz na komputerze: http://" IPSTR "/", IP2STR(&info.ip));
    ESP_LOGI(TAG, "  jedna klatka do pliku: curl -o kadr.jpg http://" IPSTR "/jpg",
             IP2STR(&info.ip));
    return ESP_OK;
}
