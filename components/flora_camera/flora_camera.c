#include <string.h>

#include "flora_camera.h"

#include "esp_camera.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "flora_camera";

static bool s_ready;

esp_err_t flora_camera_init(const flora_camera_cfg_t *cfg)
{
    ESP_RETURN_ON_FALSE(cfg, ESP_ERR_INVALID_ARG, TAG, "cfg == NULL");
    ESP_RETURN_ON_FALSE(!s_ready, ESP_ERR_INVALID_STATE, TAG, "juz zainicjalizowano");

    /* Rozdzielczosc wybieramy z listy sterownika - dowolna para liczb
     * nie jest obslugiwana. */
    framesize_t fs;
    if (cfg->width <= 160)      fs = FRAMESIZE_QQVGA;   /* 160x120 */
    else if (cfg->width <= 240) fs = FRAMESIZE_HQVGA;   /* 240x176 */
    else if (cfg->width <= 320) fs = FRAMESIZE_QVGA;    /* 320x240 */
    else {
        ESP_LOGW(TAG, "%u px szerokosci to za duzo do wykrywania znacznikow "
                      "na tym ukladzie - ustawiam 320", cfg->width);
        fs = FRAMESIZE_QVGA;
    }

    camera_config_t cc = {
        .pin_pwdn     = cfg->pins.pin_pwdn,
        .pin_reset    = cfg->pins.pin_reset,
        .pin_xclk     = cfg->pins.pin_xclk,
        .pin_sccb_sda = cfg->pins.pin_sccb_sda,
        .pin_sccb_scl = cfg->pins.pin_sccb_scl,
        .pin_d7 = cfg->pins.pin_d7, .pin_d6 = cfg->pins.pin_d6,
        .pin_d5 = cfg->pins.pin_d5, .pin_d4 = cfg->pins.pin_d4,
        .pin_d3 = cfg->pins.pin_d3, .pin_d2 = cfg->pins.pin_d2,
        .pin_d1 = cfg->pins.pin_d1, .pin_d0 = cfg->pins.pin_d0,
        .pin_vsync = cfg->pins.pin_vsync,
        .pin_href  = cfg->pins.pin_href,
        .pin_pclk  = cfg->pins.pin_pclk,

        .xclk_freq_hz = cfg->pins.xclk_freq_hz ? cfg->pins.xclk_freq_hz : 20000000,
        .ledc_timer   = LEDC_TIMER_3,     /* 0 i 1 zajmuje naped i pompka w bryle */
        .ledc_channel = LEDC_CHANNEL_7,

        /* Jeden bajt na piksel, bez kompresji - detektor i tak pracuje
         * na jasnosci, wiec JPEG trzeba by dekodowac z powrotem. */
        .pixel_format = PIXFORMAT_GRAYSCALE,
        .frame_size   = fs,
        .fb_count     = cfg->fb_count ? cfg->fb_count : 2,
        /* Klatki w PSRAM: 320x240 to 76,8 kB, a wewnetrznego SRAM-u
         * potrzebuja struktury robocze detektora, ktore sa wrazliwe na
         * opoznienia. */
        .fb_location  = CAMERA_FB_IN_PSRAM,
        .grab_mode    = CAMERA_GRAB_LATEST,
    };

    esp_err_t ret = esp_camera_init(&cc);
    ESP_RETURN_ON_ERROR(ret, TAG,
        "esp_camera_init - sprawdz pinout DVP, zasilanie modulu i tasme");

    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        /* Znaczniki to czarno-biale wzory o wysokim kontrascie. Lekkie
         * podbicie kontrastu ulatwia progowanie w detektorze. */
        s->set_contrast(s, 1);
        s->set_brightness(s, 0);
        /* Odbicia lustrzane zmieniaja ZNAK azymutu, wiec wylaczamy oba -
         * inaczej robot jechalby w przeciwna strone, niz widzi znacznik. */
        s->set_hmirror(s, 0);
        s->set_vflip(s, 0);
        ESP_LOGI(TAG, "czujnik PID 0x%02X", s->id.PID);
    }

    s_ready = true;
    ESP_LOGI(TAG, "kamera gotowa: %ux%u w odcieniach szarosci, %u bufor(y)",
             cfg->width, cfg->height, cc.fb_count);
    return ESP_OK;
}

esp_err_t flora_camera_deinit(void)
{
    ESP_RETURN_ON_FALSE(s_ready, ESP_ERR_INVALID_STATE, TAG, "nie zainicjalizowano");
    s_ready = false;
    return esp_camera_deinit();
}

esp_err_t flora_camera_grab(flora_camera_frame_t *out, uint32_t timeout_ms)
{
    ESP_RETURN_ON_FALSE(s_ready && out, ESP_ERR_INVALID_STATE, TAG, "nie gotowe");
    (void)timeout_ms;   /* sterownik nie przyjmuje limitu czasu */

    camera_fb_t *fb = esp_camera_fb_get();
    ESP_RETURN_ON_FALSE(fb, ESP_ERR_TIMEOUT, TAG, "brak klatki");

    out->data         = fb->buf;
    out->width        = (uint16_t)fb->width;
    out->height       = (uint16_t)fb->height;
    out->timestamp_us = esp_timer_get_time();
    out->_internal    = fb;
    return ESP_OK;
}

void flora_camera_release(flora_camera_frame_t *frame)
{
    if (frame && frame->_internal) {
        esp_camera_fb_return((camera_fb_t *)frame->_internal);
        frame->_internal = NULL;
        frame->data = NULL;
    }
}

bool flora_camera_is_ready(void) { return s_ready; }
