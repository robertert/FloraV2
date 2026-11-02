#include <string.h>

#include "flora_camera.h"

#include "esp_camera.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "flora_camera";

/**
 * Ile pierwszych klatek po starcie wyrzucamy, zanim oddamy obraz.
 *
 * Dziesiec, nie trzy. Zmierzone na zbiorze z panelami: przy trzech
 * pierwsza oddana klatka kazdej sesji nadal miala 6-13% pikseli poza
 * zakresem jasnosci, podczas gdy kolejne mialy 0,5-1,7%, i jako jedyna
 * dawala falszywe trafienia. Automatyka ekspozycji potrzebuje kilkunastu
 * klatek na zbiegniecie, a przy 5 klatkach na sekunde to okolo dwoch
 * sekund raz na pobudke - cena, ktora warto zaplacic za to, zeby pierwsza
 * klatka w zbiorze nie byla smieciem.
 */
#define FLORA_CAMERA_WARMUP_FRAMES 10

static bool s_ready;
static bool s_af_loaded;
static flora_camera_format_t s_format;

static const char *format_name(flora_camera_format_t f)
{
    switch (f) {
        case FLORA_CAMERA_RGB565: return "RGB565";
        case FLORA_CAMERA_JPEG:   return "JPEG";
        default:                  return "w odcieniach szarosci";
    }
}

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
    else if (cfg->width <= 640) fs = FRAMESIZE_VGA;     /* 640x480 */
    else if (cfg->width <= 800) fs = FRAMESIZE_SVGA;    /* 800x600 */
    else {
        ESP_LOGW(TAG, "%u px szerokosci nie obsluguje - ustawiam 800",
                 cfg->width);
        fs = FRAMESIZE_SVGA;
    }

    /* Nieskompresowana klatka ponad QVGA to dla nas pulapka: 640x480 w
     * RGB565 to 614 kB na bufor, a sam odczyt i przeliczenie zjadaja wiecej
     * czasu niz cokolwiek, co z ta klatka zrobimy. Przy JPEG limitu nie ma,
     * bo czujnik pakuje sprzetowo. */
    if (cfg->format != FLORA_CAMERA_JPEG && fs > FRAMESIZE_QVGA) {
        ESP_LOGW(TAG, "%u px bez kompresji to za duzo na tym ukladzie - "
                      "ustawiam 320", cfg->width);
        fs = FRAMESIZE_QVGA;
    }

    pixformat_t pf;
    switch (cfg->format) {
        case FLORA_CAMERA_RGB565: pf = PIXFORMAT_RGB565;    break;
        case FLORA_CAMERA_JPEG:   pf = PIXFORMAT_JPEG;      break;
        default:                  pf = PIXFORMAT_GRAYSCALE; break;
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

        .pixel_format = pf,
        .frame_size   = fs,
        .jpeg_quality = cfg->jpeg_quality ? cfg->jpeg_quality : 12,
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
        /* Znaczniki to czarno-biale wzory o wysokim kontrascie, wiec lekkie
         * podbicie kontrastu ulatwia progowanie w detektorze. Przy barwie
         * NIE podbijamy: wykrywanie przeszkod porownuje chromatycznosc z
         * wzorcem podloza, a kontrast przesuwa barwy nierownomiernie i
         * rozjezdza to porownanie. */
        s->set_contrast(s, cfg->format == FLORA_CAMERA_GRAY ? 1 : 0);
        s->set_brightness(s, 0);

        /* Odbicia lustrzane zmieniaja ZNAK azymutu, wiec wylaczamy oba -
         * inaczej robot jechalby w przeciwna strone, niz widzi znacznik. */
        s->set_hmirror(s, 0);
        s->set_vflip(s, 0);
        ESP_LOGI(TAG, "czujnik PID 0x%02X", s->id.PID);

        /* UWAGA NA INTERPRETACJE TEJ LINII. Sterownik OV5640 odpowiada na
         * af_is_supported() na sztywno "tak" - mowi o ukladzie, nie o tym,
         * czy do obiektywu dolaczono silniczek. Brak oznacza wiec tylko
         * tyle, ze obsluga nie zostala wkompilowana. Dowodem na silniczek
         * jest dopiero udana proba ustawienia ostrosci. */
        if (s->af_is_supported && s->af_is_supported(s)) {
            ESP_LOGI(TAG, "autofokus: sterownik go umie - czy jest silniczek, "
                          "pokaze dopiero proba");
        } else {
            ESP_LOGI(TAG, "autofokus: obsluga niewkompilowana "
                          "(CONFIG_CAMERA_AF_SUPPORT)");
        }
    }

    s_ready = true;
    s_format = cfg->format;

    /* Rozgrzewka: pierwsze klatki po inicjalizacji powstaja, zanim
     * automatyka ekspozycji i balansu bieli zdazy sie ustawic, i sa
     * zauwazalnie inne od reszty. Przy zbieraniu zbioru to jest trucizna,
     * bo pierwsza klatka sesji trafia do danych jako pelnoprawna. */
    for (int i = 0; i < FLORA_CAMERA_WARMUP_FRAMES; i++) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            break;
        }
        esp_camera_fb_return(fb);
    }

    /* BALANS BIELI BLOKUJEMY DOPIERO TERAZ, PO ROZGRZEWCE, a ekspozycje
     * zostawiamy automatyczna.
     *
     * Kolejnosc jest tu istotna i za pierwszym razem zrobilem ja zle:
     * blokada przed rozgrzewka zamrazala balans na fabrycznych wartosciach
     * czujnika, bo automatyka nie dostawala ani jednej klatki na
     * ustawienie sie. Teraz najpierw pozwalamy jej zbiec, potem zamykamy.
     *
     * Po co w ogole blokowac: wykrywanie przeszkod porownuje CHROMATYCZNOSC
     * miedzy klatkami (pamiec wzorcow podloza). Chromatycznosc jest odporna
     * na jasnosc, wiec automatyka ekspozycji nam nie szkodzi i chcemy ja
     * miec - inaczej wjazd ze slonca w cien dalby czarna klatke. Balans
     * bieli szkodzi za to wprost: przesuwa WSZYSTKIE piksele miedzy
     * kubelkami barwy naraz. Zmierzone na zywych danych - miedzy dwiema
     * sesjami udzial niebieskiego skoczyl z 24,1% na 27,6% i ponad polowa
     * komorek kadru zmienila barwe, choc scena byla ta sama. */
    if (cfg->format == FLORA_CAMERA_RGB565 && s) {
        s->set_whitebal(s, 0);
        s->set_awb_gain(s, 0);
        ESP_LOGI(TAG, "balans bieli zablokowany po rozgrzewce");
    }
    ESP_LOGI(TAG, "kamera gotowa: %ux%u %s, %u bufor(y)",
             cfg->width, cfg->height, format_name(cfg->format), cc.fb_count);
    return ESP_OK;
}

esp_err_t flora_camera_deinit(void)
{
    ESP_RETURN_ON_FALSE(s_ready, ESP_ERR_INVALID_STATE, TAG, "nie zainicjalizowano");
    s_ready = false;
    /* Firmware AF zyje w czujniku, a ten wlasnie traci zasilanie zegara,
     * wiec po pobudce trzeba je zaladowac od nowa. */
    s_af_loaded = false;
    return esp_camera_deinit();
}

esp_err_t flora_camera_grab(flora_camera_frame_t *out, uint32_t timeout_ms)
{
    ESP_RETURN_ON_FALSE(s_ready && out, ESP_ERR_INVALID_STATE, TAG, "nie gotowe");
    (void)timeout_ms;   /* sterownik nie przyjmuje limitu czasu */

    camera_fb_t *fb = esp_camera_fb_get();
    ESP_RETURN_ON_FALSE(fb, ESP_ERR_TIMEOUT, TAG, "brak klatki");

    out->data         = fb->buf;
    out->len          = (uint32_t)fb->len;
    out->width        = (uint16_t)fb->width;
    out->height       = (uint16_t)fb->height;
    out->format       = s_format;
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
        frame->len = 0;
    }
}

bool flora_camera_is_ready(void) { return s_ready; }

bool flora_camera_focus_available(void)
{
    if (!s_ready) {
        return false;
    }
    sensor_t *s = esp_camera_sensor_get();
    return s && s->af_is_supported && s->af_is_supported(s);
}

esp_err_t flora_camera_focus_trigger(uint32_t timeout_ms, bool *out_focused)
{
    ESP_RETURN_ON_FALSE(s_ready, ESP_ERR_INVALID_STATE, TAG, "kamera nie dziala");

    sensor_t *s = esp_camera_sensor_get();
    if (!s || !s->af_is_supported || !s->af_is_supported(s) ||
        !s->af_init || !s->af_trigger) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    if (!s_af_loaded) {
        ESP_LOGI(TAG, "laduje firmware autofokusu do czujnika...");
        if (s->af_init(s, timeout_ms) != 0) {
            /* Najczestszy powod: do czujnika nie dolaczono silniczka, wiec
             * firmware nie ma czym ruszac i nie potwierdza zaladowania. */
            ESP_LOGW(TAG, "firmware AF nie wystartowal - zwykle znaczy to, "
                          "ze modul nie ma silniczka ostrosci");
            return ESP_ERR_TIMEOUT;
        }
        s_af_loaded = true;
    }

    if (s->af_trigger(s) != 0) {
        return ESP_FAIL;
    }

    bool focused = false, busy = true;
    const int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    while (esp_timer_get_time() < deadline) {
        uint8_t raw = 0;
        if (s->af_get_status && s->af_get_status(s, &raw, &focused, &busy) == 0) {
            if (focused || !busy) {
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    if (out_focused) {
        *out_focused = focused;
    }
    ESP_LOGI(TAG, "autofokus: %s", focused ? "ostrosc zlapana" : "bez potwierdzenia");
    return focused ? ESP_OK : ESP_ERR_TIMEOUT;
}
