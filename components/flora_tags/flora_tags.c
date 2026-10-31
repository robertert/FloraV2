#include <inttypes.h>
#include <math.h>
#include <string.h>

#include "flora_tags.h"

#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "apriltag.h"
#include "tag36h11.h"
#include "tag25h9.h"
#include "tag16h5.h"

static const char *TAG = "flora_tags";

static struct {
    apriltag_detector_t *det;
    apriltag_family_t   *fam;
    flora_tags_family_t  which;
    flora_tags_cfg_t     cfg;
    bool initialized;
} s_t;

static apriltag_family_t *family_create(flora_tags_family_t f)
{
    switch (f) {
        case FLORA_TAGS_FAMILY_25H9: return tag25h9_create();
        case FLORA_TAGS_FAMILY_16H5: return tag16h5_create();
        default:                     return tag36h11_create();
    }
}

static void family_destroy(flora_tags_family_t f, apriltag_family_t *fam)
{
    switch (f) {
        case FLORA_TAGS_FAMILY_25H9: tag25h9_destroy(fam); break;
        case FLORA_TAGS_FAMILY_16H5: tag16h5_destroy(fam); break;
        default:                     tag36h11_destroy(fam); break;
    }
}

esp_err_t flora_tags_init(const flora_tags_cfg_t *cfg)
{
    ESP_RETURN_ON_FALSE(cfg, ESP_ERR_INVALID_ARG, TAG, "cfg == NULL");
    ESP_RETURN_ON_FALSE(!s_t.initialized, ESP_ERR_INVALID_STATE, TAG, "juz zainicjalizowano");
    ESP_RETURN_ON_FALSE(cfg->tag_size_mm > 0.0f, ESP_ERR_INVALID_ARG, TAG,
                        "tag_size_mm musi byc dodatnie");

    memset(&s_t, 0, sizeof(s_t));
    s_t.cfg   = *cfg;
    s_t.which = cfg->family;

    s_t.det = apriltag_detector_create();
    ESP_RETURN_ON_FALSE(s_t.det, ESP_ERR_NO_MEM, TAG, "apriltag_detector_create");

    s_t.fam = family_create(cfg->family);
    if (!s_t.fam) {
        apriltag_detector_destroy(s_t.det);
        ESP_RETURN_ON_FALSE(false, ESP_ERR_NO_MEM, TAG, "utworzenie rodziny znacznikow");
    }
    apriltag_detector_add_family_bits(s_t.det, s_t.fam, cfg->max_hamming);

    s_t.det->quad_decimate = cfg->quad_decimate > 0.0f ? cfg->quad_decimate : 1.0f;
    s_t.det->quad_sigma    = cfg->quad_sigma;
    /* Jeden watek: na S3 zrownoleglenie i tak walczy o te same rdzenie
     * z telemetria i laczem, a biblioteka tworzy wtedy wlasne zadania
     * o nieznanym rozmiarze stosu. */
    s_t.det->nthreads      = 1;
    s_t.det->refine_edges  = true;

    if (cfg->focal_px <= 0.0f) {
        ESP_LOGW(TAG, "brak ogniskowej (focal_px = 0) - odleglosc i azymut");
        ESP_LOGW(TAG, "beda zerowe. Wyznacz ja pomiarem: postaw znacznik na");
        ESP_LOGW(TAG, "znanej odleglosci i uzyj flora_tags_focal_from_measurement()");
    }

    s_t.initialized = true;
    ESP_LOGI(TAG, "detektor gotowy: rodzina %d, zmniejszenie %.1f, "
                  "poprawa %d bitow, znacznik %.0f mm",
             (int)cfg->family, s_t.det->quad_decimate, cfg->max_hamming,
             cfg->tag_size_mm);
    return ESP_OK;
}

esp_err_t flora_tags_deinit(void)
{
    ESP_RETURN_ON_FALSE(s_t.initialized, ESP_ERR_INVALID_STATE, TAG, "nie zainicjalizowano");
    apriltag_detector_destroy(s_t.det);
    family_destroy(s_t.which, s_t.fam);
    memset(&s_t, 0, sizeof(s_t));
    return ESP_OK;
}

/** Sredni bok znacznika w pikselach, z czterech naroznikow. */
static float apparent_size_px(const apriltag_detection_t *d)
{
    float sum = 0.0f;
    for (int i = 0; i < 4; i++) {
        int j = (i + 1) % 4;
        float dx = (float)(d->p[j][0] - d->p[i][0]);
        float dy = (float)(d->p[j][1] - d->p[i][1]);
        sum += sqrtf(dx*dx + dy*dy);
    }
    return sum / 4.0f;
}

esp_err_t flora_tags_detect(const uint8_t *gray, uint16_t w, uint16_t h,
                            flora_tags_result_t *out)
{
    ESP_RETURN_ON_FALSE(s_t.initialized, ESP_ERR_INVALID_STATE, TAG, "nie zainicjalizowano");
    ESP_RETURN_ON_FALSE(gray && out && w && h, ESP_ERR_INVALID_ARG, TAG, "zly argument");

    memset(out, 0, sizeof(*out));
    int64_t t0 = esp_timer_get_time();

    /* Biblioteka przyjmuje obraz bez kopiowania - opisujemy tylko bufor
     * kamery. stride rowny szerokosci, bo format GRAYSCALE jest ciagly. */
    image_u8_t im = {
        .width  = w,
        .height = h,
        .stride = w,
        .buf    = (uint8_t *)gray,
    };

    zarray_t *dets = apriltag_detector_detect(s_t.det, &im);
    if (!dets) {
        out->process_ms = (uint32_t)((esp_timer_get_time() - t0) / 1000);
        return ESP_OK;
    }

    float cx = (float)w / 2.0f;
    int n = zarray_size(dets);

    for (int i = 0; i < n && out->count < FLORA_TAGS_MAX_DETECTIONS; i++) {
        apriltag_detection_t *d;
        zarray_get(dets, i, &d);

        /* Niska ostrosc krawedzi to zwykle przypadkowy wzor, nie znacznik.
         * Falszywa detekcja myli filtr Kalmana gorzej niz brak detekcji. */
        if (d->decision_margin < s_t.cfg.min_decision_margin) {
            continue;
        }

        flora_tag_detection_t *o = &out->items[out->count++];
        o->id              = (uint16_t)d->id;
        o->center_x        = (float)d->c[0];
        o->center_y        = (float)d->c[1];
        o->decision_margin = d->decision_margin;

        if (s_t.cfg.focal_px > 0.0f) {
            float px = apparent_size_px(d);
            if (px > 1.0f) {
                /* Podobienstwo trojkatow: im mniejszy obraz, tym dalej. */
                o->distance_mm = s_t.cfg.focal_px * s_t.cfg.tag_size_mm / px;
            }
            /* Azymut z przesuniecia srodka. Dla obiektywow szerokokatnych
             * jest to przyblizenie - dystorsja beczkowa zawyza kat na
             * brzegach obrazu i wymagalaby pelnej kalibracji. */
            o->bearing_deg = atanf((o->center_x - cx) / s_t.cfg.focal_px)
                             * 180.0f / (float)M_PI;
        }

        /* Jakosc z DWOCH czynnikow, bo oba psuja pomiar inaczej:
         *  - ostrosc krawedzi mowi, jak pewna jest sama detekcja
         *  - pozorny rozmiar mowi, jak pewna jest ODLEGLOSC: blad jednego
         *    piksela w pomiarze boku przeklada sie na blad odleglosci
         *    rosnacy z kwadratem, wiec maly znacznik to niepewny dystans
         *    przy wciaz dobrym azymucie
         * Filtr na dole wazy tym kazda detekcje. */
        float q_margin = d->decision_margin * 1.5f;
        if (q_margin > 100.0f) {
            q_margin = 100.0f;
        }
        float q_size = 100.0f;
        if (s_t.cfg.focal_px > 0.0f) {
            float px = apparent_size_px(d);
            /* Pelne zaufanie od 40 px boku (okolo 2 m przy znaczniku
             * 300 mm), potem liniowo w dol do progu dekodowania. */
            q_size = px / 40.0f * 100.0f;
            if (q_size > 100.0f) q_size = 100.0f;
            if (q_size < 10.0f)  q_size = 10.0f;
        }
        float q = q_margin < q_size ? q_margin : q_size;
        o->quality = (uint8_t)q;
    }

    /* Biblioteka wymaga jawnego zwolnienia kazdej detekcji ORAZ tablicy -
     * bez tego pamiec ucieka po kilku klatkach. */
    apriltag_detections_destroy(dets);

    out->process_ms = (uint32_t)((esp_timer_get_time() - t0) / 1000);
    return ESP_OK;
}
