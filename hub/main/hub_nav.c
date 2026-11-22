/**
 * @file hub_nav.c
 * @brief Petla EKF pozycji (kilka wariantow rownolegle), opis w hub_nav.h.
 *
 * Zadanie 50 Hz. Cala logika taktu (warianty, bramki, zyroskop) siedzi
 * w components/flora_nav (flora_navcore), zeby nagranie przejazdu dalo sie
 * przeliczyc na komputerze tym samym kodem. Tutaj zostaje to, co zalezy od
 * ESP: zegar, enkodery, bufory hub_links, NVS, MQTT i rekorder.
 *
 * Wiadomosci glowicy czyta z buforow hub_links (ostatnia wartosc + numer
 * kopii), wiec nie konkuruje z telemetria i nie blokuje lacza.
 */
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_netif_sntp.h"
#include "esp_timer.h"

#include "hub_links.h"
#include "hub_nav.h"
#include "hub_params.h"
#include "hub_rec.h"
#include "flora_board.h"
#include "flora_encoders.h"
#include "flora_msgs.h"
#include "flora_mqtt.h"
#include "flora_navcore.h"
#include "flora_navlog.h"
#include "flora_store.h"

static const char *TAG = "nav";

#define LOOP_MS          20
#define TIME_VALID_UNIX  1735689600LL   /**< 2025-01-01: wczesniej zegar nie byl ustawiony */
#define STILL_EVERY_US   1000000        /**< odswiezenie flagi postoju dla glowicy */

#define R2D(r) ((r) * 180.0f / (float)M_PI)

/* ---- zapamietywanie pozy (NVS) ----
 * Zapis tylko na postoju, najwyzej co POSE_SAVE_GAP_US i tylko po ruchu -
 * NVS zuzywa flash przy kazdym zapisie, a na postoju poza i tak stoi. */
#define POSE_SAVE_STILL_US  3000000
#define POSE_SAVE_GAP_US    30000000
#define POSE_SAVE_MOVE_MM   50.0f
#define POSE_SAVE_TURN_DEG  3.0f
/* Niepewnosc po odtworzeniu: robot mogl zostac przeniesiony przy
 * wylaczonym zasilaniu. Wystarczajaco szeroko, zeby znaczniki i kurs
 * przeszly bramke i poprawily poze; przeniesienie o wiecej - cmd/pose_reset. */
#define POSE_RESTORE_XY_MM  300.0f
#define POSE_RESTORE_TH_DEG 10.0f
#define POSE_RESTORE_B_DEG  10.0f
#define POSE_BLOB_VERSION   1

typedef struct {
    uint32_t version;
    float    x_mm, y_mm, th_deg, b_deg;
    float    spos_mm;    /**< niepewnosc w chwili zapisu */
} pose_blob_t;

_Static_assert(HUB_NAV_MAX_TAGS == FLORA_NAVCORE_MAX_TAGS, "mapa palikow: rozne limity");
_Static_assert(FLORA_TAGS_MAX <= FLORA_NAVCORE_MAX_DETS, "za malo miejsca na wykrycia");

/** Palik w NVS - wlasny uklad, niezalezny od struktur flora_navcore. */
typedef struct {
    uint16_t id;
    uint16_t _pad;
    float    x_mm, y_mm;
} tag_blob_t;

static struct {
    SemaphoreHandle_t lock;
    bool              ready;
    flora_navcore_t   core;

    /* Zdarzenia z innych zadan wykonywane przez nav_task na poczatku taktu -
     * w nagraniu laduja wtedy dokladnie miedzy tymi samymi taktami,
     * miedzy ktorymi zadzialaly na robocie. */
    bool  pend_reset, pend_geom;
    float reset_x, reset_y, reset_th;

    /* zapamietywanie pozy - tylko nav_task (i start, zanim ruszy) */
    bool        pend_save;      /**< zapisz w najblizszym takcie (po resecie) */
    int64_t     still_since_us;
    int64_t     last_save_us;
    pose_blob_t saved;
    bool        have_saved;
} s_n;

static bool time_ok(time_t *now)
{
    time(now);
    return (long long)*now > TIME_VALID_UNIX;
}

static flora_navcore_geom_t geom_from(const hub_params_t *p)
{
    return (flora_navcore_geom_t){
        .track_mm = p->track_mm,
        .cam_x_mm = p->cam_x_mm,
        .cam_y_mm = p->cam_y_mm,
        .lat_deg  = p->lat_deg,
        .lon_deg  = p->lon_deg,
        .decl_deg = p->decl_deg,
    };
}

void hub_nav_params_changed(void)
{
    if (!s_n.ready) {
        return;   /* start i tak wczyta biezace */
    }
    xSemaphoreTake(s_n.lock, portMAX_DELAY);
    s_n.pend_geom = true;
    xSemaphoreGive(s_n.lock);
}

/** Odlozone zdarzenia - wolane z nav_task przed zapisem wejsc taktu. */
static void apply_pending(void)
{
    hub_params_t p;
    bool geom = false, reset = false;
    float x = 0, y = 0, th = 0;

    xSemaphoreTake(s_n.lock, portMAX_DELAY);
    if (s_n.pend_geom) {
        hub_params_get(&p);
        flora_navcore_geom_t g = geom_from(&p);
        flora_navcore_set_geom(&s_n.core, &g);
        s_n.pend_geom = false;
        geom = true;
    }
    if (s_n.pend_reset) {
        x = s_n.reset_x; y = s_n.reset_y; th = s_n.reset_th;
        flora_navcore_reset_pose(&s_n.core, x, y, th);
        s_n.pend_reset = false;
        reset = true;
    }
    xSemaphoreGive(s_n.lock);

    if (geom) {
        hub_rec_header();
    }
    if (reset) {
        s_n.pend_save = true;   /* nowa poza od razu do NVS */
        hub_rec_printf(FLORA_NAVLOG_FMT_R, x, y, th);
        ESP_LOGI(TAG, "poza ustawiona we wszystkich wariantach: (%.0f, %.0f) mm, %.1f st", x, y, th);
    }
}

/* ================= WEJSCIA TAKTU =================
 * Kazde wejscie jest przekladane na typ flora_navcore i jednoczesnie
 * zapisywane do nagrania w tej samej postaci, w jakiej trafia do filtra. */

static bool read_att(uint32_t *seq, flora_navcore_att_t *out)
{
    flora_attitude_t att;
    if (!hub_links_get(FLORA_MSG_ATTITUDE, &att, sizeof att, seq)) {
        return false;
    }
    /* Wiek probki: przy TIME_SYNC znacznik jest w naszym zegarze (mlodsze
     * 32 bity, odejmowanie modulo 2^32). */
    int32_t age_us = -1;
    if (att.flags & FLORA_ATT_TIME_SYNCED) {
        age_us = (int32_t)((uint32_t)esp_timer_get_time() - att.timestamp_us);
    }
    *out = (flora_navcore_att_t){
        .heading_valid = (att.flags & FLORA_ATT_HEADING_VALID) != 0,
        .mag_valid     = (att.flags & FLORA_ATT_MAG_VALID) != 0,
        .heading_deg   = att.heading_cdeg / 100.0f,
        .rate_dps      = att.rate_z_cdps / 100.0f,
        .sigma_deg     = att.heading_sigma_cdeg / 100.0f,
        .quality       = att.quality,
        .age_us        = age_us,
    };
    unsigned flags = (out->heading_valid ? FLORA_NAVLOG_ATT_HEADING_VALID : 0) |
                     (out->mag_valid ? FLORA_NAVLOG_ATT_MAG_VALID : 0);
    hub_rec_printf(FLORA_NAVLOG_FMT_A, (long)age_us, (unsigned)att.heading_cdeg,
                   (int)att.rate_z_cdps, flags, (unsigned)att.heading_sigma_cdeg,
                   (unsigned)att.quality);
    return true;
}

static bool read_sun(uint32_t *seq, flora_navcore_sun_t *out, int64_t *unix_s)
{
    flora_sun_t sun;
    if (!hub_links_get(FLORA_MSG_SUN, &sun, sizeof sun, seq)) {
        return false;
    }
    time_t now;
    *unix_s = time_ok(&now) ? (int64_t)now : 0;
    out->valid       = sun.valid != 0;
    out->azimuth_deg = sun.azimuth_cdeg / 100.0f;
    for (int i = 0; i < 4; i++) {
        out->lux[i] = sun.lux_div4[i] * 4.0f;
    }
    hub_rec_printf(FLORA_NAVLOG_FMT_S, (long long)*unix_s, (unsigned)sun.azimuth_cdeg,
                   out->lux[0], out->lux[1], out->lux[2], out->lux[3], (unsigned)out->valid);
    return true;
}

static int read_tags(uint32_t *seq, flora_navcore_det_t *dets)
{
    flora_tags_t msg;
    if (!hub_links_get(FLORA_MSG_TAGS, &msg, sizeof msg, seq)) {
        return -1;
    }
    int n = msg.count > FLORA_TAGS_MAX ? FLORA_TAGS_MAX : msg.count;
    /* Obrot kamery wzgledem przodu robota: kat znacznika wzgledem OSI ROBOTA.
     * Do nagrania idzie juz poprawiony, zeby odtworzenie na komputerze
     * dawalo to samo co robot. */
    hub_params_t p;
    hub_params_get(&p);
    const int yaw_cdeg = (int)lroundf(p.cam_yaw_deg * 100.0f);
    /* Kamera mierzy w linii prostej, EKF liczy na plaszczyznie. */
    const float dh = p.cam_h_mm - p.tag_h_mm;
    char line[160];
    int len = snprintf(line, sizeof line, FLORA_NAVLOG_FMT_G_HEAD, (unsigned)n);
    for (int i = 0; i < n; i++) {
        const flora_tag_t *t = &msg.tags[i];
        const int bearing_cdeg = (int)t->bearing_cdeg + yaw_cdeg;
        float d = t->distance_mm;
        if (d > 0.0f && fabsf(dh) > 0.0f && d > fabsf(dh)) {
            d = sqrtf(d * d - dh * dh);
        }
        const uint16_t dist_mm = (uint16_t)lroundf(d);
        dets[i] = (flora_navcore_det_t){
            .id          = t->id,
            .distance_mm = dist_mm,
            .bearing_deg = bearing_cdeg / 100.0f,
            .quality     = t->quality,
        };
        len += snprintf(line + len, sizeof line - len, FLORA_NAVLOG_FMT_G_ITEM,
                        (unsigned)t->id, (unsigned)dist_mm, bearing_cdeg,
                        (unsigned)t->quality);
    }
    hub_rec_printf("%s\n", line);
    return n;
}

/* ================= ZAPAMIETYWANIE POZY ================= */

/** Poza glownego wariantu. Zaklada blokade. */
static pose_blob_t pose_now(void)
{
    const flora_ekf_t *e = &s_n.core.v[0].ekf;
    return (pose_blob_t){
        .version = POSE_BLOB_VERSION,
        .x_mm    = e->x[FLORA_EKF_X],
        .y_mm    = e->x[FLORA_EKF_Y],
        .th_deg  = R2D(e->x[FLORA_EKF_TH]),
        .b_deg   = R2D(e->x[FLORA_EKF_B]),
        .spos_mm = flora_ekf_pos_sigma_max(e),
    };
}

static void pose_store(const pose_blob_t *p)
{
    flora_store_t h;
    if (flora_store_open(FLORA_NVS_NS_CALIB, &h) != ESP_OK) {
        return;
    }
    if (flora_store_set_blob(h, FLORA_NVS_NAV_POSE, p, sizeof *p) != ESP_OK) {
        ESP_LOGW(TAG, "zapis pozy do NVS nieudany");
    }
    flora_store_close(h);
}

/** Wolane z nav_task po kazdym takcie, bez blokady. */
static void pose_maybe_save(const pose_blob_t *cur, bool still, int64_t now_us)
{
    if (!still) {
        s_n.still_since_us = 0;
        if (!s_n.pend_save) {
            return;
        }
    } else if (!s_n.still_since_us) {
        s_n.still_since_us = now_us;
    }
    bool due = s_n.pend_save;
    if (!due && s_n.still_since_us && now_us - s_n.still_since_us >= POSE_SAVE_STILL_US &&
        now_us - s_n.last_save_us >= POSE_SAVE_GAP_US) {
        float dx = cur->x_mm - s_n.saved.x_mm, dy = cur->y_mm - s_n.saved.y_mm;
        float dth = fabsf(fmodf(cur->th_deg - s_n.saved.th_deg + 540.0f, 360.0f) - 180.0f);
        due = !s_n.have_saved || dx * dx + dy * dy > POSE_SAVE_MOVE_MM * POSE_SAVE_MOVE_MM ||
              dth > POSE_SAVE_TURN_DEG;
    }
    if (!due) {
        return;
    }
    pose_store(cur);
    s_n.saved        = *cur;
    s_n.have_saved   = true;
    s_n.last_save_us = now_us;
    s_n.pend_save    = false;
}

/** Przy starcie, przed zadaniem nav. */
static void pose_restore(void)
{
    flora_store_t h;
    if (flora_store_open(FLORA_NVS_NS_CALIB, &h) != ESP_OK) {
        return;
    }
    pose_blob_t p;
    size_t len = sizeof p;
    bool ok = flora_store_get_blob(h, FLORA_NVS_NAV_POSE, &p, &len) == ESP_OK &&
              len == sizeof p && p.version == POSE_BLOB_VERSION &&
              isfinite(p.x_mm) && isfinite(p.y_mm) && isfinite(p.th_deg) && isfinite(p.b_deg);
    flora_store_close(h);
    if (!ok) {
        ESP_LOGI(TAG, "brak zapisanej pozy - start w (0, 0, 0)");
        return;
    }
    float sxy = p.spos_mm > POSE_RESTORE_XY_MM ? p.spos_mm : POSE_RESTORE_XY_MM;
    flora_navcore_restore_pose(&s_n.core, p.x_mm, p.y_mm, p.th_deg, p.b_deg,
                               sxy, POSE_RESTORE_TH_DEG, POSE_RESTORE_B_DEG);
    s_n.saved      = p;
    s_n.have_saved = true;
    ESP_LOGI(TAG, "poza z NVS: (%.0f, %.0f) mm, %.1f st, b %.1f st, niepewnosc %.0f mm "
                  "(przeniesiony robot: cmd/pose_reset)",
             p.x_mm, p.y_mm, p.th_deg, p.b_deg, sxy);
}

/** Flaga postoju dla glowicy (ZUPT): przy zmianie i co STILL_EVERY_US. */
static void send_still(bool still, int64_t now_us)
{
    static bool    sent_still;
    static int64_t sent_us;
    static bool    sent_once;
    if (sent_once && still == sent_still && now_us - sent_us < STILL_EVERY_US) {
        return;
    }
    flora_link_handle_t h = hub_links_handle();
    if (!h) {
        return;
    }
    flora_cmd_t c = { .code = FLORA_CMD_SET_STILL, .arg = still ? 1 : 0 };
    /* Bez potwierdzenia: nastepne odswiezenie przyjdzie za sekunde,
     * a glowica sama gasi flage po 2 s ciszy. */
    flora_link_send(h, FLORA_MSG_CMD, &c, sizeof c);
    sent_still = still;
    sent_us    = now_us;
    sent_once  = true;
}

static void nav_task(void *arg)
{
    TickType_t wake = xTaskGetTickCount();
    flora_encoders_sample_t prev;
    bool have_prev = false;
    uint32_t seq_att = 0, seq_sun = 0, seq_tags = 0;
    int last_gate = FLORA_SUNGATE_NONE;

    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(LOOP_MS));

        apply_pending();

        flora_encoders_sample_t now;
        if (flora_encoders_read(&now) != ESP_OK) {
            continue;
        }
        if (!have_prev) {
            prev = now;
            have_prev = true;
            hub_rec_printf(FLORA_NAVLOG_FMT_E, (long long)now.timestamp_us,
                           (long long)now.left_counts, (long long)now.right_counts);
            continue;
        }

        flora_navcore_att_t att;
        flora_navcore_sun_t sun;
        flora_navcore_det_t dets[FLORA_NAVCORE_MAX_DETS];
        int64_t unix_s = 0;
        bool have_att = read_att(&seq_att, &att);
        bool have_sun = read_sun(&seq_sun, &sun, &unix_s);
        int  n_dets   = read_tags(&seq_tags, dets);
        /* E zamyka takt w nagraniu - po A, S, G z tego samego taktu. */
        hub_rec_printf(FLORA_NAVLOG_FMT_E, (long long)now.timestamp_us,
                       (long long)now.left_counts, (long long)now.right_counts);

        float mm_l, mm_r;
        flora_encoders_get_mm_per_count(&mm_l, &mm_r);
        /* Przyrosty z calkowitych zliczen, nie z narastajacych mm w float:
         * po kilku km float gubi milimetry, a przyrost na takt to ~2 mm. */
        flora_navcore_in_t in = {
            .t_us   = now.timestamp_us,
            .dl_mm  = (float)(now.left_counts  - prev.left_counts)  * mm_l,
            .dr_mm  = (float)(now.right_counts - prev.right_counts) * mm_r,
            .dt_s   = (float)(now.timestamp_us - prev.timestamp_us) / 1e6f,
            .att    = have_att ? &att : NULL,
            .sun    = have_sun ? &sun : NULL,
            .unix_s = unix_s,
            .dets   = n_dets >= 0 ? dets : NULL,
            .n_dets = n_dets,
        };
        prev = now;

        xSemaphoreTake(s_n.lock, portMAX_DELAY);
        bool stepped = flora_navcore_step(&s_n.core, &in);
        bool still   = s_n.core.still;
        const uint32_t head_jumps = s_n.core.n_head_jumps;
        const float    jump_deg   = s_n.core.last_head_jump_deg;
        int  gate    = s_n.core.sun_gate;
        pose_blob_t cur = pose_now();
        xSemaphoreGive(s_n.lock);
        pose_maybe_save(&cur, still, now.timestamp_us);

        static uint32_t s_seen_jumps;
        if (head_jumps != s_seen_jumps) {
            s_seen_jumps = head_jumps;
            ESP_LOGW(TAG, "skok kursu glowicy o %.1f st bez obrotu w zyroskopie - "
                          "przesuwam odniesienie (b), poza i kat zostaja", jump_deg);
        }

        if (!stepped) {
            ESP_LOGW(TAG, "skok enkoderow (%.0f / %.0f mm) - pomijam krok", in.dl_mm, in.dr_mm);
            continue;
        }
        if (gate != last_gate) {
            /* Tylko przejscia, zeby log nie puchl co sekunde. UNSTABLE zaraz
             * po DIM/LOW to normalne zbieranie probek - nie ostrzegamy. */
            if (gate == FLORA_SUNGATE_OK) {
                ESP_LOGI(TAG, "kompas sloneczny: pomiary przyjmowane");
            } else if (gate != FLORA_SUNGATE_UNSTABLE || last_gate == FLORA_SUNGATE_OK) {
                ESP_LOGW(TAG, "kompas sloneczny pominiety: %s",
                         flora_sungate_name((flora_sungate_result_t)gate));
            }
            last_gate = gate;
        }
        send_still(still, now.timestamp_us);
    }
}

/* ================= MAPA PALIKOW: NVS i cmd/tag ================= */

static void tags_load(void)
{
    flora_navcore_t *c = &s_n.core;
    c->n_tags = 0;
    flora_store_t h;
    if (flora_store_open(FLORA_NVS_NS_CALIB, &h) != ESP_OK) {
        return;
    }
    tag_blob_t blob[HUB_NAV_MAX_TAGS];
    size_t len = sizeof blob;
    if (flora_store_get_blob(h, FLORA_NVS_TAG_MAP, blob, &len) == ESP_OK &&
        len % sizeof(tag_blob_t) == 0) {
        c->n_tags = (int)(len / sizeof(tag_blob_t));
        for (int i = 0; i < c->n_tags; i++) {
            c->tags[i] = (flora_navcore_tag_t){ blob[i].id, blob[i].x_mm, blob[i].y_mm };
        }
    }
    flora_store_close(h);
}

/** Zaklada blokade. */
static esp_err_t tags_save(void)
{
    const flora_navcore_t *c = &s_n.core;
    tag_blob_t blob[HUB_NAV_MAX_TAGS];
    for (int i = 0; i < c->n_tags; i++) {
        blob[i] = (tag_blob_t){ .id = c->tags[i].id, .x_mm = c->tags[i].x_mm, .y_mm = c->tags[i].y_mm };
    }
    flora_store_t h;
    esp_err_t err = flora_store_open(FLORA_NVS_NS_CALIB, &h);
    if (err != ESP_OK) {
        return err;
    }
    err = c->n_tags ? flora_store_set_blob(h, FLORA_NVS_TAG_MAP, blob, c->n_tags * sizeof(tag_blob_t))
                    : flora_store_erase_key(h, FLORA_NVS_TAG_MAP);
    flora_store_close(h);
    return err;
}

static void tag_publish_one(const flora_navcore_tag_t *t, uint16_t id)
{
    char topic[24];
    snprintf(topic, sizeof topic, "config_tags/%u", id);
    if (t) {
        flora_mqtt_publish_fmt(topic, 1, true, "{\"x_mm\":%.0f,\"y_mm\":%.0f}", t->x_mm, t->y_mm);
    } else {
        flora_mqtt_publish(topic, "", 0, 1, true);   /* pusty retain kasuje */
    }
}

/** Kopia mapy pod blokada. */
static int tags_copy(flora_navcore_tag_t *out)
{
    xSemaphoreTake(s_n.lock, portMAX_DELAY);
    int n = s_n.core.n_tags;
    memcpy(out, s_n.core.tags, n * sizeof(flora_navcore_tag_t));
    xSemaphoreGive(s_n.lock);
    return n;
}

void hub_nav_tags_publish(void)
{
    if (!s_n.ready) {
        return;
    }
    flora_navcore_tag_t copy[HUB_NAV_MAX_TAGS];
    int n = tags_copy(copy);
    for (int i = 0; i < n; i++) {
        tag_publish_one(&copy[i], copy[i].id);
    }
}

void hub_nav_rec_tags(void)
{
    if (!s_n.ready || !hub_rec_active()) {
        return;
    }
    flora_navcore_tag_t copy[HUB_NAV_MAX_TAGS];
    int n = tags_copy(copy);
    hub_rec_printf(FLORA_NAVLOG_FMT_C);
    for (int i = 0; i < n; i++) {
        hub_rec_printf(FLORA_NAVLOG_FMT_M, (unsigned)copy[i].id, copy[i].x_mm, copy[i].y_mm);
    }
}

int hub_nav_tag_count(void)
{
    return s_n.core.n_tags;
}

esp_err_t hub_nav_tag_cmd(const char *arg)
{
    ESP_RETURN_ON_FALSE(s_n.ready, ESP_ERR_INVALID_STATE, TAG, "filtr nie dziala");
    flora_navcore_t *c = &s_n.core;

    if (strcmp(arg, "clear") == 0) {
        flora_navcore_tag_t old[HUB_NAV_MAX_TAGS];
        xSemaphoreTake(s_n.lock, portMAX_DELAY);
        int n = c->n_tags;
        memcpy(old, c->tags, sizeof old);
        c->n_tags = 0;
        esp_err_t err = tags_save();
        xSemaphoreGive(s_n.lock);
        for (int i = 0; i < n; i++) tag_publish_one(NULL, old[i].id);
        hub_params_publish();
        hub_nav_rec_tags();
        return err;
    }

    char *end;
    unsigned long id = strtoul(arg, &end, 10);
    if (end == arg || *end != ' ' || id > 586) {   /* tag36h11 ma 587 kodow */
        return ESP_ERR_INVALID_ARG;
    }
    const char *p = end + 1;
    bool del = strcmp(p, "-") == 0;
    float x = 0.0f, y = 0.0f;
    if (!del) {
        x = strtof(p, &end);
        if (end == p || *end != ' ' || !isfinite(x) || fabsf(x) > 1e6f) return ESP_ERR_INVALID_ARG;
        p = end + 1;
        y = strtof(p, &end);
        if (end == p || *end || !isfinite(y) || fabsf(y) > 1e6f) return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = ESP_OK;
    flora_navcore_tag_t saved = { 0 };
    xSemaphoreTake(s_n.lock, portMAX_DELAY);
    flora_navcore_tag_t *t = (flora_navcore_tag_t *)flora_navcore_tag(c, (uint16_t)id);
    if (del) {
        if (!t) {
            err = ESP_ERR_NOT_FOUND;
        } else {
            *t = c->tags[--c->n_tags];   /* ostatni na miejsce usunietego */
        }
    } else if (!t && c->n_tags >= HUB_NAV_MAX_TAGS) {
        err = ESP_ERR_NO_MEM;
    } else {
        if (!t) {
            t = &c->tags[c->n_tags++];
        }
        *t = (flora_navcore_tag_t){ .id = (uint16_t)id, .x_mm = x, .y_mm = y };
        saved = *t;
    }
    if (err == ESP_OK) {
        err = tags_save();
    }
    xSemaphoreGive(s_n.lock);

    if (err == ESP_OK) {
        if (del) {
            ESP_LOGI(TAG, "palik %lu usuniety", id);
        } else {
            ESP_LOGI(TAG, "palik %lu w (%.0f, %.0f) mm", id, x, y);
        }
        tag_publish_one(del ? NULL : &saved, (uint16_t)id);
        hub_params_publish();   /* liczba palikow w config_nav */
        hub_nav_rec_tags();
    }
    return err;
}

/* ================= START I ODCZYT ================= */

esp_err_t hub_nav_start(void)
{
    ESP_RETURN_ON_FALSE(!s_n.ready, ESP_ERR_INVALID_STATE, TAG, "juz dziala");
    s_n.lock = xSemaphoreCreateMutex();
    ESP_RETURN_ON_FALSE(s_n.lock, ESP_ERR_NO_MEM, TAG, "brak pamieci na mutex");

    hub_params_t p;
    hub_params_get(&p);
    flora_navcore_tuning_t tun = FLORA_NAVCORE_DEFAULT_TUNING();
    tun.k_wheel = FLORA_NAV_K_WHEEL;
    flora_navcore_geom_t g = geom_from(&p);
    flora_navcore_init(&s_n.core, &tun, &g);
    tags_load();
    pose_restore();

    /* Czas UTC tylko dla kompasu slonecznego. Bez sieci SNTP ponawia
     * w tle, a slonce jest po prostu pomijane (bramka 2). */
    esp_sntp_config_t sntp = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
    if (esp_netif_sntp_init(&sntp) != ESP_OK) {
        ESP_LOGW(TAG, "SNTP nie wystartowal - kompas sloneczny wylaczony");
    }

    if (hub_rec_init() != ESP_OK) {
        ESP_LOGW(TAG, "rekorder nie wystartowal - cmd/rec niedostepne");
    }

    s_n.ready = true;
    BaseType_t ok = xTaskCreate(nav_task, "nav", 5120, NULL, 5, NULL);
    if (ok != pdPASS) {
        s_n.ready = false;
        ESP_LOGE(TAG, "zadanie nav");
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "EKF pozycji: %d wariantow, rozstaw %.1f mm, k kola %.2f mm^2/mm, "
                  "kamera (%.0f, %.0f) mm, palikow %d",
             FLORA_NAVCORE_VARIANTS, g.track_mm, tun.k_wheel, g.cam_x_mm, g.cam_y_mm,
             s_n.core.n_tags);
    return ESP_OK;
}

bool hub_nav_ready(void)
{
    return s_n.ready;
}

int hub_nav_variant_count(void)
{
    return FLORA_NAVCORE_VARIANTS;
}

bool hub_nav_get(int variant, hub_nav_pose_t *out)
{
    memset(out, 0, sizeof(*out));
    if (!s_n.ready || variant < 0 || variant >= FLORA_NAVCORE_VARIANTS) {
        return false;
    }
    xSemaphoreTake(s_n.lock, portMAX_DELAY);
    const flora_navcore_variant_t *v = &s_n.core.v[variant];
    const flora_ekf_t *e = &v->ekf;
    out->name    = v->name;
    out->src     = v->src;
    out->x_mm    = e->x[FLORA_EKF_X];
    out->y_mm    = e->x[FLORA_EKF_Y];
    out->th_deg  = R2D(e->x[FLORA_EKF_TH]);
    float b = R2D(e->x[FLORA_EKF_B]);
    out->b_deg   = b < 0.0f ? b + 360.0f : b;
    out->sx_mm   = sqrtf(e->P[FLORA_EKF_X][FLORA_EKF_X]);
    out->sy_mm   = sqrtf(e->P[FLORA_EKF_Y][FLORA_EKF_Y]);
    out->sth_deg = R2D(sqrtf(e->P[FLORA_EKF_TH][FLORA_EKF_TH]));
    out->spos_mm = flora_ekf_pos_sigma_max(e);
    out->n_head  = v->n_head;
    out->n_sun   = v->n_sun;
    out->n_tag   = v->n_tag;
    out->n_rej   = e->n_rejected;
    out->n_rec   = e->n_recovered;
    xSemaphoreGive(s_n.lock);
    return true;
}

void hub_nav_status(hub_nav_status_t *out)
{
    memset(out, 0, sizeof(*out));
    out->sun_gate = FLORA_SUNGATE_NONE;
    if (!s_n.ready) {
        return;
    }
    time_t now;
    out->time_ok = time_ok(&now);
    xSemaphoreTake(s_n.lock, portMAX_DELAY);
    const flora_navcore_t *c = &s_n.core;
    out->sun_gate        = c->sun_gate;
    out->sun_diff_deg    = c->sun_diff_deg;
    out->n_sun_seen      = c->n_sun_seen;
    out->n_sun_gated     = c->n_sun_gated;
    out->att_age_ms      = c->att_age_ms;
    out->gyro_ok         = c->gyro_ok;
    out->gyro_vs_odo_dps = c->gyro_vs_odo_dps;
    xSemaphoreGive(s_n.lock);
}

esp_err_t hub_nav_reset_pose(float x_mm, float y_mm, float th_deg)
{
    ESP_RETURN_ON_FALSE(s_n.ready, ESP_ERR_INVALID_STATE, TAG, "filtr nie dziala");
    /* Wykona nav_task w nastepnym takcie (najwyzej 20 ms). */
    xSemaphoreTake(s_n.lock, portMAX_DELAY);
    s_n.reset_x = x_mm;
    s_n.reset_y = y_mm;
    s_n.reset_th = th_deg;
    s_n.pend_reset = true;
    xSemaphoreGive(s_n.lock);
    return ESP_OK;
}
