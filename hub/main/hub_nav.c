/**
 * @file hub_nav.c
 * @brief Petla EKF pozycji (kilka wariantow rownolegle), opis w hub_nav.h.
 *
 * Zadanie 50 Hz. Wiadomosci glowicy czyta z buforow hub_links (ostatnia
 * wartosc + numer kopii), wiec nie konkuruje z telemetria i nie blokuje
 * zadania odbiorczego lacza. Pomiary sa przygotowywane RAZ na takt,
 * a potem podawane kazdemu wariantowi, ktory ma dane zrodlo wlaczone.
 */
#include <inttypes.h>
#include <math.h>
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
#include "flora_board.h"
#include "flora_ekf.h"
#include "flora_encoders.h"
#include "flora_msgs.h"
#include "flora_sungate.h"

static const char *TAG = "nav";

#define LOOP_MS          20
#define HEAD_EVERY_US    500000   /**< kurs z glowicy co 0,5 s, patrz hub_nav.h */
#define STILL_TICKS      25       /**< 0,5 s bez ruchu = postoj */
#define STILL_MM         1.0f
#define MAX_STEP_MM      100.0f   /**< 5 m/s - wiecej to skok licznika, nie ruch */
#define TIME_VALID_UNIX  1735689600LL   /**< 2025-01-01: wczesniej zegar nie byl ustawiony */

#define GYRO_MAX_AGE_US     150000   /**< rate_z starsze niz to nie opisuje biezacego taktu */
#define STILL_EVERY_US      1000000  /**< odswiezenie flagi postoju dla glowicy */
/** Podloga niepewnosci kursu z glowicy. Bledy magnetometru zaleza od
 *  kierunku i sa skorelowane w czasie, a EKF bierze kolejne pomiary jako
 *  niezalezne - bez podlogi P theta zeszloby ponizej realnej dokladnosci. */
#define HEAD_SIGMA_FLOOR_DEG 4.0f

#define SUN_MAG_MAX_AGE_US  2000000  /**< kurs glowicy starszy niz to nie jest odniesieniem */
#define SUN_EVERY_US        5000000  /**< najwyzej jeden pomiar slonca na 5 s */

/* --- model bez odometrii (wariant no_odom) --- */
#define STATIC_Q_XY  1.0e4f                  /**< (100 mm/s)^2 - predkosc robota */
#define STATIC_Q_TH  (0.5236f * 0.5236f)     /**< (30 st/s)^2 - obrot w miejscu */

#define D2R(d) ((d) * (float)M_PI / 180.0f)
#define R2D(r) ((r) * 180.0f / (float)M_PI)

/* ---- mapa palikow ----
 * Polozenia znacznikow w ukladzie mapy [mm]. Na razie pusta: obszar
 * roboczy i rozmieszczenie palikow nie sa jeszcze ustalone.
 * TODO: przeniesc do NVS i cmd/config, gdy bedzie znany obszar. */
typedef struct {
    uint16_t id;
    float    x_mm, y_mm;
} tag_pos_t;

#define TAG_UNUSED 0xFFFF
static const tag_pos_t s_tag_map[] = {
    /* { .id = 0, .x_mm = 0.0f,    .y_mm = 0.0f    }, */
    { .id = TAG_UNUSED },
};

static const tag_pos_t *tag_lookup(uint16_t id)
{
    for (size_t i = 0; i < sizeof s_tag_map / sizeof s_tag_map[0]; i++) {
        if (s_tag_map[i].id != TAG_UNUSED && s_tag_map[i].id == id) {
            return &s_tag_map[i];
        }
    }
    return NULL;
}

/* ---- warianty ----
 * Dopisanie kombinacji = jedna linijka. Wariant 0 to glowny wynik
 * (tele/pose), pozostale ida na tele/pose_<nazwa>. Nazwa trafia do tematu,
 * a temat do nazwy measurementu w Telegrafie - bez spacji i ukosnikow. */
typedef struct {
    const char *name;
    uint8_t     src;
    flora_ekf_t ekf;
    uint32_t    n_head, n_sun, n_tag;
} variant_t;

#define ODOM HUB_NAV_SRC_ODOM
#define HEAD HUB_NAV_SRC_HEAD
#define SUN  HUB_NAV_SRC_SUN
#define TAGS HUB_NAV_SRC_TAGS
static variant_t s_variants[] = {
    { .name = "full",      .src = ODOM | HEAD | SUN | TAGS },
    { .name = "odom",      .src = ODOM },
    { .name = "head",      .src = ODOM | HEAD },
    { .name = "head_sun",  .src = ODOM | HEAD | SUN },
    { .name = "odom_tags", .src = ODOM | TAGS },
    { .name = "no_odom",   .src = HEAD | SUN | TAGS },
};
#define VARIANTS ((int)(sizeof s_variants / sizeof s_variants[0]))

static struct {
    SemaphoreHandle_t lock;
    bool              ready;

    int32_t  att_age_ms;      /**< wiek ostatniej probki kursu; -1 bez TIME_SYNC */

    /* ostatni kurs z glowicy, do bramki zgodnosci slonca */
    float    head_psi;
    bool     head_mag;
    int64_t  head_us;

    flora_sungate_t        gate;
    flora_sungate_result_t sun_gate;
    float    sun_diff_deg;
    uint32_t n_sun_seen, n_sun_gated;

    /* ostatnie ATTITUDE: rate_z do oceny odometrii */
    flora_attitude_t att;
    int64_t  att_rx_us;
    bool     have_att;
    float    gyro_vs_odo_dps;   /**< wygladzona roznica obrotu kola - zyro */
    bool     gyro_ok;
} s_n = { .sun_gate = FLORA_SUNGATE_NONE, .att_age_ms = -1 };

/** Pomiar kursu przygotowany dla wariantow w biezacym takcie. */
typedef struct {
    bool  valid;
    float psi, sigma;
} heading_meas_t;

/**
 * Zapasowa niepewnosc kursu z `quality`, gdy glowica nie podala sigma.
 * 90 (z magnetometrem) -> 4 st, 50 (sam zyroskop) -> 16 st.
 */
static float head_sigma_rad(uint8_t quality)
{
    float q = quality > 100 ? 100.0f : (float)quality;
    float deg = 4.0f + (90.0f - q) * 0.3f;
    if (deg < 3.0f)  deg = 3.0f;
    if (deg > 45.0f) deg = 45.0f;
    return D2R(deg);
}

static bool time_ok(time_t *now)
{
    time(now);
    return (long long)*now > TIME_VALID_UNIX;
}

/** @param fresh  nowa probka ATTITUDE w tym takcie (w s_n.att) */
static heading_meas_t take_heading(int64_t now_us, bool fresh, int64_t *last_us)
{
    heading_meas_t m = { 0 };
    const flora_attitude_t att = s_n.att;
    if (!fresh || !(att.flags & FLORA_ATT_HEADING_VALID)) {
        return m;
    }
    float psi_deg = att.heading_cdeg / 100.0f;

    /* Wiek probki: przy TIME_SYNC znacznik jest w naszym zegarze (mlodsze
     * 32 bity, odejmowanie modulo 2^32). Kurs przesuwamy o obrot od chwili
     * pomiaru do teraz - rate_z ma ten sam znak co kurs glowicy. */
    s_n.att_age_ms = -1;
    if (att.flags & FLORA_ATT_TIME_SYNCED) {
        int32_t age_us = (int32_t)((uint32_t)esp_timer_get_time() - att.timestamp_us);
        if (age_us >= 0 && age_us < 500000) {
            s_n.att_age_ms = age_us / 1000;
            psi_deg += (att.rate_z_cdps / 100.0f) * (age_us / 1e6f);
        }
    }
    float psi = D2R(psi_deg);

    /* Kazda probka odswieza odniesienie dla bramki slonca, niezaleznie
     * od tego, czy trafi do EKF. */
    s_n.head_psi = psi;
    s_n.head_mag = (att.flags & FLORA_ATT_MAG_VALID) != 0;
    s_n.head_us  = now_us;

    if (now_us - *last_us < HEAD_EVERY_US) {
        return m;
    }
    *last_us = now_us;
    m.valid = true;
    m.psi   = psi;
    /* Glowica (kontrakt v4) wysyla niepewnosc z Kalmana; quality zostaje
     * tylko jako zapas, gdyby pole bylo zerem. */
    m.sigma = att.heading_sigma_cdeg > 0 ? D2R(att.heading_sigma_cdeg / 100.0f)
                                         : head_sigma_rad(att.quality);
    if (m.sigma < D2R(HEAD_SIGMA_FLOOR_DEG)) {
        m.sigma = D2R(HEAD_SIGMA_FLOOR_DEG);
    }
    return m;
}

static heading_meas_t take_sun(int64_t now_us, uint32_t *seq, int64_t *last_us)
{
    heading_meas_t m = { 0 };
    flora_sun_t sun;
    if (!hub_links_get(FLORA_MSG_SUN, &sun, sizeof sun, seq) || !sun.valid) {
        return m;
    }
    s_n.n_sun_seen++;

    time_t now;
    flora_sungate_in_t in = {
        .azimuth_meas_deg = sun.azimuth_cdeg / 100.0f,
        .unix_s           = time_ok(&now) ? (int64_t)now : 0,
        .th_ref_rad       = s_variants[0].ekf.x[FLORA_EKF_TH],
        .head_mag_ok      = s_n.head_mag && now_us - s_n.head_us < SUN_MAG_MAX_AGE_US,
        .head_psi_rad     = s_n.head_psi,
    };
    for (int i = 0; i < 4; i++) {
        in.lux[i] = sun.lux_div4[i] * 4.0f;
    }
    /* ZALOZENIE DO SPRAWDZENIA: kanal N krzyza i os odniesienia kursu IMU
     * patrza w te sama strone glowicy. Jesli nie, wyjdzie staly konflikt
     * z magnetometrem (sun_vs_mag_deg w tele/nav_sun) do odjecia tutaj. */
    flora_sungate_out_t out;
    flora_sungate_result_t g = flora_sungate_eval(&s_n.gate, &in, &out);
    if (out.vs_mag_deg != 0.0f) {
        s_n.sun_diff_deg = out.vs_mag_deg;
    }

    if (g != s_n.sun_gate) {
        /* Tylko przejscia, zeby log nie puchl co sekunde. UNSTABLE zaraz
         * po DIM/LOW to normalne zbieranie probek - nie ostrzegamy. */
        if (g == FLORA_SUNGATE_OK) {
            ESP_LOGI(TAG, "kompas sloneczny: pomiary przyjmowane");
        } else if (g != FLORA_SUNGATE_UNSTABLE || s_n.sun_gate == FLORA_SUNGATE_OK) {
            ESP_LOGW(TAG, "kompas sloneczny pominiety: %s", flora_sungate_name(g));
        }
    }
    s_n.sun_gate = g;
    if (g != FLORA_SUNGATE_OK) {
        s_n.n_sun_gated++;
        return m;
    }
    if (now_us - *last_us < SUN_EVERY_US) {
        return m;
    }
    *last_us = now_us;
    m.valid = true;
    m.psi   = out.psi_rad;
    m.sigma = out.sigma_rad;
    return m;
}

static void apply_tags(variant_t *v, const flora_tags_t *msg)
{
    uint8_t n = msg->count > FLORA_TAGS_MAX ? FLORA_TAGS_MAX : msg->count;
    for (uint8_t i = 0; i < n; i++) {
        const flora_tag_t *t = &msg->tags[i];
        const tag_pos_t *pos = tag_lookup(t->id);
        if (!pos || t->distance_mm == 0) {
            continue;   /* nieznany palik albo brak kalibracji ogniskowej */
        }
        /* Gorsza detekcja = szerszy rozrzut: quality 100 -> x1, 50 -> x2. */
        float q = t->quality < 10 ? 10.0f : (float)t->quality;
        float k = 1.0f + (100.0f - q) / 50.0f;
        float d = (float)t->distance_mm;
        /* flora_tags: kat dodatni W PRAWO; EKF: CCW, czyli w lewo. */
        float beta = -D2R(t->bearing_cdeg / 100.0f);
        if (flora_ekf_update_tag(&v->ekf, pos->x_mm, pos->y_mm,
                                 FLORA_NAV_CAM_X_MM, FLORA_NAV_CAM_Y_MM,
                                 d, beta, (20.0f + 0.02f * d) * k, D2R(1.5f) * k)) {
            v->n_tag++;
        }
    }
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
    int64_t last_head_us = 0, last_sun_us = 0;
    float moved[STILL_TICKS] = {0};
    int moved_i = 0;

    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(LOOP_MS));

        flora_encoders_sample_t now;
        if (flora_encoders_read(&now) != ESP_OK) {
            continue;
        }
        if (!have_prev) {
            prev = now;
            have_prev = true;
            continue;
        }
        /* Przyrosty z calkowitych zliczen, nie z narastajacych mm w float:
         * po kilku km float gubi milimetry, a przyrost na takt to ~2 mm. */
        float dl = (float)(now.left_counts  - prev.left_counts)  * FLORA_MM_PER_COUNT;
        float dr = (float)(now.right_counts - prev.right_counts) * FLORA_MM_PER_COUNT;
        float dt = (float)(now.timestamp_us - prev.timestamp_us) / 1e6f;
        prev = now;
        if (fabsf(dl) > MAX_STEP_MM || fabsf(dr) > MAX_STEP_MM) {
            ESP_LOGW(TAG, "skok enkoderow (%.0f / %.0f mm) - pomijam krok", dl, dr);
            continue;
        }

        moved[moved_i] = fabsf(dl) + fabsf(dr);
        moved_i = (moved_i + 1) % STILL_TICKS;
        float recent = 0.0f;
        for (int i = 0; i < STILL_TICKS; i++) recent += moved[i];
        bool still = recent < STILL_MM;

        xSemaphoreTake(s_n.lock, portMAX_DELAY);

        /* ATTITUDE raz na takt: kurs (co 0,5 s) i rate_z (w kazdym takcie). */
        flora_attitude_t att;
        bool att_fresh = hub_links_get(FLORA_MSG_ATTITUDE, &att, sizeof att, &seq_att);
        if (att_fresh) {
            s_n.att       = att;
            s_n.att_rx_us = now.timestamp_us;
            s_n.have_att  = true;
        }
        /* Obrot z zyroskopu w ukladzie mapy: rate_z to tempo kursu
         * kompasowego (CW), theta jest CCW - stad minus. */
        s_n.gyro_ok = s_n.have_att &&
                      (s_n.att.flags & FLORA_ATT_HEADING_VALID) &&
                      now.timestamp_us - s_n.att_rx_us < GYRO_MAX_AGE_US;
        float dth_gyro = s_n.gyro_ok ? -D2R(s_n.att.rate_z_cdps / 100.0f) * dt : 0.0f;
        if (s_n.gyro_ok && dt > 0.0f) {
            float dth_odo = (dr - dl) / FLORA_TRACK_MM;
            float mis_dps = R2D(dth_odo - dth_gyro) / dt;
            s_n.gyro_vs_odo_dps += 0.05f * (mis_dps - s_n.gyro_vs_odo_dps);   /* ~0,4 s */
        }

        /* Pomiary raz na takt, wspolne dla wszystkich wariantow. */
        heading_meas_t head = take_heading(now.timestamp_us, att_fresh, &last_head_us);
        heading_meas_t sun  = take_sun(now.timestamp_us, &seq_sun, &last_sun_us);
        flora_tags_t tags;
        bool have_tags = hub_links_get(FLORA_MSG_TAGS, &tags, sizeof tags, &seq_tags) && still;

        for (int i = 0; i < VARIANTS; i++) {
            variant_t *v = &s_variants[i];
            if (v->src & HUB_NAV_SRC_ODOM) {
                flora_ekf_predict_odom(&v->ekf, dl, dr, dt);
                /* Zyroskop jest zrodlem glowicy - wariant "odom" zostaje czysty. */
                if ((v->src & HUB_NAV_SRC_HEAD) && s_n.gyro_ok) {
                    flora_ekf_observe_gyro(&v->ekf, dth_gyro);
                }
            } else {
                flora_ekf_predict_static(&v->ekf, dt, STATIC_Q_XY, STATIC_Q_TH);
            }
            if ((v->src & HUB_NAV_SRC_HEAD) && head.valid &&
                flora_ekf_update_heading(&v->ekf, head.psi, head.sigma)) {
                v->n_head++;
            }
            if ((v->src & HUB_NAV_SRC_SUN) && sun.valid &&
                flora_ekf_update_heading(&v->ekf, sun.psi, sun.sigma)) {
                v->n_sun++;
            }
            if ((v->src & HUB_NAV_SRC_TAGS) && have_tags) {
                apply_tags(v, &tags);
            }
        }
        xSemaphoreGive(s_n.lock);

        send_still(still, now.timestamp_us);
    }
}

esp_err_t hub_nav_start(void)
{
    ESP_RETURN_ON_FALSE(!s_n.ready, ESP_ERR_INVALID_STATE, TAG, "juz dziala");
    s_n.lock = xSemaphoreCreateMutex();
    ESP_RETURN_ON_FALSE(s_n.lock, ESP_ERR_NO_MEM, TAG, "brak pamieci na mutex");

    flora_ekf_cfg_t cfg = FLORA_EKF_DEFAULT_CFG();
    cfg.track_mm = FLORA_TRACK_MM;
    cfg.k_l = cfg.k_r = FLORA_NAV_K_WHEEL;
    for (int i = 0; i < VARIANTS; i++) {
        flora_ekf_init(&s_variants[i].ekf, &cfg);
    }

    flora_sungate_cfg_t gcfg = FLORA_SUNGATE_DEFAULT_CFG();
    gcfg.lat_deg         = FLORA_SITE_LAT_DEG;
    gcfg.lon_deg         = FLORA_SITE_LON_DEG;
    gcfg.declination_deg = FLORA_MAG_DECLINATION_DEG;
    flora_sungate_init(&s_n.gate, &gcfg);

    /* Czas UTC tylko dla kompasu slonecznego. Bez sieci SNTP ponawia
     * w tle, a slonce jest po prostu pomijane (bramka 2). */
    esp_sntp_config_t sntp = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
    if (esp_netif_sntp_init(&sntp) != ESP_OK) {
        ESP_LOGW(TAG, "SNTP nie wystartowal - kompas sloneczny wylaczony");
    }

    BaseType_t ok = xTaskCreate(nav_task, "nav", 4096, NULL, 5, NULL);
    ESP_RETURN_ON_FALSE(ok == pdPASS, ESP_ERR_NO_MEM, TAG, "zadanie nav");
    s_n.ready = true;
    ESP_LOGI(TAG, "EKF pozycji: %d wariantow, rozstaw %.0f mm, k kola %.2f mm^2/mm, kamera (%.0f, %.0f) mm",
             VARIANTS, cfg.track_mm, cfg.k_l, FLORA_NAV_CAM_X_MM, FLORA_NAV_CAM_Y_MM);
    return ESP_OK;
}

bool hub_nav_ready(void)
{
    return s_n.ready;
}

int hub_nav_variant_count(void)
{
    return VARIANTS;
}

bool hub_nav_get(int variant, hub_nav_pose_t *out)
{
    memset(out, 0, sizeof(*out));
    if (!s_n.ready || variant < 0 || variant >= VARIANTS) {
        return false;
    }
    xSemaphoreTake(s_n.lock, portMAX_DELAY);
    const variant_t *v = &s_variants[variant];
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
    out->sun_gate     = s_n.sun_gate;
    out->sun_diff_deg = s_n.sun_diff_deg;
    out->n_sun_seen   = s_n.n_sun_seen;
    out->n_sun_gated  = s_n.n_sun_gated;
    out->att_age_ms   = s_n.att_age_ms;
    out->gyro_ok      = s_n.gyro_ok;
    out->gyro_vs_odo_dps = s_n.gyro_vs_odo_dps;
    xSemaphoreGive(s_n.lock);
}

esp_err_t hub_nav_reset_pose(float x_mm, float y_mm, float th_deg)
{
    ESP_RETURN_ON_FALSE(s_n.ready, ESP_ERR_INVALID_STATE, TAG, "filtr nie dziala");
    xSemaphoreTake(s_n.lock, portMAX_DELAY);
    /* Mala, niezerowa niepewnosc: robot postawiony "na oko" w znanym miejscu.
     * b liczone od nowa: nowe theta ze starym b dawaloby kurs poza bramka. */
    for (int i = 0; i < VARIANTS; i++) {
        flora_ekf_set_pose(&s_variants[i].ekf, x_mm, y_mm, D2R(th_deg), 20.0f, D2R(2.0f), true);
    }
    flora_sungate_reset(&s_n.gate);   /* stare pomiary slonca liczone przy innym theta */
    xSemaphoreGive(s_n.lock);
    ESP_LOGI(TAG, "poza ustawiona we wszystkich wariantach: (%.0f, %.0f) mm, %.1f st",
             x_mm, y_mm, th_deg);
    return ESP_OK;
}
