#include <inttypes.h>
#include <math.h>
#include <string.h>

#include "head_slow.h"
#include "head_cmd.h"
#include "head_fusion.h"
#include "head_math.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "flora_head_board.h"
#include "flora_msgs.h"

static const char *TAG = "slow";

#define TICK_MS          1000
#define STATUS_DIVIDER   5     /**< HEAD_STATUS 0,2 Hz */
#define WEATHER_DIVIDER  10    /**< WEATHER 0,1 Hz */

/* Po tylu bledach z rzedu czujnik przechodzi do DEAD. Trzy, bo jeden
 * zgubiony odczyt to kolizja na magistrali, a nie awaria. */
#define DEAD_THRESHOLD   3
/* Jak czesto probowac czujnik w stanie DEAD albo ABSENT. Wychwytuje tez
 * czujnik wpiety po starcie. */
#define RETRY_TICKS      10

/* --- kompas sloneczny --- */
#define SUN_MIN_LUX       100.0f  /**< ponizej uznajemy, ze za ciemno */
#define SUN_MIN_CONTRAST  0.15f   /**< ponizej uznajemy pelne zachmurzenie */
#define SUN_RAW_HIGH      60000   /**< blisko nasycenia - zmniejsz czulosc */
#define SUN_RAW_LOW       1000    /**< ciemno - zwieksz czulosc */
#define SUN_GAIN_HOLD_TICKS 2     /**< przerwa po zmianie, by nie oscylowalo */

/** Nastawy czulosci od najmniej do najbardziej czulej. */
static const struct { flora_veml_gain_t g; flora_veml_it_t it; const char *name; }
SUN_STEPS[] = {
    { FLORA_VEML_GAIN_1_8, FLORA_VEML_IT_25MS,  "1/8 x 25ms"  },
    { FLORA_VEML_GAIN_1_4, FLORA_VEML_IT_25MS,  "1/4 x 25ms"  },
    { FLORA_VEML_GAIN_1,   FLORA_VEML_IT_50MS,  "1 x 50ms"    },
    { FLORA_VEML_GAIN_2,   FLORA_VEML_IT_100MS, "2 x 100ms"   },
};
#define SUN_STEP_COUNT ((int)(sizeof(SUN_STEPS)/sizeof(SUN_STEPS[0])))

typedef struct {
    head_sensor_state_t state;
    uint8_t  fail_streak;
    uint32_t ok_count;
    uint32_t fail_count;
    uint8_t  bit;          /**< FLORA_SENSOR_* */
    const char *name;
} health_t;

static struct {
    head_slow_cfg_t cfg;
    uint32_t tick;

    health_t h_sun[4];
    health_t h_bme;

    int   sun_step;
    int   sun_hold;
    bool  sun_auto;

    uint16_t errors;
} s_s;

static const char *SUN_NAME[4] = { "VEML_N", "VEML_E", "VEML_S", "VEML_W" };
static const uint8_t SUN_BIT[4] = {
    FLORA_SENSOR_VEML_N, FLORA_SENSOR_VEML_E,
    FLORA_SENSOR_VEML_S, FLORA_SENSOR_VEML_W,
};

const char *head_sensor_state_name(head_sensor_state_t s)
{
    switch (s) {
        case HEAD_SENSOR_ABSENT:  return "brak";
        case HEAD_SENSOR_OK:      return "ok";
        case HEAD_SENSOR_SUSPECT: return "podejrzany";
        case HEAD_SENSOR_DEAD:    return "martwy";
        default:                  return "?";
    }
}

/**
 * Aktualizuje stan zdrowia czujnika.
 *
 * Po co osobny stan SUSPECT: na nowej plytce w maszcie realna awaria to
 * luzny styk pod kopula, czyli czujnik MIGOTLIWY, nie trwale martwy.
 * Bez tego stanu migotanie objawialoby sie tylko brakiem danych, bez sladu
 * w logu - a wlasnie slad jest tu najcenniejszy.
 */
static void health_update(health_t *h, bool ok)
{
    head_sensor_state_t before = h->state;

    if (ok) {
        h->ok_count++;
        h->fail_streak = 0;
        h->state = HEAD_SENSOR_OK;
    } else {
        h->fail_count++;
        s_s.errors++;
        if (h->fail_streak < 255) {
            h->fail_streak++;
        }
        if (h->state == HEAD_SENSOR_ABSENT) {
            /* nigdy nie odpowiedzial - zostaje jak byl */
        } else if (h->fail_streak >= DEAD_THRESHOLD) {
            h->state = HEAD_SENSOR_DEAD;
        } else {
            h->state = HEAD_SENSOR_SUSPECT;
        }
    }

    if (h->state != before) {
        if (h->state == HEAD_SENSOR_OK && before != HEAD_SENSOR_ABSENT) {
            ESP_LOGW(TAG, ">>> %s wrocil (po %" PRIu32 " bledach)",
                     h->name, h->fail_count);
        } else if (h->state == HEAD_SENSOR_DEAD) {
            ESP_LOGE(TAG, ">>> %s PRZESTAL odpowiadac (po %" PRIu32 " udanych)",
                     h->name, h->ok_count);
        } else {
            ESP_LOGI(TAG, "%s: %s -> %s", h->name,
                     head_sensor_state_name(before), head_sensor_state_name(h->state));
        }
    }
}

/** Czy czujnik ma byc odpytany w tym ticku. */
static bool should_poll(const health_t *h)
{
    if (h->state == HEAD_SENSOR_DEAD || h->state == HEAD_SENSOR_ABSENT) {
        return s_s.tick % RETRY_TICKS == 0;
    }
    return true;
}

/* ================= KOMPAS SLONECZNY ================= */

static void sun_apply_step(int step)
{
    if (step < 0) step = 0;
    if (step >= SUN_STEP_COUNT) step = SUN_STEP_COUNT - 1;
    if (step == s_s.sun_step) {
        return;
    }
    for (int i = 0; i < 4; i++) {
        if (s_s.cfg.sun[i]) {
            flora_veml7700_set_sensitivity(s_s.cfg.sun[i],
                                           SUN_STEPS[step].g, SUN_STEPS[step].it);
        }
    }
    ESP_LOGI(TAG, "czulosc kompasu: %s", SUN_STEPS[step].name);
    s_s.sun_step = step;
    s_s.sun_hold = SUN_GAIN_HOLD_TICKS;
}

static void do_sun(void)
{
    float    lux[4] = { 0 };
    uint16_t raw[4] = { 0 };
    int got = 0;

    /* Cztery kanaly czytamy JEDEN PO DRUGIM, lacznie okolo 1,8 ms.
     * Rozlozenie ich w czasie zepsuloby porownanie, bo robot sie obraca -
     * a to wlasnie rozrzut miedzy kanalami nosi informacje o kierunku. */
    for (int i = 0; i < 4; i++) {
        if (!s_s.cfg.sun[i] || !should_poll(&s_s.h_sun[i])) {
            continue;
        }
        bool ok = flora_veml7700_read_raw(s_s.cfg.sun[i], &raw[i]) == ESP_OK &&
                  flora_veml7700_read_lux(s_s.cfg.sun[i], &lux[i]) == ESP_OK;
        health_update(&s_s.h_sun[i], ok);
        if (ok) {
            got++;
        }
    }

    /* Automatyczny dobor czulosci. Kompas pracuje od nocy do pelnego slonca,
     * czyli przez pare rzedow wielkosci - przy nasyceniu wszystkie cztery
     * kanaly pokazuja maksimum i kompas przestaje rozrozniac kierunki. */
    if (s_s.sun_auto && got == 4) {
        if (s_s.sun_hold > 0) {
            s_s.sun_hold--;
        } else {
            uint16_t hi = 0, lo = 0xFFFF;
            for (int i = 0; i < 4; i++) {
                if (raw[i] > hi) hi = raw[i];
                if (raw[i] < lo) lo = raw[i];
            }
            if (hi >= SUN_RAW_HIGH) {
                sun_apply_step(s_s.sun_step - 1);   /* mniej czule */
                return;                             /* odczyt nasycony - nie wysylamy */
            } else if (hi <= SUN_RAW_LOW) {
                sun_apply_step(s_s.sun_step + 1);   /* bardziej czule */
            }
        }
    }

    if (got < 2) {
        return;   /* mniej niz dwa kanaly - kierunku nie ma z czego policzyc */
    }

    head_sun_t sun = head_sun_azimuth(lux, SUN_MIN_LUX, SUN_MIN_CONTRAST);

    flora_sun_t m = {
        .timestamp_us   = (uint32_t)esp_timer_get_time(),
        .azimuth_cdeg   = (uint16_t)lroundf(sun.azimuth_deg * 100.0f),
        .elevation_cdeg = 0,     /* wysokosci nie szacujemy - krzyz patrzy w boki */
        .valid          = sun.valid ? 1 : 0,
    };
    for (int i = 0; i < 4; i++) {
        /* Dzielimy przez 4, zeby pelne slonce (ponad 100 000 lx) zmiescilo
         * sie w uint16 - zakres siega wtedy 262 140 lx. */
        float v = lux[i] / 4.0f;
        m.lux_div4[i] = (uint16_t)(v > 65535.0f ? 65535.0f : v);
    }
    flora_link_send(s_s.cfg.down, FLORA_MSG_SUN, &m, sizeof m);
}

/* ================= POGODA ================= */

static void do_weather(void)
{
    if (!s_s.cfg.bme || !should_poll(&s_s.h_bme)) {
        return;
    }
    flora_bme280_data_t d;
    bool ok = flora_bme280_read(s_s.cfg.bme, &d) == ESP_OK;
    health_update(&s_s.h_bme, ok);
    if (!ok) {
        return;
    }

    flora_weather_t m = {
        .timestamp_us  = (uint32_t)esp_timer_get_time(),
        .pressure_pa   = (uint32_t)lroundf(d.pressure_pa),
        .temp_cc       = (int16_t)lroundf(d.temperature_c * 100.0f),
        .humidity_cpct = isnan(d.humidity_pct) ? 0
                         : (uint16_t)lroundf(d.humidity_pct * 100.0f),
        .rain          = 0,   /* zarezerwowane, czujnika deszczu nie bedzie */
    };
    flora_link_send(s_s.cfg.down, FLORA_MSG_WEATHER, &m, sizeof m);
}

/* ================= STAN ================= */

uint8_t head_slow_sensors_ok(void)
{
    uint8_t mask = 0;
    for (int i = 0; i < 4; i++) {
        if (s_s.h_sun[i].state == HEAD_SENSOR_OK ||
            s_s.h_sun[i].state == HEAD_SENSOR_SUSPECT) {
            mask |= s_s.h_sun[i].bit;
        }
    }
    if (s_s.h_bme.state == HEAD_SENSOR_OK || s_s.h_bme.state == HEAD_SENSOR_SUSPECT) {
        mask |= FLORA_SENSOR_BME280;
    }
    if (head_fusion_state() == HEAD_FUSION_RUN) {
        mask |= FLORA_SENSOR_IMU;
        if (head_fusion_heading_valid()) {
            mask |= FLORA_SENSOR_MAG;
        }
    }
    return mask;
}

uint16_t head_slow_errors(void) { return s_s.errors; }

static void do_status(void)
{
    flora_head_status_t m = {
        .uptime_ms   = (uint32_t)(esp_timer_get_time() / 1000),
        .sensors_ok  = head_slow_sensors_ok(),
        .error_count = (uint16_t)(s_s.errors + head_fusion_errors()),
    };
    flora_link_send(s_s.cfg.down, FLORA_MSG_HEAD_STATUS, &m, sizeof m);

    ESP_LOGI(TAG, "kurs %.1f st (%s) | czujniki 0x%02X | bledy %u",
             head_fusion_heading(),
             head_fusion_state_name(head_fusion_state()),
             m.sensors_ok, m.error_count);
}

/* ================= PETLA ================= */

static void slow_task(void *arg)
{
    TickType_t next = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&next, pdMS_TO_TICKS(TICK_MS));
        s_s.tick++;

        uint8_t gain;
        if (head_cmd_take_sun_gain(&gain)) {
            s_s.sun_auto = false;
            sun_apply_step((int)gain);
            ESP_LOGI(TAG, "czulosc wymuszona komenda, automat wylaczony");
        }

        do_sun();
        if (s_s.tick % WEATHER_DIVIDER == 0) {
            do_weather();
        }
        if (s_s.tick % STATUS_DIVIDER == 0) {
            do_status();
        }
    }
}

esp_err_t head_slow_start(const head_slow_cfg_t *cfg)
{
    ESP_RETURN_ON_FALSE(cfg && cfg->down, ESP_ERR_INVALID_ARG, TAG, "zly argument");

    memset(&s_s, 0, sizeof(s_s));
    s_s.cfg      = *cfg;
    s_s.sun_auto = true;
    s_s.sun_step = 0;

    for (int i = 0; i < 4; i++) {
        s_s.h_sun[i].name  = SUN_NAME[i];
        s_s.h_sun[i].bit   = SUN_BIT[i];
        s_s.h_sun[i].state = cfg->sun[i] ? HEAD_SENSOR_OK : HEAD_SENSOR_ABSENT;
    }
    s_s.h_bme.name  = "BME280";
    s_s.h_bme.bit   = FLORA_SENSOR_BME280;
    s_s.h_bme.state = cfg->bme ? HEAD_SENSOR_OK : HEAD_SENSOR_ABSENT;

    ESP_RETURN_ON_FALSE(xTaskCreate(slow_task, "head_slow", 4096, NULL, 5, NULL) == pdPASS,
                        ESP_ERR_NO_MEM, TAG, "nie udalo sie utworzyc zadania");
    ESP_LOGI(TAG, "zadanie 1 Hz: kompas 1 Hz, pogoda 0,1 Hz, stan 0,2 Hz");
    return ESP_OK;
}
