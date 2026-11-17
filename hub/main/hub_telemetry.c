/**
 * @file hub_telemetry.c
 * @brief Publikacja stanu bryly i wiadomosci glowicy na MQTT.
 *
 * Konfiguracje sprzetu sa przeniesione z testow rozruchowych, ktore je
 * potwierdzily na sprzecie. Kierunki silnikow i enkoderow wspolne dla obu
 * miejsc: FLORA_*_INVERT w flora_board.h.
 *
 * Jedno zadanie, takt 100 ms, kazdy temat ma wlasny dzielnik. Payloady to
 * plaski JSON z jednostkami w nazwach pol, zeby dalo sie je pozniej wprost
 * zaciagnac do bazy szeregow czasowych (Telegraf, parser json).
 *
 * NAZWY POL TRUE/FALSE I TEKSTOWYCH MUSZA BYC UNIKALNE w calej telemetrii:
 * Telegraf odrzuca takie wartosci, chyba ze nazwa pola jest na liscie
 * json_string_fields w tools/telemetry/telegraf.conf - a lista dziala po
 * nazwie, wiec przy wspolnej nazwie z polem liczbowym jedno z nich
 * zapisaloby sie w zlym typie.
 * Nowe pole logiczne albo tekstowe = dopisz je tam.
 *
 * Publikujemy tylko przy polaczonym brokerze. Bez tego kazdy takt
 * produkowalby ostrzezenie flora_mqtt i log bylby nieczytelny.
 */
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"

#include "hub_telemetry.h"
#include "hub_config.h"
#include "hub_diag.h"
#include "hub_drive.h"
#include "hub_nav.h"
#include "hub_params.h"
#include "hub_sensorcal.h"
#include "hub_links.h"
#include "flora_board.h"
#include "flora_encoders.h"
#include "flora_level.h"
#include "flora_link.h"
#include "flora_motors.h"
#include "flora_mqtt.h"
#include "flora_msgs.h"
#include "flora_pump.h"
#include "flora_soil.h"
#include "flora_sonar.h"
#include "flora_store.h"
#include "flora_temp.h"

static const char *TAG = "tele";

#define TICK_MS 100

/* Przerwa miedzy dawkami na czas debugowania: krotsza niz docelowa minuta,
 * zeby on/off z panelu nie odbijal sie co chwile od min_interval_ms. */
#define DEBUG_PUMP_INTERVAL_MS 5000

static flora_ina226_handle_t s_ina;
static flora_level_handle_t  s_level;
static flora_pwrsw_handle_t  s_mast;

static bool s_drive_ok, s_pump_ok, s_soil_ok, s_sonar_ok, s_temp_ok;
static bool temp_init(void);
static void sonar_task(void *arg);
static SemaphoreHandle_t s_sonar_lock;
/** Odczyt ADC gleby z dwoch zadan (telemetria i hub_water) - po kolei. */
static SemaphoreHandle_t s_soil_lock;

/** Powod ostatniego zakonczenia pompki; -1 gdy jeszcze nie pracowala. */
static volatile int s_pump_last_reason = -1;
static volatile uint32_t s_pump_last_ms;

flora_pwrsw_handle_t hub_telemetry_mast(void) { return s_mast; }
flora_ina226_handle_t hub_telemetry_ina(void)  { return s_ina; }
uint32_t hub_telemetry_pump_last_ms(void)      { return s_pump_last_ms; }

/* ================= ZRODLA DLA STRAZNIKA UTYKU ================= */

static float bus_current(void *ctx)
{
    float a = 0.0f;
    return flora_ina226_read_current(s_ina, &a) == ESP_OK ? a : NAN;
}

static int64_t encoder_counts(flora_motors_wheel_t wheel, void *ctx)
{
    flora_encoders_sample_t s;
    if (flora_encoders_read(&s) != ESP_OK) {
        return 0;
    }
    return wheel == FLORA_MOTORS_WHEEL_LEFT ? s.left_counts : s.right_counts;
}

static void on_stall(flora_motors_stall_reason_t reason, flora_motors_wheel_t wheel,
                     float value, void *ctx)
{
    ESP_LOGE(TAG, "straznik odcial naped (%s, %.2f)",
             reason == FLORA_MOTORS_STALL_OVERCURRENT ? "prad" : "brak ruchu", value);
}

static void pump_done(flora_pump_done_reason_t reason, uint32_t ran_ms, void *ctx)
{
    /* Wolane z timera pompki - tylko zapamietujemy, publikuje zadanie. */
    s_pump_last_reason = (int)reason;
    s_pump_last_ms     = ran_ms;
}

/* ================= INICJALIZACJA SPRZETU ================= */

/** Konfiguracja z drive_hw_init() i enable_stall_guard() w bringup_drive.c. */
static void drive_init(void)
{
    flora_motors_cfg_t mcfg = FLORA_MOTORS_DEFAULT_CFG();
    mcfg.left  = (flora_motor_cfg_t){ .pwm_gpio = FLORA_PIN_MOTOR_L_PWM,
                                      .dir_gpio = FLORA_PIN_MOTOR_L_DIR,
                                      .invert   = FLORA_MOTOR_L_INVERT };
    mcfg.right = (flora_motor_cfg_t){ .pwm_gpio = FLORA_PIN_MOTOR_R_PWM,
                                      .dir_gpio = FLORA_PIN_MOTOR_R_DIR,
                                      .invert   = FLORA_MOTOR_R_INVERT };
    if (flora_motors_init(&mcfg) != ESP_OK) {
        ESP_LOGW(TAG, "naped niedostepny");
        return;
    }

    flora_encoders_cfg_t ecfg = FLORA_ENCODERS_DEFAULT_CFG();
    ecfg.left  = (flora_encoder_cfg_t){ .a_gpio = FLORA_PIN_ENC_L_A,
                                        .b_gpio = FLORA_PIN_ENC_L_B,
                                        .invert = FLORA_ENC_L_INVERT };
    ecfg.right = (flora_encoder_cfg_t){ .a_gpio = FLORA_PIN_ENC_R_A,
                                        .b_gpio = FLORA_PIN_ENC_R_B,
                                        .invert = FLORA_ENC_R_INVERT };
    /* Srednice kol z NVS (cmd/wheel_cal), osobno dla kazdego kola. */
    hub_params_load();
    hub_params_t hp;
    hub_params_get(&hp);
    float mm_l, mm_r;
    hub_params_mm_per_count(&hp, &mm_l, &mm_r);
    ecfg.mm_per_count = mm_l;
    if (flora_encoders_init(&ecfg) != ESP_OK) {
        /* Bez enkoderow straznik widzialby wieczny utyk - naped zostaje
         * wylaczony, zamiast jezdzic bez zabezpieczenia. */
        ESP_LOGW(TAG, "enkodery niedostepne - naped zablokowany");
        flora_motors_deinit();
        return;
    }

    flora_encoders_set_mm_per_count(mm_l, mm_r);

    flora_motors_stall_cfg_t guard = FLORA_MOTORS_STALL_DEFAULT_CFG();
    guard.counts_cb  = encoder_counts;
    guard.on_stall   = on_stall;
    guard.current_cb = s_ina ? bus_current : NULL;
    if (flora_motors_stall_guard_enable(&guard) != ESP_OK) {
        ESP_LOGW(TAG, "straznik utyku nie wystartowal - naped zablokowany");
        flora_encoders_deinit();
        flora_motors_deinit();
        return;
    }
    s_drive_ok = true;

    /* Petla predkosci startuje zawsze, ale w trybie pwm nic nie wystawia. */
    if (hub_drive_start() != ESP_OK) {
        ESP_LOGW(TAG, "petla predkosci nie wystartowala - zostaje jazda na surowej mocy");
    }
    /* Filtr pozycji potrzebuje tylko enkoderow; glowica i znaczniki
     * dochodza, gdy sie pojawia. */
    if (hub_nav_start() != ESP_OK) {
        ESP_LOGW(TAG, "filtr pozycji nie wystartowal");
    }
}

/** Wzorzec z test_pump() w bringup_hydro.c: poziom przez flora_level. */
static void pump_init(void)
{
    flora_level_cfg_t lvl = FLORA_LEVEL_DEFAULT_CFG();
    lvl.gpio = FLORA_PIN_WATER_LEVEL;
    if (flora_level_init(&lvl, &s_level) != ESP_OK) {
        s_level = NULL;
        ESP_LOGW(TAG, "czujnik poziomu niedostepny - pompka bez ochrony przed sucho");
    }

    flora_pump_cfg_t cfg = FLORA_PUMP_DEFAULT_CFG();
    cfg.pump_gpio       = FLORA_PIN_PUMP;
    cfg.level_gpio      = GPIO_NUM_NC;
    if (s_level) {
        cfg.water_cb  = flora_level_pump_cb;
        cfg.water_ctx = s_level;
    }
    cfg.max_run_ms      = HUB_PUMP_MAX_RUN_MS;
    cfg.min_interval_ms = DEBUG_PUMP_INTERVAL_MS;
    cfg.done_cb         = pump_done;
    cfg.ml_per_second   = hub_sensorcal_pump_ml_s();
    s_pump_ok = (flora_pump_init(&cfg) == ESP_OK);
    if (!s_pump_ok) {
        ESP_LOGW(TAG, "pompka niedostepna");
    } else if (cfg.ml_per_second <= 0.0f) {
        ESP_LOGW(TAG, "pompka bez kalibracji wydajnosci - dawki tylko w ms (cmd/pump_cal)");
    }
}

/** Kalibracja z NVS jak w test_soil() w bringup_sensors.c. */
static void soil_init(void)
{
    flora_soil_cfg_t cfg = FLORA_SOIL_DEFAULT_CFG();
    cfg.gpio    = FLORA_PIN_SOIL_ADC;
    cfg.raw_dry = FLORA_SOIL_RAW_DRY;
    cfg.raw_wet = FLORA_SOIL_RAW_WET;

    flora_store_t store = NULL;
    if (flora_store_open(FLORA_NVS_NS_CALIB, &store) == ESP_OK) {
        int32_t dry = FLORA_SOIL_RAW_DRY, wet = FLORA_SOIL_RAW_WET;
        flora_store_get_i32(store, FLORA_NVS_SOIL_DRY, &dry, FLORA_SOIL_RAW_DRY);
        flora_store_get_i32(store, FLORA_NVS_SOIL_WET, &wet, FLORA_SOIL_RAW_WET);
        cfg.raw_dry = dry;
        cfg.raw_wet = wet;
        flora_store_close(store);
    }
    s_soil_lock = xSemaphoreCreateMutex();
    s_soil_ok = s_soil_lock && (flora_soil_init(&cfg) == ESP_OK);
    if (!s_soil_ok) {
        ESP_LOGW(TAG, "czujnik gleby niedostepny");
    }
}

static void sensors_init(void)
{
    s_temp_ok = temp_init();
    if (!s_temp_ok) {
        ESP_LOGW(TAG, "brak DS18B20 - szukam dalej w tle");
    }

    flora_sonar_cfg_t scfg = FLORA_SONAR_DEFAULT_CFG();
    scfg.trig_gpio    = FLORA_PIN_US_TRIG;
    /* Kolejnosc = zlacza na plytce: A (J8), B (J9), C (J10). */
    scfg.echo_gpio[0] = FLORA_PIN_US_ECHO_1;
    scfg.echo_gpio[1] = FLORA_PIN_US_ECHO_2;
    scfg.echo_gpio[2] = FLORA_PIN_US_ECHO_3;
    scfg.count        = 3;
    s_sonar_ok = (flora_sonar_init(&scfg) == ESP_OK);
    if (s_sonar_ok) {
        s_sonar_lock = xSemaphoreCreateMutex();
        s_sonar_ok = s_sonar_lock &&
                     xTaskCreate(sonar_task, "sonar", 3072, NULL, 3, NULL) == pdPASS;
    }
    if (!s_sonar_ok) {
        ESP_LOGW(TAG, "dalmierz niedostepny");
    }

    soil_init();
}

static void mast_init(void)
{
    flora_pwrsw_cfg_t cfg = FLORA_PWRSW_DEFAULT_CFG();
    cfg.gpio = FLORA_PIN_MAST_SWITCH;
    if (flora_pwrsw_init(&cfg, &s_mast) != ESP_OK) {
        s_mast = NULL;
        ESP_LOGW(TAG, "klucz zasilania masztu niedostepny");
        return;
    }
    flora_pwrsw_on(s_mast);
}

/* ================= PUBLIKACJA: BRYLA ================= */

/** JSON nie zna NaN - zamieniamy na null. */
static const char *num(char *buf, size_t cap, float v, int decimals)
{
    if (isnan(v)) {
        return "null";
    }
    snprintf(buf, cap, "%.*f", decimals, v);
    return buf;
}

static void pub_sys(void)
{
    wifi_ap_record_t ap;
    int rssi = (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) ? ap.rssi : 0;
    flora_mqtt_publish_fmt("tele/sys", 0, false,
                           "{\"uptime_s\":%" PRId64 ",\"heap\":%" PRIu32 ",\"rssi_dbm\":%d}",
                           esp_timer_get_time() / 1000000,
                           (uint32_t)esp_get_free_heap_size(), rssi);
}

static void pub_power(void)
{
    flora_ina226_reading_t r;
    if (flora_ina226_read(s_ina, &r) != ESP_OK) {
        return;
    }
    flora_mqtt_publish_fmt("tele/power", 0, false,
                           "{\"v\":%.3f,\"a\":%.3f,\"w\":%.2f}",
                           r.bus_voltage_v, r.current_a, r.power_w);
}

static void pub_drive(void)
{
    static flora_encoders_sample_t prev;
    static bool have_prev;

    flora_encoders_sample_t now;
    if (flora_encoders_read(&now) != ESP_OK) {
        return;
    }
    float vl = 0.0f, vr = 0.0f;
    if (have_prev) {
        flora_encoders_speed_mm_s(&prev, &now, &vl, &vr);
    }
    prev = now;
    have_prev = true;

    float pl = 0.0f, pr = 0.0f;
    flora_motors_get_last(&pl, &pr);

    const char *stall = "none";
    if (flora_motors_is_stalled()) {
        if (flora_motors_stall_reason() == FLORA_MOTORS_STALL_OVERCURRENT) {
            stall = "current";
        } else {
            stall = flora_motors_stalled_wheel() == FLORA_MOTORS_WHEEL_LEFT ? "left" : "right";
        }
    }
    flora_mqtt_publish_fmt("tele/drive", 0, false,
                           "{\"pwr_l\":%.0f,\"pwr_r\":%.0f,"
                           "\"cnt_l\":%" PRId64 ",\"cnt_r\":%" PRId64 ","
                           "\"mm_l\":%.0f,\"mm_r\":%.0f,"
                           "\"v_l_mm_s\":%.0f,\"v_r_mm_s\":%.0f,\"stall\":\"%s\"}",
                           pl, pr, now.left_counts, now.right_counts,
                           now.left_mm, now.right_mm, vl, vr, stall);
}

/**
 * Przebieg petli predkosci do strojenia. Tylko w trybie pid, w trakcie
 * jazdy i sekunde po niej (widac wybieg), zeby na postoju nie smiecic.
 */
static void pub_pid(void)
{
    static int64_t last_active_us;

    hub_drive_state_t st;
    hub_drive_get_state(&st);
    if (st.mode != HUB_DRIVE_MODE_PID) {
        return;
    }
    int64_t now = esp_timer_get_time();
    if (st.active) {
        last_active_us = now;
    } else if (now - last_active_us > 1000000) {
        return;
    }
    flora_mqtt_publish_fmt("tele/pid", 0, false,
                           "{\"sp_l\":%.1f,\"sp_r\":%.1f,\"v_l\":%.1f,\"v_r\":%.1f,"
                           "\"u_l\":%.1f,\"u_r\":%.1f,\"ff_l\":%.1f,\"ff_r\":%.1f,"
                           "\"i_l\":%.1f,\"i_r\":%.1f}",
                           st.sp_l, st.sp_r, st.v_l, st.v_r, st.u_l, st.u_r,
                           st.ff_l, st.ff_r, st.i_l, st.i_r);
}

/**
 * Wszystkie warianty EKF: glowny na tele/pose, reszta na tele/pose_<nazwa>
 * (w Telegrafie osobne measurementy z tymi samymi polami - latwo nalozyc
 * na jeden wykres). Payload zostaje ponizej 256 B bufora flora_mqtt.
 */
static void pub_pose(void)
{
    for (int i = 0; i < hub_nav_variant_count(); i++) {
        hub_nav_pose_t p;
        if (!hub_nav_get(i, &p)) {
            continue;
        }
        char topic[32];
        if (i == 0) {
            snprintf(topic, sizeof topic, "tele/pose");
        } else {
            snprintf(topic, sizeof topic, "tele/pose_%s", p.name);
        }
        flora_mqtt_publish_fmt(topic, 0, false,
                               "{\"x_mm\":%.0f,\"y_mm\":%.0f,\"th_deg\":%.1f,\"b_deg\":%.1f,"
                               "\"sx_mm\":%.0f,\"sy_mm\":%.0f,\"sth_deg\":%.2f,\"spos_mm\":%.0f,"
                               "\"src\":%u,\"n_head\":%" PRIu32 ",\"n_sun\":%" PRIu32
                               ",\"n_tag\":%" PRIu32 ",\"n_rej\":%" PRIu32 ",\"n_rec\":%" PRIu32 "}",
                               p.x_mm, p.y_mm, p.th_deg, p.b_deg, p.sx_mm, p.sy_mm, p.sth_deg,
                               p.spos_mm, p.src, p.n_head, p.n_sun, p.n_tag, p.n_rej, p.n_rec);
    }
}

/** Stan bramek kompasu slonecznego - kody w hub_nav.h. */
static void pub_nav_sun(void)
{
    hub_nav_status_t st;
    hub_nav_status(&st);
    flora_mqtt_publish_fmt("tele/nav_sun", 0, false,
                           "{\"utc_ok\":%s,\"gate\":%d,\"sun_vs_mag_deg\":%.1f,"
                           "\"seen\":%" PRIu32 ",\"gated\":%" PRIu32 ",\"att_age_ms\":%" PRId32
                           ",\"gyro_ok\":%s,\"gyro_vs_odo_dps\":%.2f}",
                           st.time_ok ? "true" : "false", st.sun_gate, st.sun_diff_deg,
                           st.n_sun_seen, st.n_sun_gated, st.att_age_ms,
                           st.gyro_ok ? "true" : "false", st.gyro_vs_odo_dps);
}

static void pub_soil(void)
{
    int raw = 0, mv = 0;
    float pct = NAN;
    xSemaphoreTake(s_soil_lock, portMAX_DELAY);
    if (flora_soil_read_raw(&raw) != ESP_OK) {
        xSemaphoreGive(s_soil_lock);
        return;
    }
    flora_soil_read_mv(&mv);
    flora_soil_read_percent(&pct);
    xSemaphoreGive(s_soil_lock);
    char b[16];
    flora_mqtt_publish_fmt("tele/soil", 0, false,
                           "{\"raw\":%d,\"mv\":%d,\"pct\":%s}",
                           raw, mv, num(b, sizeof b, pct, 1));
}

/* ================= DALMIERZE: POMIAR CIAGLY I FILTR =================
 *
 * Pomiar idzie we wlasnym zadaniu co SONAR_PERIOD_MS, a telemetria tylko
 * publikuje wynik z ostatnich SONAR_WIN pomiarow (pomiar blokuje do ~250 ms,
 * wiec w petli telemetrii opoznial wszystkie inne tematy).
 *
 * Dlaczego filtr: wiazka ma ~60-70 st. Maly, ukosny albo ruchomy cel daje
 * za slabe echo i czujnik lapie cos dalej - na 70 cm pojawialo sie nagle
 * 1000-1800 mm, a stale przedmioty w pokoju dawaly powtarzalne "drugie"
 * odleglosci. Przy plaskim celu na wprost rozrzut to kilka mm.
 *
 * Publikujemy dwie liczby na czujnik:
 *   mm_x   mediana - stabilna wartosc do pokazania, wycina pojedyncze skoki;
 *          null, gdy w oknie mniej niz polowa pomiarow sie udala,
 *   near_x minimum - ostrozna wartosc dla przyszlego weta napedu: przy
 *          omijaniu przeszkod grozny jest odczyt za DALEKI, nie za bliski.
 * st_x to status: "ok" gdy jest mediana, inaczej ostatni blad. */
#define SONAR_N          3
#define SONAR_WIN        5
#define SONAR_PERIOD_MS  100

/* Wybor dalmierza do testu: -1 wszystkie, 0..2 tylko A/B/C (cmd/sonar). */
static volatile int s_sonar_sel = -1;

typedef struct {
    float     mm[SONAR_WIN];     /* NAN = pomiar nieudany */
    esp_err_t last_err;
    int       head, filled;
} sonar_hist_t;

static sonar_hist_t      s_sonar[SONAR_N];

esp_err_t hub_telemetry_sonar_select(const char *arg)
{
    if (strcmp(arg, "all") == 0 || arg[0] == 0) { s_sonar_sel = -1; return ESP_OK; }
    if ((arg[0] == 'a' || arg[0] == 'b' || arg[0] == 'c') && arg[1] == 0) {
        s_sonar_sel = arg[0] - 'a';
        return ESP_OK;
    }
    return ESP_ERR_INVALID_ARG;
}

static const char *sonar_status(esp_err_t err)
{
    switch (err) {
    case ESP_OK:                   return "ok";
    case ESP_ERR_TIMEOUT:          return "echo";
    case ESP_ERR_INVALID_RESPONSE: return "range";
    case ESP_ERR_INVALID_STATE:    return "busy";
    default:                       return "err";
    }
}

static void sonar_task(void *arg)
{
    TickType_t wake = xTaskGetTickCount();
    int last_sel = -2;
    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(SONAR_PERIOD_MS));
        const int sel = s_sonar_sel;

        float mm[FLORA_SONAR_MAX_SENSORS];
        esp_err_t res[FLORA_SONAR_MAX_SENSORS];
        for (int i = 0; i < SONAR_N; i++) {
            mm[i] = NAN;
            res[i] = ESP_FAIL;
        }
        if (sel < 0) {
            if (flora_sonar_measure_all(mm, res) != ESP_OK) {
                continue;
            }
        } else {
            res[sel] = flora_sonar_measure(sel, &mm[sel]);
        }

        xSemaphoreTake(s_sonar_lock, portMAX_DELAY);
        if (sel != last_sel) {
            /* Zmiana trybu testu - stare probki nie pasuja do nowego. */
            memset(s_sonar, 0, sizeof s_sonar);
            last_sel = sel;
        }
        for (int i = 0; i < SONAR_N; i++) {
            if (sel >= 0 && sel != i) {
                continue;
            }
            sonar_hist_t *h = &s_sonar[i];
            h->mm[h->head] = (res[i] == ESP_OK && mm[i] != FLORA_SONAR_INVALID) ? mm[i] : NAN;
            h->head = (h->head + 1) % SONAR_WIN;
            if (h->filled < SONAR_WIN) h->filled++;
            h->last_err = res[i];
        }
        xSemaphoreGive(s_sonar_lock);
    }
}

float hub_telemetry_sonar_near(int index)
{
    if (!s_sonar_ok || index < 0 || index >= SONAR_N) {
        return NAN;
    }
    const int sel = s_sonar_sel;
    if (sel >= 0 && sel != index) {
        return NAN;
    }
    float near = NAN;
    xSemaphoreTake(s_sonar_lock, portMAX_DELAY);
    const sonar_hist_t *h = &s_sonar[index];
    for (int k = 0; k < h->filled; k++) {
        if (!isnan(h->mm[k]) && (isnan(near) || h->mm[k] < near)) {
            near = h->mm[k];
        }
    }
    xSemaphoreGive(s_sonar_lock);
    return near;
}

esp_err_t hub_telemetry_soil_read(float *pct)
{
    if (!s_soil_ok) {
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreTake(s_soil_lock, portMAX_DELAY);
    const esp_err_t err = flora_soil_read_percent(pct);
    xSemaphoreGive(s_soil_lock);
    return err;
}

bool hub_telemetry_water_present(void)
{
    return s_level ? flora_level_water_present(s_level) : true;
}

static int cmp_float(const void *a, const void *b)
{
    float fa = *(const float *)a, fb = *(const float *)b;
    return (fa > fb) - (fa < fb);
}

static void pub_sonar(void)
{
    const int sel = s_sonar_sel;
    float med[SONAR_N], near[SONAR_N];
    const char *st[SONAR_N];

    xSemaphoreTake(s_sonar_lock, portMAX_DELAY);
    for (int i = 0; i < SONAR_N; i++) {
        const sonar_hist_t *h = &s_sonar[i];
        float ok[SONAR_WIN];
        int n = 0;
        for (int k = 0; k < h->filled; k++) {
            if (!isnan(h->mm[k])) ok[n++] = h->mm[k];
        }
        med[i] = NAN;
        near[i] = NAN;
        if (n > 0) {
            qsort(ok, n, sizeof ok[0], cmp_float);
            near[i] = ok[0];
            if (2 * n >= h->filled && h->filled > 0) {
                med[i] = ok[n / 2];
            }
        }
        if (sel >= 0 && sel != i) {
            st[i] = "off";
        } else if (h->filled == 0) {
            st[i] = "err";
        } else {
            st[i] = !isnan(med[i]) ? "ok" : sonar_status(h->last_err);
        }
    }
    xSemaphoreGive(s_sonar_lock);

    char b[2][SONAR_N][16];
    const char *v[2][SONAR_N];   /* num() przy NaN zwraca "null" i NIE pisze do bufora */
    for (int i = 0; i < SONAR_N; i++) {
        v[0][i] = num(b[0][i], sizeof b[0][i], med[i], 0);
        v[1][i] = num(b[1][i], sizeof b[1][i], near[i], 0);
    }
    /* Predkosc dzwieku wprost w telemetrii - widac, czy korekta
     * temperatury dziala (343 m/s przy 20 st C, 331 przy 0). */
    flora_mqtt_publish_fmt("tele/sonar", 0, false,
                           "{\"mm_a\":%s,\"mm_b\":%s,\"mm_c\":%s,"
                           "\"near_a\":%s,\"near_b\":%s,\"near_c\":%s,"
                           "\"st_a\":\"%s\",\"st_b\":\"%s\",\"st_c\":\"%s\","
                           "\"sonar_sel\":\"%s\",\"sound_m_s\":%.1f}",
                           v[0][0], v[0][1], v[0][2], v[1][0], v[1][1], v[1][2],
                           st[0], st[1], st[2],
                           sel < 0 ? "all" : (const char *[]){ "a", "b", "c" }[sel],
                           flora_sonar_speed_of_sound());
}

/**
 * Temperatura dwuetapowo: zlecenie konwersji w jednym takcie, odczyt
 * po czasie konwersji. Odczyt blokujacy zatrzymalby cala telemetrie
 * na 375 ms.
 */
/* ================= DS18B20 Z PONAWIANIEM =================
 *
 * Czujnik bywa niewidoczny przy starcie (styk na zaciskach / dupontach),
 * a dawniej znaczylo to brak temperatury az do restartu. Teraz:
 *   - brak przy starcie -> szukamy dalej, co 30 s, potem coraz rzadziej
 *     (do 10 min), zeby nie zasypywac logow, gdy czujnika naprawde nie ma,
 *   - pojedynczy blad odczytu (CRC, brak odpowiedzi) -> jedna powtorka,
 *   - TEMP_LOST_AFTER odczytow pod rzad bez zadnego wyniku -> czujnik
 *     uznany za zgubiony, magistrala zwolniona i szukanie od nowa
 *     (np. po przepieciu przewodu adres i liczba czujnikow moga sie zmienic).
 * W tele/temp: err = bledy odczytu od startu, rescan = ile razy szukano od nowa. */
#define TEMP_RETRY_MIN_MS  30000
#define TEMP_RETRY_MAX_MS  600000
#define TEMP_LOST_AFTER    5

static uint32_t s_temp_errors, s_temp_rescans;

static bool temp_init(void)
{
    flora_temp_cfg_t tcfg = FLORA_TEMP_DEFAULT_CFG();
    tcfg.gpio = FLORA_PIN_DS18B20;
    return flora_temp_init(&tcfg) == ESP_OK;
}

/** Wolane co takt, gdy czujnika nie ma: ponawia szukanie z rosnacym odstepem. */
static void temp_retry(void)
{
    static int64_t next_us;
    static uint32_t gap_ms = TEMP_RETRY_MIN_MS;
    int64_t now = esp_timer_get_time();
    if (next_us == 0) {
        next_us = now + (int64_t)gap_ms * 1000;
        return;
    }
    if (now < next_us) {
        return;
    }
    s_temp_rescans++;
    if (temp_init()) {
        ESP_LOGI(TAG, "DS18B20 znaleziony przy ponownym szukaniu (%" PRIu32 ")", s_temp_rescans);
        s_temp_ok = true;
        next_us = 0;
        gap_ms = TEMP_RETRY_MIN_MS;
        return;
    }
    gap_ms = gap_ms * 2 > TEMP_RETRY_MAX_MS ? TEMP_RETRY_MAX_MS : gap_ms * 2;
    next_us = now + (int64_t)gap_ms * 1000;
}

static void temp_step(bool due)
{
    static int64_t started_us;
    static int lost_streak;

    if (started_us == 0) {
        if (due) {
            if (flora_temp_start_conversion() == ESP_OK) {
                started_us = esp_timer_get_time();
            } else {
                s_temp_errors++;
                lost_streak++;
            }
        }
    } else if (esp_timer_get_time() - started_us >= (int64_t)flora_temp_conversion_ms() * 1000) {
        started_us = 0;

        char buf[128];
        int n = snprintf(buf, sizeof buf, "{\"c\":[");
        bool any = false;
        for (int i = 0; i < flora_temp_count() && n < (int)sizeof buf - 48; i++) {
            float c = NAN;
            esp_err_t err = flora_temp_fetch(i, &c);
            if (err != ESP_OK) {
                /* Zwykle CRC po zakloceniu - wynik konwersji dalej lezy
                 * w czujniku, wiec drugi odczyt czesto juz przechodzi. */
                s_temp_errors++;
                err = flora_temp_fetch(i, &c);
            }
            if (err != ESP_OK || c == FLORA_TEMP_INVALID) {
                c = NAN;
            } else {
                any = true;
            }
            char b[16];
            n += snprintf(buf + n, sizeof buf - n, "%s%s", i ? "," : "", num(b, sizeof b, c, 2));
        }
        snprintf(buf + n, sizeof buf - n, "],\"err\":%" PRIu32 ",\"rescan\":%" PRIu32 "}",
                 s_temp_errors, s_temp_rescans);
        flora_mqtt_publish_str("tele/temp", buf, 0, false);
        lost_streak = any ? 0 : lost_streak + 1;
    }

    if (lost_streak >= TEMP_LOST_AFTER) {
        ESP_LOGW(TAG, "DS18B20 nie odpowiada %d razy pod rzad - szukam od nowa", lost_streak);
        flora_temp_deinit();
        s_temp_ok = false;
        started_us = 0;
        lost_streak = 0;
    }
}

static void pub_water(void)
{
    flora_mqtt_publish_fmt("tele/water", 0, false, "{\"present\":%s,\"pin\":%s}",
                           flora_level_water_present(s_level) ? "true" : "false",
                           flora_level_raw(s_level) ? "true" : "false");
}

static void pub_pump(void)
{
    static const char *reasons[] = {
        [FLORA_PUMP_DONE_COMPLETED] = "completed",
        [FLORA_PUMP_DONE_STOPPED]   = "stopped",
        [FLORA_PUMP_DONE_DRY]       = "dry",
        [FLORA_PUMP_DONE_CLAMPED]   = "clamped",
    };
    flora_pump_stats_t st;
    flora_pump_get_stats(&st);
    int last = s_pump_last_reason;
    flora_mqtt_publish_fmt("tele/pump", 0, false,
                           "{\"running\":%s,\"remaining_ms\":%" PRIu32 ","
                           "\"doses\":%" PRIu32 ",\"total_ms\":%" PRIu32 ","
                           "\"window_ms\":%" PRIu32 ",\"refused_interval\":%" PRIu32 ","
                           "\"refused_limit\":%" PRIu32 ",\"refused_dry\":%" PRIu32 ","
                           "\"last\":\"%s\"}",
                           flora_pump_is_running() ? "true" : "false",
                           flora_pump_remaining_ms(), st.dose_count, st.total_run_ms,
                           st.window_run_ms, st.refused_interval, st.refused_limit,
                           st.refused_dry, last >= 0 ? reasons[last] : "none");
}

static void pub_mast(void)
{
    flora_mqtt_publish_fmt("tele/mast", 0, false, "{\"on\":%s}",
                           flora_pwrsw_is_on(s_mast) ? "true" : "false");
}

/* ================= PUBLIKACJA: GLOWICA ================= */

static void pub_link(void)
{
    flora_link_handle_t h = hub_links_handle();
    flora_link_stats_t st;
    flora_link_get_stats(h, &st);
    flora_mqtt_publish_fmt("head/link", 0, false,
                           "{\"up\":%s,\"ok\":%" PRIu32 ",\"sent\":%" PRIu32 ","
                           "\"crc\":%" PRIu32 ",\"len\":%" PRIu32 ",\"ver\":%" PRIu32 ","
                           "\"retx\":%" PRIu32 ",\"timeouts\":%" PRIu32 ","
                           "\"overflow\":%" PRIu32 "}",
                           flora_link_is_up(h) ? "true" : "false",
                           st.frames_ok, st.frames_sent, st.err_crc, st.err_length,
                           st.err_version, st.retransmits, st.cmd_timeouts,
                           st.uart_overflow);
}

/** Publikuje wiadomosci glowicy, ktore przyszly od poprzedniego taktu. */
static void pub_head(bool attitude_due)
{
    static uint32_t seq_hello, seq_att, seq_sun, seq_wx, seq_st, seq_obs, seq_tags, seq_cam;
    static uint32_t seq_mag, seq_magcal, seq_lvl;

    flora_hello_t hello;
    if (hub_links_get(FLORA_MSG_HELLO, &hello, sizeof hello, &seq_hello)) {
        flora_mqtt_publish_fmt("head/hello", 0, false,
                               "{\"ver\":%u,\"sensors\":%u,\"fw\":%" PRIu32 "}",
                               hello.msgs_version, hello.sensors_present, hello.fw_build);
    }

    /* ATTITUDE przychodzi 20 Hz, publikujemy najwyzej 5 Hz. Nie ruszamy
     * seq miedzy publikacjami, wiec zawsze idzie najswiezsza probka. */
    flora_attitude_t a;
    if (attitude_due && hub_links_get(FLORA_MSG_ATTITUDE, &a, sizeof a, &seq_att)) {
        /* Wiek probki tylko przy TIME_SYNC - inaczej znacznik jest
         * w zegarze glowicy i roznica nic nie znaczy (-1). */
        long age_ms = -1;
        if (a.flags & FLORA_ATT_TIME_SYNCED) {
            age_ms = (long)((int32_t)((uint32_t)esp_timer_get_time() - a.timestamp_us) / 1000);
        }
        flora_mqtt_publish_fmt("head/attitude", 0, false,
                               "{\"hdg_deg\":%.2f,\"sigma_deg\":%.2f,\"rate_dps\":%.2f,"
                               "\"pitch_deg\":%.2f,\"roll_deg\":%.2f,\"q\":%u,\"flags\":%u,"
                               "\"age_ms\":%ld}",
                               a.heading_cdeg / 100.0f, a.heading_sigma_cdeg / 100.0f,
                               a.rate_z_cdps / 100.0f, a.pitch_cdeg / 100.0f,
                               a.roll_cdeg / 100.0f, a.quality, a.flags, age_ms);
    }

    flora_sun_t s;
    if (hub_links_get(FLORA_MSG_SUN, &s, sizeof s, &seq_sun)) {
        flora_mqtt_publish_fmt("head/sun", 0, false,
                               "{\"az_deg\":%.2f,\"el_deg\":%.2f,"
                               "\"lux\":[%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 "],"
                               "\"valid\":%s}",
                               s.azimuth_cdeg / 100.0f, s.elevation_cdeg / 100.0f,
                               (uint32_t)s.lux_div4[0] * 4, (uint32_t)s.lux_div4[1] * 4,
                               (uint32_t)s.lux_div4[2] * 4, (uint32_t)s.lux_div4[3] * 4,
                               s.valid ? "true" : "false");
    }

    /* Diagnostyka magnetometru 1 Hz - test D-009 (silniki on/off). */
    flora_mag_info_t mi;
    if (hub_links_get(FLORA_MSG_MAG_INFO, &mi, sizeof mi, &seq_mag)) {
        flora_mqtt_publish_fmt("head/mag", 0, false,
                               "{\"field_h_ut\":%.2f,\"dip_deg\":%.2f,\"field_ut\":%.2f,"
                               "\"gyro_bias_dps\":%.3f,\"hdg_sigma_deg\":%.2f,"
                               "\"mag_cal_ok\":%s,\"mag_disturbed\":%s,\"mag_collecting\":%s,"
                               "\"dist_cause\":%u}",
                               mi.field_h_cut / 100.0f, mi.dip_cdeg / 100.0f,
                               mi.field_total_cut / 100.0f, mi.gyro_bias_mdps / 1000.0f,
                               mi.heading_sigma_cdeg / 100.0f,
                               (mi.flags & FLORA_MAG_CAL_OK) ? "true" : "false",
                               (mi.flags & FLORA_MAG_DISTURBED) ? "true" : "false",
                               (mi.flags & FLORA_MAG_COLLECTING) ? "true" : "false",
                               mi.cause);
    }

    /* Wynik kalibracji - retain, zeby panel otwarty pozniej go widzial. */
    flora_mag_cal_result_t mc;
    if (hub_links_get(FLORA_MSG_MAG_CAL_RESULT, &mc, sizeof mc, &seq_magcal)) {
        flora_mqtt_publish_fmt("head/mag_cal", 1, true,
                               "{\"cal_status\":%u,\"sectors\":%u,\"samples\":%u,"
                               "\"off_x_ut\":%.2f,\"off_y_ut\":%.2f,\"off_z_ut\":%.2f,"
                               "\"axis_ratio\":%.3f,"
                               "\"rms_pct\":%.2f,\"ref_h_ut\":%.2f,\"ref_dip_deg\":%.2f}",
                               mc.status, mc.sectors, mc.samples,
                               mc.offset_x_cut / 100.0f, mc.offset_y_cut / 100.0f,
                               mc.offset_z_cut / 100.0f,
                               mc.axis_ratio_milli / 1000.0f, mc.rms_cpct / 100.0f,
                               mc.field_h_cut / 100.0f, mc.dip_cdeg / 100.0f);
    }

    flora_level_cal_result_t lc;
    if (hub_links_get(FLORA_MSG_LEVEL_CAL_RESULT, &lc, sizeof lc, &seq_lvl)) {
        flora_mqtt_publish_fmt("head/level_cal", 1, true,
                               "{\"lvl_status\":%u,\"step\":%u,\"meas_pitch_deg\":%.2f,"
                               "\"meas_roll_deg\":%.2f,\"off_pitch_deg\":%.2f,\"off_roll_deg\":%.2f}",
                               lc.status, lc.step, lc.pitch_cdeg / 100.0f, lc.roll_cdeg / 100.0f,
                               lc.off_pitch_cdeg / 100.0f, lc.off_roll_cdeg / 100.0f);
    }

    flora_weather_t w;
    if (hub_links_get(FLORA_MSG_WEATHER, &w, sizeof w, &seq_wx)) {
        /* Temperatura powietrza z klatki Stevensona do korekty predkosci
         * dzwieku w dalmierzach: 0,6 m/s na stopien, czyli przy 0 st C
         * blad ~3,5% wzgledem domyslnych 20 st C. DS18B20 sie nie nadaje -
         * mierzy pakiet baterii we wnetrzu bryly, nie powietrze. */
        float t_air = w.temp_cc / 100.0f;
        if (s_sonar_ok && t_air > -40.0f && t_air < 60.0f) {
            flora_sonar_set_temperature(t_air);
        }
        flora_mqtt_publish_fmt("head/weather", 0, false,
                               "{\"t_c\":%.2f,\"rh_pct\":%.2f,\"p_hpa\":%.2f}",
                               w.temp_cc / 100.0f, w.humidity_cpct / 100.0f,
                               w.pressure_pa / 100.0f);
    }

    flora_head_status_t st;
    if (hub_links_get(FLORA_MSG_HEAD_STATUS, &st, sizeof st, &seq_st)) {
        flora_mqtt_publish_fmt("head/status", 0, false,
                               "{\"uptime_s\":%" PRIu32 ",\"sensors_ok\":%u,\"errors\":%u}",
                               st.uptime_ms / 1000, st.sensors_ok, st.error_count);
    }

    flora_obstacles_t o;
    if (hub_links_get(FLORA_MSG_OBSTACLES, &o, sizeof o, &seq_obs)) {
        char buf[160];
        int n = snprintf(buf, sizeof buf, "{\"fov_deg\":%.1f,\"valid\":%s,\"cm\":[",
                         o.fov_cdeg / 100.0f, o.valid ? "true" : "false");
        for (int i = 0; i < FLORA_OBSTACLE_SECTORS; i++) {
            n += snprintf(buf + n, sizeof buf - n, "%s%u", i ? "," : "", o.sectors[i]);
        }
        snprintf(buf + n, sizeof buf - n, "]}");
        flora_mqtt_publish_str("head/obstacles", buf, 0, false);
    }

    flora_tags_t t;
    if (hub_links_get(FLORA_MSG_TAGS, &t, sizeof t, &seq_tags)) {
        char buf[256];
        int n = snprintf(buf, sizeof buf, "{\"tags\":[");
        for (int i = 0; i < t.count && i < FLORA_TAGS_MAX; i++) {
            n += snprintf(buf + n, sizeof buf - n,
                          "%s{\"id\":%u,\"mm\":%u,\"brg_deg\":%.2f,\"q\":%u}",
                          i ? "," : "", t.tags[i].id, t.tags[i].distance_mm,
                          t.tags[i].bearing_cdeg / 100.0f, t.tags[i].quality);
        }
        snprintf(buf + n, sizeof buf - n, "]}");
        flora_mqtt_publish_str("head/tags", buf, 0, false);
    }

    flora_cam_status_t c;
    if (hub_links_get(FLORA_MSG_CAM_STATUS, &c, sizeof c, &seq_cam)) {
        flora_mqtt_publish_fmt("head/cam", 0, false,
                               "{\"uptime_s\":%" PRIu32 ",\"frames\":%u,\"errors\":%u,"
                               "\"camera_ok\":%s}",
                               c.uptime_ms / 1000, c.frames_processed, c.error_count,
                               c.camera_ok ? "true" : "false");
    }
}

/* ================= ZADANIE ================= */

/** true co `every_ms`, liczone w taktach zadania. */
#define DUE(tick, every_ms) ((tick) % ((every_ms) / TICK_MS) == 0)

static void telemetry_task(void *arg)
{
    TickType_t wake = xTaskGetTickCount();
    uint32_t tick = 0;
    bool last_water = false, last_mast = false;
    bool was_connected = false;

    for (;; tick++) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(TICK_MS));
        if (!flora_mqtt_is_connected()) {
            was_connected = false;
            continue;
        }
        /* Po kazdym (ponownym) polaczeniu - broker mogl byc zrestartowany
         * bez trwalego zapisu i zgubic wiadomosc z flaga retain. */
        if (!was_connected) {
            was_connected = true;
            hub_config_publish();
            hub_diag_publish_boot();
        }

        if (DUE(tick, 5000)) { pub_sys(); }
        if (s_ina && DUE(tick, 500)) { pub_power(); }
        if (s_drive_ok && DUE(tick, 200)) { pub_drive(); }
        if (s_drive_ok) { pub_pid(); }
        if (hub_nav_ready() && DUE(tick, 500)) { pub_pose(); }
        if (hub_nav_ready() && DUE(tick, 1000)) { pub_nav_sun(); }
        if (s_soil_ok && DUE(tick, 2000)) { pub_soil(); }
        /* Jeden czujnik w tescie: szybciej, zeby reakcja na dlon byla od razu. */
        if (s_sonar_ok && DUE(tick, s_sonar_sel < 0 ? 500 : 200)) { pub_sonar(); }
        if (s_temp_ok) { temp_step(DUE(tick, 2000)); } else { temp_retry(); }

        if (s_level) {
            bool water = flora_level_water_present(s_level);
            if (water != last_water || DUE(tick, 1000)) {
                pub_water();
                last_water = water;
            }
        }
        if (s_pump_ok && (DUE(tick, 1000) || (flora_pump_is_running() && DUE(tick, 200)))) {
            pub_pump();
        }
        if (s_mast) {
            bool on = flora_pwrsw_is_on(s_mast);
            if (on != last_mast || DUE(tick, 5000)) {
                pub_mast();
                last_mast = on;
            }
        }

        if (hub_links_handle()) {
            if (DUE(tick, 2000)) { pub_link(); }
            pub_head(DUE(tick, 200));
        }
    }
}

esp_err_t hub_telemetry_start(flora_ina226_handle_t ina)
{
    s_ina = ina;

    drive_init();
    pump_init();
    sensors_init();
    mast_init();

    ESP_LOGI(TAG, "naped %s, pompka %s, gleba %s, sonar %s, DS18B20 %s, zasilanie %s, maszt %s",
             s_drive_ok ? "ok" : "BRAK", s_pump_ok ? "ok" : "BRAK",
             s_soil_ok ? "ok" : "BRAK", s_sonar_ok ? "ok" : "BRAK",
             s_temp_ok ? "ok" : "BRAK", s_ina ? "ok" : "BRAK", s_mast ? "ok" : "BRAK");

    /* Stos z zapasem: kilka buforow JSON po 256 B na raz plus vsnprintf. */
    BaseType_t ok = xTaskCreate(telemetry_task, "telemetry", 6144, NULL, 4, NULL);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}
