/**
 * @file hub_telemetry.c
 * @brief Publikacja stanu bryly i wiadomosci glowicy na MQTT.
 *
 * Konfiguracje sprzetu sa przeniesione z testow rozruchowych, ktore je
 * potwierdzily na sprzecie - w szczegolnosci zamienione wtyczki enkoderow
 * z bringup_drive.c. Zmiana tam wymaga zmiany tutaj.
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
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"

#include "hub_telemetry.h"
#include "hub_config.h"
#include "hub_diag.h"
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

/** Powod ostatniego zakonczenia pompki; -1 gdy jeszcze nie pracowala. */
static volatile int s_pump_last_reason = -1;

flora_pwrsw_handle_t hub_telemetry_mast(void) { return s_mast; }

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
}

/* ================= INICJALIZACJA SPRZETU ================= */

/** Konfiguracja z drive_hw_init() i enable_stall_guard() w bringup_drive.c. */
static void drive_init(void)
{
    flora_motors_cfg_t mcfg = FLORA_MOTORS_DEFAULT_CFG();
    mcfg.left  = (flora_motor_cfg_t){ .pwm_gpio = FLORA_PIN_MOTOR_L_PWM,
                                      .dir_gpio = FLORA_PIN_MOTOR_L_DIR,
                                      .invert   = false };
    mcfg.right = (flora_motor_cfg_t){ .pwm_gpio = FLORA_PIN_MOTOR_R_PWM,
                                      .dir_gpio = FLORA_PIN_MOTOR_R_DIR,
                                      .invert   = true };
    if (flora_motors_init(&mcfg) != ESP_OK) {
        ESP_LOGW(TAG, "naped niedostepny");
        return;
    }

    /* Wtyczki enkoderow sa zamienione wzgledem pinoutu - patrz bringup_drive.c. */
    flora_encoders_cfg_t ecfg = FLORA_ENCODERS_DEFAULT_CFG();
    ecfg.left  = (flora_encoder_cfg_t){ .a_gpio = FLORA_PIN_ENC_R_A,
                                        .b_gpio = FLORA_PIN_ENC_R_B,
                                        .invert = true };
    ecfg.right = (flora_encoder_cfg_t){ .a_gpio = FLORA_PIN_ENC_L_A,
                                        .b_gpio = FLORA_PIN_ENC_L_B,
                                        .invert = true };
    ecfg.mm_per_count = FLORA_MM_PER_COUNT;
    if (flora_encoders_init(&ecfg) != ESP_OK) {
        /* Bez enkoderow straznik widzialby wieczny utyk - naped zostaje
         * wylaczony, zamiast jezdzic bez zabezpieczenia. */
        ESP_LOGW(TAG, "enkodery niedostepne - naped zablokowany");
        flora_motors_deinit();
        return;
    }

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
    s_pump_ok = (flora_pump_init(&cfg) == ESP_OK);
    if (!s_pump_ok) {
        ESP_LOGW(TAG, "pompka niedostepna");
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
    s_soil_ok = (flora_soil_init(&cfg) == ESP_OK);
    if (!s_soil_ok) {
        ESP_LOGW(TAG, "czujnik gleby niedostepny");
    }
}

static void sensors_init(void)
{
    flora_temp_cfg_t tcfg = FLORA_TEMP_DEFAULT_CFG();
    tcfg.gpio = FLORA_PIN_DS18B20;
    s_temp_ok = (flora_temp_init(&tcfg) == ESP_OK);
    if (!s_temp_ok) {
        ESP_LOGW(TAG, "brak DS18B20");
    }

    flora_sonar_cfg_t scfg = FLORA_SONAR_DEFAULT_CFG();
    scfg.trig_gpio    = FLORA_PIN_US_TRIG;
    scfg.echo_gpio[0] = FLORA_PIN_US_ECHO_1;   /* jedyny podlaczony */
    scfg.count        = 1;
    s_sonar_ok = (flora_sonar_init(&scfg) == ESP_OK);
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

static void pub_soil(void)
{
    int raw = 0, mv = 0;
    float pct = NAN;
    if (flora_soil_read_raw(&raw) != ESP_OK) {
        return;
    }
    flora_soil_read_mv(&mv);
    flora_soil_read_percent(&pct);
    char b[16];
    flora_mqtt_publish_fmt("tele/soil", 0, false,
                           "{\"raw\":%d,\"mv\":%d,\"pct\":%s}",
                           raw, mv, num(b, sizeof b, pct, 1));
}

static void pub_sonar(void)
{
    float mm = NAN;
    if (flora_sonar_measure(0, &mm) != ESP_OK || mm == FLORA_SONAR_INVALID) {
        mm = NAN;
    }
    char b[16];
    flora_mqtt_publish_fmt("tele/sonar", 0, false, "{\"mm\":%s}", num(b, sizeof b, mm, 0));
}

/**
 * Temperatura dwuetapowo: zlecenie konwersji w jednym takcie, odczyt
 * po czasie konwersji. Odczyt blokujacy zatrzymalby cala telemetrie
 * na 375 ms.
 */
static void temp_step(bool due)
{
    static int64_t started_us;

    if (started_us == 0) {
        if (due && flora_temp_start_conversion() == ESP_OK) {
            started_us = esp_timer_get_time();
        }
        return;
    }
    if (esp_timer_get_time() - started_us < (int64_t)flora_temp_conversion_ms() * 1000) {
        return;
    }
    started_us = 0;

    char buf[96];
    int n = snprintf(buf, sizeof buf, "{\"c\":[");
    for (int i = 0; i < flora_temp_count() && n < (int)sizeof buf - 12; i++) {
        float c = NAN;
        if (flora_temp_fetch(i, &c) != ESP_OK || c == FLORA_TEMP_INVALID) {
            c = NAN;
        }
        char b[16];
        n += snprintf(buf + n, sizeof buf - n, "%s%s", i ? "," : "", num(b, sizeof b, c, 2));
    }
    snprintf(buf + n, sizeof buf - n, "]}");
    flora_mqtt_publish_str("tele/temp", buf, 0, false);
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
        flora_mqtt_publish_fmt("head/attitude", 0, false,
                               "{\"hdg_deg\":%.2f,\"rate_dps\":%.2f,\"pitch_deg\":%.2f,"
                               "\"roll_deg\":%.2f,\"q\":%u,\"flags\":%u}",
                               a.heading_cdeg / 100.0f, a.rate_z_cdps / 100.0f,
                               a.pitch_cdeg / 100.0f, a.roll_cdeg / 100.0f,
                               a.quality, a.flags);
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

    flora_weather_t w;
    if (hub_links_get(FLORA_MSG_WEATHER, &w, sizeof w, &seq_wx)) {
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
        if (s_soil_ok && DUE(tick, 2000)) { pub_soil(); }
        if (s_sonar_ok && DUE(tick, 500)) { pub_sonar(); }
        if (s_temp_ok) { temp_step(DUE(tick, 2000)); }

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
