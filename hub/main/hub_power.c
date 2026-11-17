/**
 * @file hub_power.c
 * @brief Energia bryly, opis w hub_power.h.
 */
#include <inttypes.h>
#include <math.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_sleep.h"
#include "esp_timer.h"

#include "hub_autocfg.h"
#include "hub_drive.h"
#include "hub_headcal.h"
#include "hub_mission.h"
#include "hub_ota.h"
#include "hub_params.h"
#include "hub_power.h"
#include "hub_rec.h"
#include "hub_telemetry.h"
#include "flora_board.h"
#include "flora_ina226.h"
#include "flora_motors.h"
#include "flora_mqtt.h"
#include "flora_pump.h"
#include "flora_pwrsw.h"
#include "flora_sun.h"

static const char *TAG = "energia";

/** Po tylu sekundach bez ruchu klucz MDD10A jest wylaczany. */
#define KEY_IDLE_S        10
/** Rozruch sterownika po zalaczeniu klucza (plan: wlacz -> ~100 ms -> PWM). */
#define KEY_BOOT_MS      100
/** Prad szyny, od ktorego probka napiecia jest "pod obciazeniem". */
#define LOAD_HIGH_A      0.8f
/** Pomiar napiecia starszy niz to nie jest "swiezy". */
#define V_MAX_AGE_US  3000000
/** Jak w hub_nav: czas przed ta data to zegar bez SNTP. */
#define TIME_VALID_UNIX 1700000000LL

static flora_energy_t     s_energy;
static uint32_t           s_gen;
static flora_energy_out_t s_out;          /**< pod s_mux */
static portMUX_TYPE       s_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile float     s_bus_v = NAN;
static volatile int64_t   s_bus_v_us;
static volatile int64_t   s_activity_us;
static bool               s_started;
static bool               s_mast_forced_off;
static flora_energy_mode_t s_last_mode;

static SemaphoreHandle_t    s_key_lock;
static flora_pwrsw_handle_t s_key;
static volatile int64_t     s_key_use_us;

bool hub_power_bus_v(float *v)
{
    const float x = s_bus_v;
    *v = x;
    return !isnan(x) && esp_timer_get_time() - s_bus_v_us <= V_MAX_AGE_US;
}

void hub_power_get(flora_energy_out_t *out)
{
    portENTER_CRITICAL(&s_mux);
    *out = s_out;
    portEXIT_CRITICAL(&s_mux);
}

void hub_power_note_activity(void)
{
    s_activity_us = esp_timer_get_time();
}

bool hub_power_sun(float *elev_deg, float *azimuth_deg)
{
    time_t now;
    time(&now);
    if ((long long)now <= TIME_VALID_UNIX) {
        return false;
    }
    hub_params_t p;
    hub_params_get(&p);
    const flora_sun_pos_t s = flora_sun_position((int64_t)now, p.lat_deg, p.lon_deg);
    *elev_deg = (float)s.elevation_deg;
    *azimuth_deg = (float)s.azimuth_deg;
    return true;
}

/* ================= KLUCZ MDD10A ================= */

static bool motors_idle(void)
{
    float l, r;
    flora_motors_get_last(&l, &r);
    if (l != 0.0f || r != 0.0f) {
        return false;
    }
    hub_drive_state_t st;
    hub_drive_get_state(&st);
    return !st.active;
}

/** Wylacza klucz. Zaklada trzymana blokade. */
static void key_off_locked(void)
{
    if (!s_key || !flora_pwrsw_is_on(s_key)) {
        return;
    }
    /* Przy odcietym sterowniku jego wejscia maja byc w stanie niskim -
     * inaczej zasilalyby go przez diody zabezpieczajace. PWM jest juz
     * zerem (naped stoi), DIR ustawi od nowa najblizsze flora_motors_set. */
    gpio_set_level(FLORA_PIN_MOTOR_L_DIR, 0);
    gpio_set_level(FLORA_PIN_MOTOR_R_DIR, 0);
    flora_pwrsw_off(s_key);
    ESP_LOGI(TAG, "klucz MDD10A wylaczony");
}

void hub_power_drive_wake(void)
{
    if (!hub_autocfg_on(HUB_AC_MDD_KEY_EN) || !s_key_lock) {
        return;
    }
    xSemaphoreTake(s_key_lock, portMAX_DELAY);
    s_key_use_us = esp_timer_get_time();
    if (!s_key) {
        flora_pwrsw_cfg_t cfg = FLORA_PWRSW_DEFAULT_CFG();
        cfg.gpio       = FLORA_PIN_MDD_SWITCH;
        cfg.min_off_ms = 200;
        cfg.boot_ms    = KEY_BOOT_MS;
        if (flora_pwrsw_init(&cfg, &s_key) != ESP_OK) {
            s_key = NULL;
            ESP_LOGE(TAG, "klucz MDD10A niedostepny (GPIO %d)", FLORA_PIN_MDD_SWITCH);
        }
    }
    if (s_key && !flora_pwrsw_is_on(s_key)) {
        if (flora_pwrsw_on_wait(s_key) == ESP_OK) {
            flora_pwrsw_wait_boot(s_key);
            ESP_LOGI(TAG, "klucz MDD10A zalaczony");
        }
    }
    xSemaphoreGive(s_key_lock);
}

static void key_tick(void)
{
    if (!s_key_lock || !s_key) {
        return;
    }
    xSemaphoreTake(s_key_lock, portMAX_DELAY);
    if (!hub_autocfg_on(HUB_AC_MDD_KEY_EN)) {
        /* Przelacznik zdjety: oddajemy pin, jak przed wlaczeniem. */
        if (motors_idle()) {
            key_off_locked();
            flora_pwrsw_deinit(s_key);
            s_key = NULL;
        }
    } else if (flora_pwrsw_is_on(s_key) && motors_idle() &&
               esp_timer_get_time() - s_key_use_us > KEY_IDLE_S * 1000000LL) {
        key_off_locked();
    }
    xSemaphoreGive(s_key_lock);
}

/* ================= POLITYKA I SEN ================= */

static void load_cfg(flora_energy_cfg_t *c)
{
    *c = (flora_energy_cfg_t)FLORA_ENERGY_DEFAULT_CFG();
    c->v_save        = hub_autocfg_get(HUB_AC_E_V_SAVE);
    c->v_survival    = hub_autocfg_get(HUB_AC_E_V_SURV);
    c->sleep_night_s = (uint32_t)hub_autocfg_get(HUB_AC_E_NIGHT_S);
    c->sleep_day_s   = (uint32_t)hub_autocfg_get(HUB_AC_E_DAY_S);
}

static const char *mode_name(flora_energy_mode_t m)
{
    switch (m) {
    case FLORA_ENERGY_SAVE:     return "save";
    case FLORA_ENERGY_SURVIVAL: return "survival";
    default:                    return "normal";
    }
}

/** Ile sekund jeszcze trzeba byc na jawie; 0 = wolno spac. */
static uint32_t awake_left_s(void)
{
    const int64_t now = esp_timer_get_time();
    int64_t left = HUB_POWER_AWAKE_MIN_S * 1000000LL - now;
    if (s_activity_us) {
        const int64_t a = s_activity_us + HUB_POWER_AWAKE_AFTER_CMD_S * 1000000LL - now;
        if (a > left) {
            left = a;
        }
    }
    return left > 0 ? (uint32_t)(left / 1000000 + 1) : 0;
}

/**
 * Swiezo wgrana wersja czeka na potwierdzenie (hub_ota: minuta stabilnego
 * polaczenia z brokerem). Sen to restart, a restart przed potwierdzeniem
 * bootloader traktuje jak awarie i wraca do starej wersji.
 */
static bool ota_unconfirmed(void)
{
    esp_ota_img_states_t st;
    const esp_partition_t *run = esp_ota_get_running_partition();
    return run && esp_ota_get_state_partition(run, &st) == ESP_OK &&
           st == ESP_OTA_IMG_PENDING_VERIFY;
}

static bool quiet(void)
{
    return !ota_unconfirmed() && motors_idle() && !flora_pump_is_running() && hub_mission_may_sleep() &&
           !hub_ota_in_progress() && !hub_headcal_running() && !hub_rec_active();
}

static void go_to_sleep(uint32_t seconds, flora_energy_mode_t mode)
{
    ESP_LOGW(TAG, "zasypiam na %" PRIu32 " s (tryb %s)", seconds, mode_name(mode));
    flora_mqtt_publish_fmt("tele/sleep", 1, false, "{\"s\":%" PRIu32 ",\"mode\":\"%s\"}",
                           seconds, mode_name(mode));
    hub_drive_stop();
    flora_pump_stop();
    /* Chwila na wyslanie wiadomosci i logu przez MQTT. */
    vTaskDelay(pdMS_TO_TICKS(800));

    flora_pwrsw_handle_t mast = hub_telemetry_mast();
    if (mast) {
        flora_pwrsw_off(mast);
    }
    if (s_key_lock) {
        xSemaphoreTake(s_key_lock, portMAX_DELAY);
        key_off_locked();
        xSemaphoreGive(s_key_lock);
    }
    /* Piny kluczy, pompki i PWM maja rezystory do masy na plytce, wiec we
     * snie (piny w stanie wysokiej impedancji) wszystko zostaje wylaczone. */
    esp_sleep_enable_timer_wakeup((uint64_t)seconds * 1000000ULL);
    esp_deep_sleep_start();
}

void hub_power_tick(void)
{
    if (!s_started) {
        return;
    }
    key_tick();

    const bool sleep_en = hub_autocfg_on(HUB_AC_SLEEP_EN);
    const bool needed = sleep_en || hub_autocfg_on(HUB_AC_GUARD_EN) ||
                        hub_autocfg_on(HUB_AC_MISSION_EN) || hub_autocfg_on(HUB_AC_WATER_EN);
    if (!sleep_en && s_mast_forced_off) {
        /* Sen wylaczony: maszt, ktory sami zgasilismy, oddajemy. */
        flora_pwrsw_handle_t m = hub_telemetry_mast();
        if (!m || flora_pwrsw_on(m) == ESP_OK) {
            s_mast_forced_off = false;
            ESP_LOGW(TAG, "maszt zalaczony z powrotem");
        }
    }
    if (!needed) {
        /* Nic nie korzysta z polityki - nie ruszamy nawet magistrali I2C. */
        s_bus_v = NAN;
        return;
    }
    if (s_gen != hub_autocfg_generation()) {
        s_gen = hub_autocfg_generation();
        flora_energy_cfg_t cfg;
        load_cfg(&cfg);
        flora_energy_set_cfg(&s_energy, &cfg);
    }

    flora_energy_in_t in = { .dt_s = 1.0f };
    flora_ina226_handle_t ina = hub_telemetry_ina();
    flora_ina226_reading_t r;
    if (ina && flora_ina226_read(ina, &r) == ESP_OK) {
        s_bus_v = r.bus_voltage_v;
        s_bus_v_us = esp_timer_get_time();
        in.v_valid = true;
        in.bus_v = r.bus_voltage_v;
        in.load_high = fabsf(r.current_a) > LOAD_HIGH_A || !motors_idle() ||
                       flora_pump_is_running();
    }
    float elev, az;
    if (hub_power_sun(&elev, &az)) {
        in.night_known = true;
        in.night = elev < -3.0f;
    }
    const flora_energy_out_t o = flora_energy_update(&s_energy, &in);
    portENTER_CRITICAL(&s_mux);
    s_out = o;
    portEXIT_CRITICAL(&s_mux);

    static uint32_t tick;
    if (o.mode != s_last_mode) {
        ESP_LOGW(TAG, "tryb energii: %s -> %s (%.2f V)", mode_name(s_last_mode),
                 mode_name(o.mode), o.v_filt);
        s_last_mode = o.mode;
        tick = 0;
    }
    const uint32_t awake = sleep_en ? awake_left_s() : 0;
    if (tick++ % 10 == 0 && flora_mqtt_is_connected()) {
        char b[12] = "null";
        if (!isnan(o.v_filt)) {
            snprintf(b, sizeof b, "%.2f", o.v_filt);
        }
        flora_mqtt_publish_fmt("tele/energy", 0, false,
                               "{\"mode\":\"%s\",\"v_filt\":%s,\"drive\":%s,\"water\":%s,"
                               "\"sleep_s\":%" PRIu32 ",\"sleep_en\":%s,\"awake_s\":%" PRIu32 "}",
                               mode_name(o.mode), b,
                               o.allow_drive ? "true" : "false",
                               o.allow_water ? "true" : "false",
                               o.sleep_s, sleep_en ? "true" : "false", awake);
    }

    if (!sleep_en) {
        return;      /* sen wylaczony: masztu nie ruszamy i nie zasypiamy */
    }

    /* SURVIVAL: maszt wylaczony takze na jawie; po wyjsciu wraca. */
    flora_pwrsw_handle_t mast = hub_telemetry_mast();
    if (mast) {
        if (!o.mast_on && flora_pwrsw_is_on(mast)) {
            flora_pwrsw_off(mast);
            s_mast_forced_off = true;
            ESP_LOGW(TAG, "maszt wylaczony (tryb %s)", mode_name(o.mode));
        } else if (o.mast_on && s_mast_forced_off && flora_pwrsw_on(mast) == ESP_OK) {
            s_mast_forced_off = false;
            ESP_LOGW(TAG, "maszt zalaczony z powrotem");
        }
    }

    if (o.sleep_s > 0 && awake == 0 && quiet()) {
        go_to_sleep(o.sleep_s, o.mode);
    }
}

esp_err_t hub_power_start(void)
{
    if (s_started) {
        return ESP_ERR_INVALID_STATE;
    }
    flora_energy_cfg_t cfg;
    load_cfg(&cfg);
    flora_energy_init(&s_energy, &cfg);
    s_gen = hub_autocfg_generation();
    s_out = (flora_energy_out_t){ .mode = FLORA_ENERGY_NORMAL, .v_filt = NAN,
                                  .allow_drive = true, .allow_water = true, .mast_on = true };
    s_last_mode = FLORA_ENERGY_NORMAL;
    s_key_lock = xSemaphoreCreateMutex();
    if (!s_key_lock) {
        return ESP_ERR_NO_MEM;
    }
    if (esp_sleep_get_wakeup_causes() & (1u << ESP_SLEEP_WAKEUP_TIMER)) {
        ESP_LOGW(TAG, "wybudzenie ze snu; okno na komendy %d s", HUB_POWER_AWAKE_MIN_S);
    }
    s_started = true;
    return ESP_OK;
}
