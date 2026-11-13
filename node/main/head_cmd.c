#include <inttypes.h>

#include "head_cmd.h"
#include "head_math.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "flora_msgs.h"

static const char *TAG = "head_cmd";

/* Flagi sa pisane z zadania odbiorczego i czytane z zadan roboczych.
 * volatile wystarcza, bo kazda jest jednym slowem i nie ma miedzy nimi
 * zaleznosci - nie budujemy tu zadnej niepodzielnej calosci. */
static volatile bool     s_req_gyro_cal;
static volatile bool     s_req_att_hz;
static volatile uint8_t  s_att_hz;
static volatile bool     s_req_sun_gain;
static volatile uint8_t  s_sun_gain;
static volatile int64_t  s_clock_offset_us;
static volatile bool     s_time_synced;
static volatile bool     s_req_mag_cal;
static volatile uint32_t s_mag_cal_s;
static volatile bool     s_req_level;
static volatile uint8_t  s_level_step;
static volatile bool     s_hub_still;
static volatile int64_t  s_hub_still_us;

#define HUB_STILL_TIMEOUT_US 2000000   /**< bryla odswieza co 1 s */

static void on_cmd(uint8_t type, const void *payload, uint8_t len, void *ctx)
{
    if (len != sizeof(flora_cmd_t)) {
        return;
    }
    const flora_cmd_t *c = payload;

    switch (c->code) {
        case FLORA_CMD_NOP:
            break;

        case FLORA_CMD_REBOOT:
            /* Jedyna komenda wykonywana na miejscu - i tak nie ma do czego
             * wracac. Potwierdzenie poszlo juz przed wejsciem do handlera. */
            ESP_LOGW(TAG, "restart na zadanie bryly");
            esp_restart();
            break;

        case FLORA_CMD_CALIBRATE_GYRO:
            ESP_LOGI(TAG, "zadanie kalibracji zyroskopu");
            s_req_gyro_cal = true;
            break;

        case FLORA_CMD_SET_ATT_HZ:
            if (c->arg >= 1 && c->arg <= 60) {
                s_att_hz     = (uint8_t)c->arg;
                s_req_att_hz = true;
                ESP_LOGI(TAG, "zadanie tempa ATTITUDE: %" PRIu32 " Hz", c->arg);
            } else {
                ESP_LOGW(TAG, "tempo %" PRIu32 " Hz poza zakresem 1-60", c->arg);
            }
            break;

        case FLORA_CMD_CALIBRATE_MAG:
            if (c->arg >= 10 && c->arg <= 120) {
                s_mag_cal_s   = c->arg;
                s_req_mag_cal = true;
                ESP_LOGI(TAG, "zadanie kalibracji magnetometru: %" PRIu32 " s", c->arg);
            } else {
                ESP_LOGW(TAG, "kalibracja magnetometru: %" PRIu32 " s poza zakresem 10-120", c->arg);
            }
            break;

        case FLORA_CMD_CALIBRATE_LEVEL:
            if (c->arg <= 2) {
                s_level_step = (uint8_t)c->arg;
                s_req_level  = true;
                ESP_LOGI(TAG, "zadanie kalibracji poziomu, krok %" PRIu32, c->arg);
            }
            break;

        case FLORA_CMD_SET_SUN_GAIN:
            s_sun_gain     = (uint8_t)c->arg;
            s_req_sun_gain = true;
            break;

        case FLORA_CMD_SET_STILL:
            /* Co sekunde - bez logu. */
            s_hub_still    = c->arg != 0;
            s_hub_still_us = esp_timer_get_time();
            break;

        default:
            ESP_LOGW(TAG, "nieznana komenda 0x%02X", c->code);
            break;
    }
}

static void on_time_sync(uint8_t type, const void *payload, uint8_t len, void *ctx)
{
    if (len != sizeof(flora_time_sync_t)) {
        return;
    }
    const flora_time_sync_t *t = payload;
    /* Roznica zegarow, zeby znaczniki czasu w ATTITUDE dalo sie przelozyc
     * na os czasu bryly. Nie korygujemy wlasnego zegara - monotonicznosc
     * jest wazniejsza dla calkowania zyroskopu niz zgodnosc bezwzgledna. */
    int64_t off = (int64_t)t->bottom_us - esp_timer_get_time();
    /* Zegar bryly jest 64-bitowy, a po kablu idzie mlodsze 32 bity - wiec
     * przesuniecie ma sens tylko modulo 2^32. Tak tez uzywamy go nizej. */
    if (!s_time_synced) {
        ESP_LOGI(TAG, "synchronizacja zegara, przesuniecie %" PRId64 " us", off);
    }
    s_clock_offset_us = off;
    s_time_synced     = true;
}

void head_cmd_register(flora_link_handle_t down)
{
    flora_link_on(down, FLORA_MSG_CMD, on_cmd, NULL);
    flora_link_on(down, FLORA_MSG_TIME_SYNC, on_time_sync, NULL);
}

bool head_cmd_take_gyro_cal(void)
{
    if (!s_req_gyro_cal) {
        return false;
    }
    s_req_gyro_cal = false;
    return true;
}

bool head_cmd_take_att_hz(uint8_t *hz)
{
    if (!s_req_att_hz) {
        return false;
    }
    *hz = s_att_hz;
    s_req_att_hz = false;
    return true;
}

bool head_cmd_take_sun_gain(uint8_t *gain_index)
{
    if (!s_req_sun_gain) {
        return false;
    }
    *gain_index = s_sun_gain;
    s_req_sun_gain = false;
    return true;
}

int64_t head_cmd_clock_offset_us(void) { return s_clock_offset_us; }

bool head_cmd_time_synced(void) { return s_time_synced; }

uint32_t head_cmd_stamp(int64_t local_us)
{
    return s_time_synced ? (uint32_t)(local_us + s_clock_offset_us) : (uint32_t)local_us;
}

bool head_cmd_hub_still(void)
{
    return s_hub_still && esp_timer_get_time() - s_hub_still_us < HUB_STILL_TIMEOUT_US;
}

bool head_cmd_take_level_cal(uint8_t *step)
{
    if (!s_req_level) {
        return false;
    }
    *step = s_level_step;
    s_req_level = false;
    return true;
}

bool head_cmd_take_mag_cal(uint32_t *seconds)
{
    if (!s_req_mag_cal) {
        return false;
    }
    *seconds = s_mag_cal_s;
    s_req_mag_cal = false;
    return true;
}
