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

        case FLORA_CMD_SET_SUN_GAIN:
            s_sun_gain     = (uint8_t)c->arg;
            s_req_sun_gain = true;
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
    s_clock_offset_us = (int64_t)t->bottom_us - esp_timer_get_time();
    ESP_LOGI(TAG, "synchronizacja zegara, przesuniecie %" PRId64 " us",
             s_clock_offset_us);
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
