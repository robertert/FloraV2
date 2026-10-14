#include <string.h>

#include "flora_encoders.h"
#include "driver/pulse_cnt.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "flora_enc";

typedef struct {
    pcnt_unit_handle_t    unit;
    pcnt_channel_handle_t chan_a;
    pcnt_channel_handle_t chan_b;
    bool                  invert;
} enc_t;

static struct {
    enc_t  left;
    enc_t  right;
    float  mm_per_count;
    bool   initialized;
} s_e;

static esp_err_t enc_setup(enc_t *e, const flora_encoder_cfg_t *cfg,
                           const flora_encoders_cfg_t *common)
{
    pcnt_unit_config_t unit_cfg = {
        .high_limit = common->high_limit,
        .low_limit  = common->low_limit,
        /* Bez tej flagi licznik zawijalby sie co ~16k zliczen i odometria
         * gubilaby obroty. Z nia sterownik sam dolicza przepelnienia. */
        .flags.accum_count = 1,
    };
    ESP_RETURN_ON_ERROR(pcnt_new_unit(&unit_cfg, &e->unit), TAG, "pcnt_new_unit");

    pcnt_glitch_filter_config_t filter = { .max_glitch_ns = common->glitch_filter_ns };
    ESP_RETURN_ON_ERROR(pcnt_unit_set_glitch_filter(e->unit, &filter), TAG, "glitch filter");

    if (common->enable_pullups) {
        ESP_RETURN_ON_ERROR(gpio_pullup_en(cfg->a_gpio), TAG, "pullup A");
        ESP_RETURN_ON_ERROR(gpio_pullup_en(cfg->b_gpio), TAG, "pullup B");
    }

    /* Dekodowanie x4: kanal A liczy zbocza na A z kierunkiem z poziomu B,
     * kanal B symetrycznie. Razem daje to 4 zliczenia na pelny okres. */
    pcnt_chan_config_t chan_a_cfg = { .edge_gpio_num = cfg->a_gpio, .level_gpio_num = cfg->b_gpio };
    ESP_RETURN_ON_ERROR(pcnt_new_channel(e->unit, &chan_a_cfg, &e->chan_a), TAG, "kanal A");
    pcnt_chan_config_t chan_b_cfg = { .edge_gpio_num = cfg->b_gpio, .level_gpio_num = cfg->a_gpio };
    ESP_RETURN_ON_ERROR(pcnt_new_channel(e->unit, &chan_b_cfg, &e->chan_b), TAG, "kanal B");

    ESP_RETURN_ON_ERROR(pcnt_channel_set_edge_action(e->chan_a,
                            PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                            PCNT_CHANNEL_EDGE_ACTION_INCREASE), TAG, "A edge");
    ESP_RETURN_ON_ERROR(pcnt_channel_set_level_action(e->chan_a,
                            PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                            PCNT_CHANNEL_LEVEL_ACTION_INVERSE), TAG, "A level");
    ESP_RETURN_ON_ERROR(pcnt_channel_set_edge_action(e->chan_b,
                            PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                            PCNT_CHANNEL_EDGE_ACTION_DECREASE), TAG, "B edge");
    ESP_RETURN_ON_ERROR(pcnt_channel_set_level_action(e->chan_b,
                            PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                            PCNT_CHANNEL_LEVEL_ACTION_INVERSE), TAG, "B level");

    /* Akumulacja dziala tylko wtedy, gdy na limitach sa punkty obserwacyjne. */
    ESP_RETURN_ON_ERROR(pcnt_unit_add_watch_point(e->unit, common->high_limit), TAG, "watch high");
    ESP_RETURN_ON_ERROR(pcnt_unit_add_watch_point(e->unit, common->low_limit),  TAG, "watch low");

    ESP_RETURN_ON_ERROR(pcnt_unit_enable(e->unit),      TAG, "pcnt_unit_enable");
    ESP_RETURN_ON_ERROR(pcnt_unit_clear_count(e->unit), TAG, "pcnt_unit_clear_count");
    ESP_RETURN_ON_ERROR(pcnt_unit_start(e->unit),       TAG, "pcnt_unit_start");

    e->invert = cfg->invert;
    return ESP_OK;
}

static void enc_teardown(enc_t *e)
{
    if (!e->unit) {
        return;
    }
    pcnt_unit_stop(e->unit);
    pcnt_unit_disable(e->unit);
    if (e->chan_a) { pcnt_del_channel(e->chan_a); e->chan_a = NULL; }
    if (e->chan_b) { pcnt_del_channel(e->chan_b); e->chan_b = NULL; }
    pcnt_del_unit(e->unit);
    e->unit = NULL;
}

static esp_err_t enc_count(const enc_t *e, int64_t *out)
{
    int raw = 0;
    ESP_RETURN_ON_ERROR(pcnt_unit_get_count(e->unit, &raw), TAG, "pcnt_unit_get_count");
    *out = e->invert ? -(int64_t)raw : (int64_t)raw;
    return ESP_OK;
}

esp_err_t flora_encoders_init(const flora_encoders_cfg_t *cfg)
{
    ESP_RETURN_ON_FALSE(cfg, ESP_ERR_INVALID_ARG, TAG, "cfg == NULL");
    ESP_RETURN_ON_FALSE(!s_e.initialized, ESP_ERR_INVALID_STATE, TAG, "juz zainicjalizowano");
    ESP_RETURN_ON_FALSE(cfg->high_limit > 0 && cfg->low_limit < 0,
                        ESP_ERR_INVALID_ARG, TAG,
                        "high_limit musi byc dodatni, low_limit ujemny");
    ESP_RETURN_ON_FALSE(cfg->mm_per_count > 0.0f, ESP_ERR_INVALID_ARG, TAG,
                        "mm_per_count musi byc dodatnie");

    memset(&s_e, 0, sizeof(s_e));
    s_e.mm_per_count = cfg->mm_per_count;

    esp_err_t ret = ESP_OK;
    ESP_GOTO_ON_ERROR(enc_setup(&s_e.left, &cfg->left, cfg),   fail, TAG, "enkoder lewy");
    ESP_GOTO_ON_ERROR(enc_setup(&s_e.right, &cfg->right, cfg), fail, TAG, "enkoder prawy");

    s_e.initialized = true;
    ESP_LOGI(TAG, "enkodery gotowe: L(A=%d,B=%d,inv=%d) P(A=%d,B=%d,inv=%d), %.4f mm/zliczenie",
             cfg->left.a_gpio, cfg->left.b_gpio, cfg->left.invert,
             cfg->right.a_gpio, cfg->right.b_gpio, cfg->right.invert,
             cfg->mm_per_count);
    return ESP_OK;

fail:
    enc_teardown(&s_e.left);
    enc_teardown(&s_e.right);
    return ret;
}

esp_err_t flora_encoders_deinit(void)
{
    ESP_RETURN_ON_FALSE(s_e.initialized, ESP_ERR_INVALID_STATE, TAG, "nie zainicjalizowano");
    enc_teardown(&s_e.left);
    enc_teardown(&s_e.right);
    s_e.initialized = false;
    return ESP_OK;
}

esp_err_t flora_encoders_read(flora_encoders_sample_t *out)
{
    ESP_RETURN_ON_FALSE(s_e.initialized, ESP_ERR_INVALID_STATE, TAG, "nie zainicjalizowano");
    ESP_RETURN_ON_FALSE(out, ESP_ERR_INVALID_ARG, TAG, "out == NULL");

    ESP_RETURN_ON_ERROR(enc_count(&s_e.left,  &out->left_counts),  TAG, "odczyt lewy");
    ESP_RETURN_ON_ERROR(enc_count(&s_e.right, &out->right_counts), TAG, "odczyt prawy");
    out->left_mm      = (float)out->left_counts  * s_e.mm_per_count;
    out->right_mm     = (float)out->right_counts * s_e.mm_per_count;
    out->timestamp_us = esp_timer_get_time();
    return ESP_OK;
}

esp_err_t flora_encoders_reset(void)
{
    ESP_RETURN_ON_FALSE(s_e.initialized, ESP_ERR_INVALID_STATE, TAG, "nie zainicjalizowano");
    ESP_RETURN_ON_ERROR(pcnt_unit_clear_count(s_e.left.unit),  TAG, "zerowanie lewego");
    ESP_RETURN_ON_ERROR(pcnt_unit_clear_count(s_e.right.unit), TAG, "zerowanie prawego");
    return ESP_OK;
}

void flora_encoders_speed_mm_s(const flora_encoders_sample_t *prev,
                               const flora_encoders_sample_t *now,
                               float *out_left, float *out_right)
{
    int64_t dt_us = now->timestamp_us - prev->timestamp_us;
    if (dt_us <= 0) {
        if (out_left)  *out_left  = 0.0f;
        if (out_right) *out_right = 0.0f;
        return;
    }
    float dt_s = (float)dt_us / 1000000.0f;
    if (out_left)  *out_left  = (now->left_mm  - prev->left_mm)  / dt_s;
    if (out_right) *out_right = (now->right_mm - prev->right_mm) / dt_s;
}
