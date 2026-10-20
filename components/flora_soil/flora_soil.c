#include <string.h>

#include "flora_soil.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "flora_soil";

static struct {
    adc_oneshot_unit_handle_t unit;
    adc_cali_handle_t         cali;
    adc_channel_t             channel;
    adc_unit_t                unit_id;
    int                       samples;
    int                       raw_dry;
    int                       raw_wet;
    bool                      has_cali;
    bool                      initialized;
} s_s;

esp_err_t flora_soil_init(const flora_soil_cfg_t *cfg)
{
    ESP_RETURN_ON_FALSE(cfg, ESP_ERR_INVALID_ARG, TAG, "cfg == NULL");
    ESP_RETURN_ON_FALSE(!s_s.initialized, ESP_ERR_INVALID_STATE, TAG, "juz zainicjalizowano");
    ESP_RETURN_ON_FALSE(cfg->samples > 0, ESP_ERR_INVALID_ARG, TAG, "samples musi byc > 0");

    memset(&s_s, 0, sizeof(s_s));

    /* Mapowanie pinu na kanal zostawiamy sterownikowi - recznie przepisane
     * tablice kanalow to klasyczne zrodlo pomylek przy zmianie ukladu. */
    ESP_RETURN_ON_ERROR(adc_oneshot_io_to_channel(cfg->gpio, &s_s.unit_id, &s_s.channel),
                        TAG, "GPIO%d nie jest wejsciem ADC", cfg->gpio);
    ESP_RETURN_ON_FALSE(s_s.unit_id == ADC_UNIT_1, ESP_ERR_INVALID_ARG, TAG,
                        "GPIO%d lezy na ADC2, ktory koliduje z WiFi - uzyj pinu z ADC1",
                        cfg->gpio);

    adc_oneshot_unit_init_cfg_t unit_cfg = { .unit_id = s_s.unit_id };
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&unit_cfg, &s_s.unit), TAG, "adc_oneshot_new_unit");

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten    = (adc_atten_t)cfg->atten,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    esp_err_t ret = ESP_OK;
    ESP_GOTO_ON_ERROR(adc_oneshot_config_channel(s_s.unit, s_s.channel, &chan_cfg),
                      fail, TAG, "konfiguracja kanalu");

    /* Kalibracja fabryczna: bez niej mamy tylko wartosci surowe. Na ESP32-S3
     * dostepny jest schemat krzywej. Brak kalibracji nie jest bledem
     * krytycznym - procenty licza sie z wartosci surowych. */
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id  = s_s.unit_id,
        .chan     = s_s.channel,
        .atten    = (adc_atten_t)cfg->atten,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_s.cali) == ESP_OK) {
        s_s.has_cali = true;
    } else {
        ESP_LOGW(TAG, "brak fabrycznej kalibracji ADC - miliwolty beda niedostepne");
    }

    s_s.samples = cfg->samples;
    s_s.raw_dry = cfg->raw_dry;
    s_s.raw_wet = cfg->raw_wet;
    s_s.initialized = true;

    ESP_LOGI(TAG, "gotowe: GPIO%d = ADC%d kanal %d, %d probek%s",
             cfg->gpio, s_s.unit_id + 1, s_s.channel, s_s.samples,
             s_s.raw_dry != s_s.raw_wet ? ", kalibracja ustawiona" : ", BRAK kalibracji procentow");
    return ESP_OK;

fail:
    adc_oneshot_del_unit(s_s.unit);
    memset(&s_s, 0, sizeof(s_s));
    return ret;
}

esp_err_t flora_soil_deinit(void)
{
    ESP_RETURN_ON_FALSE(s_s.initialized, ESP_ERR_INVALID_STATE, TAG, "nie zainicjalizowano");
    if (s_s.has_cali) {
        adc_cali_delete_scheme_curve_fitting(s_s.cali);
    }
    adc_oneshot_del_unit(s_s.unit);
    memset(&s_s, 0, sizeof(s_s));
    return ESP_OK;
}

esp_err_t flora_soil_read_raw(int *raw)
{
    ESP_RETURN_ON_FALSE(s_s.initialized, ESP_ERR_INVALID_STATE, TAG, "nie zainicjalizowano");
    ESP_RETURN_ON_FALSE(raw, ESP_ERR_INVALID_ARG, TAG, "raw == NULL");

    int32_t sum = 0;
    for (int i = 0; i < s_s.samples; i++) {
        int one = 0;
        ESP_RETURN_ON_ERROR(adc_oneshot_read(s_s.unit, s_s.channel, &one),
                            TAG, "adc_oneshot_read");
        sum += one;
    }
    *raw = (int)(sum / s_s.samples);
    return ESP_OK;
}

esp_err_t flora_soil_read_mv(int *mv)
{
    ESP_RETURN_ON_FALSE(s_s.initialized, ESP_ERR_INVALID_STATE, TAG, "nie zainicjalizowano");
    ESP_RETURN_ON_FALSE(mv, ESP_ERR_INVALID_ARG, TAG, "mv == NULL");
    ESP_RETURN_ON_FALSE(s_s.has_cali, ESP_ERR_NOT_SUPPORTED, TAG,
                        "brak fabrycznej kalibracji ADC");

    int raw = 0;
    ESP_RETURN_ON_ERROR(flora_soil_read_raw(&raw), TAG, "odczyt surowy");
    return adc_cali_raw_to_voltage(s_s.cali, raw, mv);
}

esp_err_t flora_soil_read_percent(float *percent)
{
    ESP_RETURN_ON_FALSE(s_s.initialized, ESP_ERR_INVALID_STATE, TAG, "nie zainicjalizowano");
    ESP_RETURN_ON_FALSE(percent, ESP_ERR_INVALID_ARG, TAG, "percent == NULL");
    ESP_RETURN_ON_FALSE(s_s.raw_dry != s_s.raw_wet, ESP_ERR_INVALID_STATE, TAG,
                        "brak kalibracji: zmierz raw w powietrzu i w wodzie, "
                        "potem flora_soil_set_calibration()");

    int raw = 0;
    ESP_RETURN_ON_ERROR(flora_soil_read_raw(&raw), TAG, "odczyt surowy");

    /* Czujnik pojemnosciowy: sucho = wyzsze napiecie, wiec skala jest odwrocona. */
    float span = (float)(s_s.raw_dry - s_s.raw_wet);
    float p = ((float)(s_s.raw_dry - raw) / span) * 100.0f;

    if (p < 0.0f)   p = 0.0f;
    if (p > 100.0f) p = 100.0f;
    *percent = p;
    return ESP_OK;
}

esp_err_t flora_soil_set_calibration(int raw_dry, int raw_wet)
{
    ESP_RETURN_ON_FALSE(s_s.initialized, ESP_ERR_INVALID_STATE, TAG, "nie zainicjalizowano");
    ESP_RETURN_ON_FALSE(raw_dry != raw_wet, ESP_ERR_INVALID_ARG, TAG,
                        "punkty kalibracji nie moga byc rowne");
    s_s.raw_dry = raw_dry;
    s_s.raw_wet = raw_wet;
    ESP_LOGI(TAG, "kalibracja: sucho %d, mokro %d", raw_dry, raw_wet);
    return ESP_OK;
}

bool flora_soil_has_voltage_calibration(void)
{
    return s_s.initialized && s_s.has_cali;
}
