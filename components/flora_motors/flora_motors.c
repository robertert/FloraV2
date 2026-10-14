#include <inttypes.h>
#include <math.h>
#include <string.h>

#include "flora_motors.h"
#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "flora_motors";

static struct {
    flora_motors_cfg_t cfg;
    uint32_t           duty_max;
    float              last_left;
    float              last_right;
    bool               initialized;
} s_m;

/** Przelicza moc -100..100 na wypelnienie i wystawia na jeden kanal. */
static esp_err_t motor_apply(const flora_motor_cfg_t *m, ledc_channel_t ch, float power)
{
    if (power > 100.0f)  power = 100.0f;
    if (power < -100.0f) power = -100.0f;

    bool forward = (power >= 0.0f);
    if (m->invert) {
        forward = !forward;
    }

    uint32_t duty = (uint32_t)lroundf((fabsf(power) / 100.0f) * (float)s_m.duty_max);
    if (duty > s_m.duty_max) {
        duty = s_m.duty_max;
    }

    /* Kierunek ustawiamy przed wypelnieniem: przy zmianie znaku unikamy
     * krotkiej chwili, w ktorej silnik dostaje pelna moc w starym kierunku. */
    ESP_RETURN_ON_ERROR(gpio_set_level(m->dir_gpio, forward ? 1 : 0),
                        TAG, "gpio_set_level(DIR=%d)", m->dir_gpio);
    ESP_RETURN_ON_ERROR(ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, duty),
                        TAG, "ledc_set_duty(ch=%d)", ch);
    ESP_RETURN_ON_ERROR(ledc_update_duty(LEDC_LOW_SPEED_MODE, ch),
                        TAG, "ledc_update_duty(ch=%d)", ch);
    return ESP_OK;
}

static esp_err_t dir_pin_init(gpio_num_t pin)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << pin,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,  /* spojne z rezystorem 10k na plytce */
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io), TAG, "gpio_config(DIR=%d)", pin);
    return gpio_set_level(pin, 0);
}

static esp_err_t pwm_channel_init(gpio_num_t pin, ledc_channel_t ch, ledc_timer_t timer)
{
    ledc_channel_config_t c = {
        .gpio_num   = pin,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = ch,
        .timer_sel  = timer,
        .duty       = 0,        /* start zawsze od zatrzymanego silnika */
        .hpoint     = 0,
    };
    return ledc_channel_config(&c);
}

esp_err_t flora_motors_init(const flora_motors_cfg_t *cfg)
{
    ESP_RETURN_ON_FALSE(cfg, ESP_ERR_INVALID_ARG, TAG, "cfg == NULL");
    ESP_RETURN_ON_FALSE(!s_m.initialized, ESP_ERR_INVALID_STATE, TAG,
                        "flora_motors_init juz wywolane");

    /* Zrodlem LEDC jest APB 80 MHz. Iloczyn czestotliwosci i liczby krokow
     * musi sie w nim zmiescic, inaczej ledc_timer_config zwroci blad
     * dopiero w runtime - lepiej zlapac to tutaj z czytelnym komunikatem. */
    uint32_t steps = 1UL << (uint32_t)cfg->pwm_resolution;
    ESP_RETURN_ON_FALSE((uint64_t)cfg->pwm_freq_hz * steps <= 80000000ULL,
                        ESP_ERR_INVALID_ARG, TAG,
                        "PWM %" PRIu32 " Hz przy %" PRIu32 " krokach nie miesci sie w 80 MHz APB",
                        cfg->pwm_freq_hz, steps);

    memset(&s_m, 0, sizeof(s_m));
    s_m.cfg      = *cfg;
    s_m.duty_max = steps - 1;

    ledc_timer_config_t t = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = cfg->pwm_resolution,
        .timer_num       = cfg->timer,
        .freq_hz         = cfg->pwm_freq_hz,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&t), TAG, "ledc_timer_config");

    ESP_RETURN_ON_ERROR(dir_pin_init(cfg->left.dir_gpio),  TAG, "DIR lewy");
    ESP_RETURN_ON_ERROR(dir_pin_init(cfg->right.dir_gpio), TAG, "DIR prawy");

    ESP_RETURN_ON_ERROR(pwm_channel_init(cfg->left.pwm_gpio, cfg->channel_left, cfg->timer),
                        TAG, "PWM lewy");
    ESP_RETURN_ON_ERROR(pwm_channel_init(cfg->right.pwm_gpio, cfg->channel_right, cfg->timer),
                        TAG, "PWM prawy");

    s_m.initialized = true;
    ESP_LOGI(TAG, "naped gotowy: PWM %" PRIu32 " Hz, %" PRIu32 " krokow, L(pwm=%d,dir=%d,inv=%d) P(pwm=%d,dir=%d,inv=%d)",
             cfg->pwm_freq_hz, steps,
             cfg->left.pwm_gpio, cfg->left.dir_gpio, cfg->left.invert,
             cfg->right.pwm_gpio, cfg->right.dir_gpio, cfg->right.invert);
    return ESP_OK;
}

esp_err_t flora_motors_deinit(void)
{
    ESP_RETURN_ON_FALSE(s_m.initialized, ESP_ERR_INVALID_STATE, TAG, "nie zainicjalizowano");
    flora_motors_stop();
    ledc_stop(LEDC_LOW_SPEED_MODE, s_m.cfg.channel_left, 0);
    ledc_stop(LEDC_LOW_SPEED_MODE, s_m.cfg.channel_right, 0);
    s_m.initialized = false;
    return ESP_OK;
}

esp_err_t flora_motors_set(float left, float right)
{
    ESP_RETURN_ON_FALSE(s_m.initialized, ESP_ERR_INVALID_STATE, TAG, "nie zainicjalizowano");
    ESP_RETURN_ON_ERROR(motor_apply(&s_m.cfg.left, s_m.cfg.channel_left, left),
                        TAG, "silnik lewy");
    ESP_RETURN_ON_ERROR(motor_apply(&s_m.cfg.right, s_m.cfg.channel_right, right),
                        TAG, "silnik prawy");
    s_m.last_left  = left;
    s_m.last_right = right;
    return ESP_OK;
}

esp_err_t flora_motors_stop(void)
{
    return flora_motors_set(0.0f, 0.0f);
}

void flora_motors_get_last(float *out_left, float *out_right)
{
    if (out_left)  *out_left  = s_m.last_left;
    if (out_right) *out_right = s_m.last_right;
}
