#include <inttypes.h>
#include <math.h>
#include <string.h>

#include "flora_motors.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "flora_motors";

typedef struct {
    int64_t  last_counts;
    uint32_t idle_ms;      /**< ile trwa juz bezruch mimo zadanej mocy */
    bool     primed;       /**< mamy juz pierwszy odczyt do porownania */
} stall_wheel_t;

static struct {
    flora_motors_cfg_t cfg;
    uint32_t           duty_max;
    float              last_left;
    float              last_right;
    bool               initialized;

    /* --- straznik utyku --- */
    SemaphoreHandle_t        lock;
    flora_motors_stall_cfg_t guard;
    esp_timer_handle_t       guard_timer;
    stall_wheel_t            wheel[2];
    bool                     guard_on;
    bool                     stalled;
    flora_motors_wheel_t     stalled_wheel;
    flora_motors_stall_reason_t stall_reason;
    uint32_t                 over_ms;     /**< jak dlugo prad jest ponad progiem */
} s_m;

/** Wystawia moc na kanal bez zadnych sprawdzen. Zaklada trzymana blokade. */
static esp_err_t motor_apply(const flora_motor_cfg_t *m, ledc_channel_t ch, float power);

/** Odcina oba silniki. Uzywane przez straznika, pomija blokade stanu. */
static void cut_power(void)
{
    motor_apply(&s_m.cfg.left,  s_m.cfg.channel_left,  0.0f);
    motor_apply(&s_m.cfg.right, s_m.cfg.channel_right, 0.0f);
    s_m.last_left  = 0.0f;
    s_m.last_right = 0.0f;
}

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

/* ================= STRAZNIK UTYKU ================= */

/**
 * Sprawdza jedno kolo. Zwraca true, gdy wykryto utyk.
 * Wolane z zadania timera, przy trzymanej blokadzie.
 */
static bool guard_check_wheel(flora_motors_wheel_t which, float power)
{
    stall_wheel_t *w = &s_m.wheel[which];

    /* Przy malej mocy silnik ma prawo stac - nie da rady pokonac tarcia
     * spoczynkowego i nie jest to usterka. Liczniki zerujemy, zeby po
     * powrocie do wyzszej mocy zaczac odliczanie od nowa. */
    if (fabsf(power) < s_m.guard.min_power) {
        w->idle_ms = 0;
        w->primed  = false;
        return false;
    }

    int64_t now = s_m.guard.counts_cb(which, s_m.guard.counts_ctx);

    if (!w->primed) {
        w->last_counts = now;
        w->primed      = true;
        w->idle_ms     = 0;
        return false;
    }

    int64_t moved = now - w->last_counts;
    if (moved < 0) {
        moved = -moved;
    }
    w->last_counts = now;

    if (moved >= s_m.guard.min_counts) {
        w->idle_ms = 0;
        return false;
    }

    w->idle_ms += s_m.guard.check_ms;
    return w->idle_ms >= s_m.guard.grace_ms;
}

/**
 * Sprawdza prad. Zwraca true przy utyku.
 *
 * Liczymy czas ponad progiem zamiast reagowac na pierwsza probke: impuls
 * rozruchowy silnika jest krotki i sam w sobie nie jest usterka.
 */
static bool guard_check_current(float *out_amps)
{
    if (!s_m.guard.current_cb) {
        return false;
    }
    float a = s_m.guard.current_cb(s_m.guard.current_ctx);
    *out_amps = a;

    /* NAN znaczy, ze pomiar jest chwilowo niedostepny - wtedy ten detektor
     * po prostu milczy, zamiast zglaszac falszywy alarm. */
    if (isnan(a) || a < s_m.guard.stall_current_a) {
        s_m.over_ms = 0;
        return false;
    }
    s_m.over_ms += s_m.guard.check_ms;
    return s_m.over_ms >= s_m.guard.current_grace_ms;
}

static void guard_timer_cb(void *arg)
{
    flora_motors_stall_cb_t cb = NULL;
    void *ctx = NULL;
    flora_motors_wheel_t which = FLORA_MOTORS_WHEEL_LEFT;
    float power = 0.0f;
    bool fire = false;

    flora_motors_stall_reason_t reason = FLORA_MOTORS_STALL_NO_MOTION;

    xSemaphoreTake(s_m.lock, portMAX_DELAY);
    if (s_m.guard_on && !s_m.stalled) {
        /* Prad sprawdzamy jako pierwszy: jest niezalezny od enkoderow,
         * wiec lapie takze przypadek, w ktorym to ENKODER jest zepsuty. */
        float amps = 0.0f;
        if (guard_check_current(&amps)) {
            reason = FLORA_MOTORS_STALL_OVERCURRENT;
            power  = amps;
            fire   = true;
        } else if (guard_check_wheel(FLORA_MOTORS_WHEEL_LEFT, s_m.last_left)) {
            which = FLORA_MOTORS_WHEEL_LEFT;
            power = s_m.last_left;
            fire  = true;
        } else if (guard_check_wheel(FLORA_MOTORS_WHEEL_RIGHT, s_m.last_right)) {
            which = FLORA_MOTORS_WHEEL_RIGHT;
            power = s_m.last_right;
            fire  = true;
        }

        if (fire) {
            /* Odcinamy OBA silniki. Zatrzymanie samego zablokowanego
             * zostawiloby robota krecacego sie w kolko na drugim kole. */
            cut_power();
            s_m.stalled       = true;
            s_m.stalled_wheel = which;
            s_m.stall_reason  = reason;
            s_m.over_ms       = 0;
            cb  = s_m.guard.on_stall;
            ctx = s_m.guard.stall_ctx;
        }
    }
    xSemaphoreGive(s_m.lock);

    if (fire) {
        if (reason == FLORA_MOTORS_STALL_OVERCURRENT) {
            ESP_LOGE(TAG, "UTYK (prad): %.2f A ponad progiem %.2f A przez %" PRIu32 " ms. "
                          "Oba silniki odciete, blokada zatrzasnieta.",
                     power, s_m.guard.stall_current_a, s_m.guard.current_grace_ms);
            ESP_LOGE(TAG, "  pomiar obejmuje cala szyne, wiec zrodlem moze byc tez "
                          "pompka albo maszt - sprawdz, co bylo zalaczone");
        } else {
            ESP_LOGE(TAG, "UTYK (enkoder): %s kolo nie rusza przy mocy %.0f%% przez "
                          "%" PRIu32 " ms. Oba silniki odciete, blokada zatrzasnieta.",
                     which == FLORA_MOTORS_WHEEL_LEFT ? "lewe" : "prawe",
                     power, s_m.guard.grace_ms);
        }
        if (cb) {
            cb(reason, which, power, ctx);
        }
    }
}

esp_err_t flora_motors_stall_guard_enable(const flora_motors_stall_cfg_t *cfg)
{
    ESP_RETURN_ON_FALSE(s_m.initialized, ESP_ERR_INVALID_STATE, TAG, "nie zainicjalizowano");
    ESP_RETURN_ON_FALSE(cfg && cfg->counts_cb, ESP_ERR_INVALID_ARG, TAG,
                        "counts_cb jest wymagany - bez zrodla zliczen nie ma czego pilnowac");
    ESP_RETURN_ON_FALSE(cfg->check_ms > 0 && cfg->grace_ms >= cfg->check_ms,
                        ESP_ERR_INVALID_ARG, TAG,
                        "grace_ms (%" PRIu32 ") musi byc nie mniejsze niz check_ms (%" PRIu32 ")",
                        cfg->grace_ms, cfg->check_ms);
    ESP_RETURN_ON_FALSE(!s_m.guard_on, ESP_ERR_INVALID_STATE, TAG, "straznik juz dziala");
    ESP_RETURN_ON_FALSE(!cfg->current_cb ||
                        (cfg->stall_current_a > 0.0f && cfg->current_grace_ms >= cfg->check_ms),
                        ESP_ERR_INVALID_ARG, TAG,
                        "detektor pradowy wymaga dodatniego progu i current_grace_ms "
                        "nie mniejszego niz check_ms");

    xSemaphoreTake(s_m.lock, portMAX_DELAY);
    s_m.guard   = *cfg;
    s_m.stalled = false;
    memset(s_m.wheel, 0, sizeof(s_m.wheel));
    xSemaphoreGive(s_m.lock);

    const esp_timer_create_args_t args = {
        .callback = guard_timer_cb,
        .name     = "motor_stall",
    };
    ESP_RETURN_ON_ERROR(esp_timer_create(&args, &s_m.guard_timer), TAG, "timer straznika");
    ESP_RETURN_ON_ERROR(esp_timer_start_periodic(s_m.guard_timer,
                                                 (uint64_t)cfg->check_ms * 1000),
                        TAG, "start timera straznika");
    s_m.guard_on = true;

    ESP_LOGI(TAG, "straznik utyku: co %" PRIu32 " ms; enkodery - tolerancja %" PRIu32 " ms, "
                  "prog mocy %.0f%%, prog ruchu %" PRId64 " zliczen",
             cfg->check_ms, cfg->grace_ms, cfg->min_power, cfg->min_counts);
    if (cfg->current_cb) {
        ESP_LOGI(TAG, "  drugi detektor: prad powyzej %.2f A przez %" PRIu32 " ms",
                 cfg->stall_current_a, cfg->current_grace_ms);
    } else {
        ESP_LOGW(TAG, "  detektor pradowy WYLACZONY - przy awarii enkodera naped "
                      "zostanie bez ochrony");
    }
    return ESP_OK;
}

esp_err_t flora_motors_stall_guard_disable(void)
{
    ESP_RETURN_ON_FALSE(s_m.guard_on, ESP_ERR_INVALID_STATE, TAG, "straznik nie dziala");
    esp_timer_stop(s_m.guard_timer);
    esp_timer_delete(s_m.guard_timer);
    s_m.guard_timer = NULL;
    s_m.guard_on    = false;
    return ESP_OK;
}

bool flora_motors_is_stalled(void)
{
    return s_m.initialized && s_m.stalled;
}

flora_motors_wheel_t flora_motors_stalled_wheel(void)
{
    return s_m.stalled_wheel;
}

flora_motors_stall_reason_t flora_motors_stall_reason(void)
{
    return s_m.stall_reason;
}

esp_err_t flora_motors_clear_stall(void)
{
    ESP_RETURN_ON_FALSE(s_m.initialized, ESP_ERR_INVALID_STATE, TAG, "nie zainicjalizowano");
    xSemaphoreTake(s_m.lock, portMAX_DELAY);
    s_m.stalled = false;
    s_m.over_ms = 0;
    memset(s_m.wheel, 0, sizeof(s_m.wheel));
    xSemaphoreGive(s_m.lock);
    ESP_LOGW(TAG, "blokada po utyku zdjeta");
    return ESP_OK;
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

    /* Blokada chroni stan dzielony miedzy zadaniem aplikacji
     * (flora_motors_set) a zadaniem timera (straznik utyku). */
    s_m.lock = xSemaphoreCreateMutex();
    ESP_RETURN_ON_FALSE(s_m.lock, ESP_ERR_NO_MEM, TAG, "brak pamieci na mutex");

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
    if (s_m.guard_on) {
        flora_motors_stall_guard_disable();
    }
    flora_motors_stop();
    vSemaphoreDelete(s_m.lock);
    s_m.lock = NULL;
    ledc_stop(LEDC_LOW_SPEED_MODE, s_m.cfg.channel_left, 0);
    ledc_stop(LEDC_LOW_SPEED_MODE, s_m.cfg.channel_right, 0);
    s_m.initialized = false;
    return ESP_OK;
}

esp_err_t flora_motors_set(float left, float right)
{
    ESP_RETURN_ON_FALSE(s_m.initialized, ESP_ERR_INVALID_STATE, TAG, "nie zainicjalizowano");

    xSemaphoreTake(s_m.lock, portMAX_DELAY);
    esp_err_t ret = ESP_OK;

    /* Po zatrzasnietej blokadzie przepuszczamy wylacznie zatrzymanie.
     * Inaczej petla sterowania, ktora nie sprawdza bledu, podawalaby moc
     * na zablokowany silnik w kolko - czyli dokladnie to, przed czym
     * straznik ma chronic. */
    if (s_m.stalled && (left != 0.0f || right != 0.0f)) {
        xSemaphoreGive(s_m.lock);
        ESP_LOGW(TAG, "blokada po utyku - odrzucam zadanie mocy. "
                      "Usun przyczyne i wywolaj flora_motors_clear_stall()");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_GOTO_ON_ERROR(motor_apply(&s_m.cfg.left, s_m.cfg.channel_left, left),
                      out, TAG, "silnik lewy");
    ESP_GOTO_ON_ERROR(motor_apply(&s_m.cfg.right, s_m.cfg.channel_right, right),
                      out, TAG, "silnik prawy");
    s_m.last_left  = left;
    s_m.last_right = right;

out:
    xSemaphoreGive(s_m.lock);
    return ret;
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
