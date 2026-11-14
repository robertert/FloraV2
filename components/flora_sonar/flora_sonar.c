#include <inttypes.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "flora_sonar.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"

static const char *TAG = "flora_sonar";

typedef struct {
    gpio_num_t        gpio;
    volatile int64_t  rise_us;
    volatile int64_t  fall_us;
    SemaphoreHandle_t done;
} echo_t;

static struct {
    flora_sonar_cfg_t cfg;
    echo_t   echo[FLORA_SONAR_MAX_SENSORS];
    int64_t  last_measure_us;
    bool     isr_service_ours;
    bool     initialized;
} s_n;

/**
 * Przerwanie na obu zboczach ECHO. Znacznik czasu zdejmujemy tutaj, zeby
 * wynik nie zalezal od tego, kiedy zadanie dostanie procesor.
 * Bez logowania i bez alokacji - to kod przerwania.
 */
static void IRAM_ATTR echo_isr(void *arg)
{
    echo_t *e = (echo_t *)arg;
    int64_t now = esp_timer_get_time();

    if (gpio_get_level(e->gpio)) {
        if (e->fall_us == 0) {
            e->rise_us = now;
        }
    } else if (e->rise_us != 0 && e->fall_us == 0) {
        /* Tylko zbocze opadajace PO naszym narastajacym. Osierocone (koniec
         * impulsu, ktory zaczal sie przed strzalem - np. echo poprzedniego
         * strzalu przy wspolnym TRIG) ignorujemy, zamiast budzic pomiar. */
        e->fall_us = now;
        BaseType_t woken = pdFALSE;
        xSemaphoreGiveFromISR(e->done, &woken);
        if (woken == pdTRUE) {
            portYIELD_FROM_ISR();
        }
    }
}

float flora_sonar_speed_of_sound(void)
{
    /* Predkosc dzwieku rosnie o ok. 0,6 m/s na kazdy stopien. Miedzy zima
     * a latem to roznica ponad 5%, czyli kilkanascie cm na 3 metrach -
     * dlatego korekta ma sens przy czujniku pracujacym na dworze. */
    return 331.3f + 0.606f * s_n.cfg.temperature_c;
}

void flora_sonar_set_temperature(float celsius)
{
    s_n.cfg.temperature_c = celsius;
}

esp_err_t flora_sonar_init(const flora_sonar_cfg_t *cfg)
{
    ESP_RETURN_ON_FALSE(cfg, ESP_ERR_INVALID_ARG, TAG, "cfg == NULL");
    ESP_RETURN_ON_FALSE(!s_n.initialized, ESP_ERR_INVALID_STATE, TAG, "juz zainicjalizowano");
    ESP_RETURN_ON_FALSE(cfg->count > 0 && cfg->count <= FLORA_SONAR_MAX_SENSORS,
                        ESP_ERR_INVALID_ARG, TAG, "count musi byc w zakresie 1-%d",
                        FLORA_SONAR_MAX_SENSORS);

    memset(&s_n, 0, sizeof(s_n));
    s_n.cfg = *cfg;

    gpio_config_t trig_io = {
        .pin_bit_mask = 1ULL << cfg->trig_gpio,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&trig_io), TAG, "konfiguracja TRIG");
    gpio_set_level(cfg->trig_gpio, 0);

    uint64_t echo_mask = 0;
    for (int i = 0; i < cfg->count; i++) {
        echo_mask |= 1ULL << cfg->echo_gpio[i];
    }
    gpio_config_t echo_io = {
        .pin_bit_mask = echo_mask,
        .mode         = GPIO_MODE_INPUT,
        /* Czujnik steruje ECHO aktywnie w obie strony, ale na plytce nie ma
         * zadnego rezystora na tej linii. Odpiety czujnik zostawial wejscie
         * w powietrzu, a przerwanie na obu zboczach lapalo z niego szum
         * setki tysiecy razy na sekunde - bryla przestawala nadawac.
         * Sciagniecie do masy (~45k) trzyma odpiete wejscie na 0, czyli
         * "brak echa", a podpietemu czujnikowi nie przeszkadza. */
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type    = GPIO_INTR_ANYEDGE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&echo_io), TAG, "konfiguracja ECHO");

    /* Serwis przerwan moze byc juz zainstalowany przez inny sterownik.
     * Zapamietujemy, czy to my go wlaczylismy, zeby przy deinit nie zabrac
     * go komus innemu. */
    esp_err_t isr_ret = gpio_install_isr_service(0);
    if (isr_ret == ESP_OK) {
        s_n.isr_service_ours = true;
    } else if (isr_ret != ESP_ERR_INVALID_STATE) {
        ESP_RETURN_ON_ERROR(isr_ret, TAG, "gpio_install_isr_service");
    }

    esp_err_t ret = ESP_OK;
    for (int i = 0; i < cfg->count; i++) {
        s_n.echo[i].gpio = cfg->echo_gpio[i];
        s_n.echo[i].done = xSemaphoreCreateBinary();
        ESP_GOTO_ON_FALSE(s_n.echo[i].done, ESP_ERR_NO_MEM, fail, TAG, "semafor %d", i);
        ESP_GOTO_ON_ERROR(gpio_isr_handler_add(cfg->echo_gpio[i], echo_isr, &s_n.echo[i]),
                          fail, TAG, "przerwanie ECHO %d", i);
    }

    s_n.initialized = true;
    ESP_LOGI(TAG, "gotowe: TRIG GPIO%d, %d czujnik(ow), predkosc dzwieku %.1f m/s przy %.1f C",
             cfg->trig_gpio, cfg->count, flora_sonar_speed_of_sound(), cfg->temperature_c);
    for (int i = 0; i < cfg->count; i++) {
        ESP_LOGI(TAG, "  [%d] ECHO GPIO%d", i, cfg->echo_gpio[i]);
    }
    return ESP_OK;

fail:
    for (int i = 0; i < cfg->count; i++) {
        if (s_n.echo[i].done) {
            gpio_isr_handler_remove(cfg->echo_gpio[i]);
            vSemaphoreDelete(s_n.echo[i].done);
        }
    }
    if (s_n.isr_service_ours) {
        gpio_uninstall_isr_service();
    }
    memset(&s_n, 0, sizeof(s_n));
    return ret;
}

esp_err_t flora_sonar_deinit(void)
{
    ESP_RETURN_ON_FALSE(s_n.initialized, ESP_ERR_INVALID_STATE, TAG, "nie zainicjalizowano");
    for (int i = 0; i < s_n.cfg.count; i++) {
        gpio_isr_handler_remove(s_n.echo[i].gpio);
        vSemaphoreDelete(s_n.echo[i].done);
    }
    if (s_n.isr_service_ours) {
        gpio_uninstall_isr_service();
    }
    memset(&s_n, 0, sizeof(s_n));
    return ESP_OK;
}

/* Czujnik bez echa potrafi trzymac ECHO wysoko dluzej niz settle_ms
 * (zalezy od modelu). Strzal w tym czasie czujnik zignoruje, a my
 * dostalibysmy koniec starego impulsu - czekamy wiec, az linie opadna. */
#define BUSY_WAIT_MS 150

static bool echo_busy(int i)
{
    return gpio_get_level(s_n.echo[i].gpio) != 0;
}

/** Odstep od poprzedniego strzalu i czekanie na opadniecie linii ECHO. */
static void prepare_shot(void)
{
    int64_t since_ms = (esp_timer_get_time() - s_n.last_measure_us) / 1000;
    if (since_ms < (int64_t)s_n.cfg.settle_ms) {
        vTaskDelay(pdMS_TO_TICKS(s_n.cfg.settle_ms - since_ms));
    }
    for (int t = 0; t < BUSY_WAIT_MS; t += 5) {
        bool any = false;
        for (int i = 0; i < s_n.cfg.count; i++) {
            any |= echo_busy(i);
        }
        if (!any) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    for (int i = 0; i < s_n.cfg.count; i++) {
        xSemaphoreTake(s_n.echo[i].done, 0);   /* kasujemy sygnal po poprzednim pomiarze */
        s_n.echo[i].rise_us = 0;
        s_n.echo[i].fall_us = 0;
    }
}

/** Impuls wyzwalajacy: 10 us stanu wysokiego wg noty katalogowej. */
static void fire(void)
{
    gpio_set_level(s_n.cfg.trig_gpio, 0);
    esp_rom_delay_us(4);
    gpio_set_level(s_n.cfg.trig_gpio, 1);
    esp_rom_delay_us(10);
    gpio_set_level(s_n.cfg.trig_gpio, 0);
    s_n.last_measure_us = esp_timer_get_time();
}

/** Czeka na echo czujnika i do deadline_us i przelicza na mm. */
static esp_err_t collect(int index, bool was_busy, int64_t deadline_us, float *distance_mm)
{
    *distance_mm = FLORA_SONAR_INVALID;
    echo_t *e = &s_n.echo[index];
    if (was_busy) {
        /* Linia wciaz wysoko z poprzedniego strzalu - czujnik nie przyjal
         * nowego, wynik bylby stary albo zaden. */
        ESP_LOGD(TAG, "[%d] ECHO wciaz zajete przed strzalem", index);
        return ESP_ERR_INVALID_STATE;
    }
    int64_t left_us = deadline_us - esp_timer_get_time();
    TickType_t wait = left_us > 0 ? pdMS_TO_TICKS((left_us + 999) / 1000) : 0;
    if (xSemaphoreTake(e->done, wait) != pdTRUE) {
        ESP_LOGD(TAG, "[%d] brak echa w %" PRIu32 " ms", index, s_n.cfg.timeout_ms);
        return ESP_ERR_TIMEOUT;
    }

    int64_t rise = e->rise_us;
    int64_t fall = e->fall_us;
    if (!(rise > 0 && fall > rise)) {
        ESP_LOGD(TAG, "[%d] niespojne znaczniki czasu echa", index);
        return ESP_ERR_INVALID_RESPONSE;
    }

    float pulse_us = (float)(fall - rise);
    float mm = pulse_us * flora_sonar_speed_of_sound() / 2000.0f;

    if (mm < s_n.cfg.min_distance_mm || mm > s_n.cfg.max_distance_mm) {
        ESP_LOGD(TAG, "[%d] %.0f mm poza zakresem wiarygodnosci", index, mm);
        return ESP_ERR_INVALID_RESPONSE;
    }

    *distance_mm = mm;
    return ESP_OK;
}

esp_err_t flora_sonar_measure(int index, float *distance_mm)
{
    ESP_RETURN_ON_FALSE(s_n.initialized, ESP_ERR_INVALID_STATE, TAG, "nie zainicjalizowano");
    ESP_RETURN_ON_FALSE(distance_mm, ESP_ERR_INVALID_ARG, TAG, "distance_mm == NULL");
    ESP_RETURN_ON_FALSE(index >= 0 && index < s_n.cfg.count, ESP_ERR_INVALID_ARG, TAG,
                        "indeks %d poza zakresem", index);

    prepare_shot();
    bool busy = echo_busy(index);
    fire();
    int64_t deadline = s_n.last_measure_us + (int64_t)s_n.cfg.timeout_ms * 1000;
    return collect(index, busy, deadline, distance_mm);
}

esp_err_t flora_sonar_measure_all(float *distance_mm, esp_err_t *result)
{
    ESP_RETURN_ON_FALSE(s_n.initialized, ESP_ERR_INVALID_STATE, TAG, "nie zainicjalizowano");
    ESP_RETURN_ON_FALSE(distance_mm && result, ESP_ERR_INVALID_ARG, TAG, "NULL");

    prepare_shot();
    bool busy[FLORA_SONAR_MAX_SENSORS];
    for (int i = 0; i < s_n.cfg.count; i++) {
        busy[i] = echo_busy(i);
    }
    fire();
    /* Wszystkie echa biegna rownolegle, wspolny termin - calosc trwa tyle
     * co jeden pomiar, a nie suma. */
    int64_t deadline = s_n.last_measure_us + (int64_t)s_n.cfg.timeout_ms * 1000;
    for (int i = 0; i < s_n.cfg.count; i++) {
        result[i] = collect(i, busy[i], deadline, &distance_mm[i]);
    }
    return ESP_OK;
}

int flora_sonar_count(void)
{
    return s_n.initialized ? s_n.cfg.count : 0;
}

static int cmp_float(const void *a, const void *b)
{
    float fa = *(const float *)a, fb = *(const float *)b;
    return (fa > fb) - (fa < fb);
}

esp_err_t flora_sonar_measure_median(int index, int samples, float *distance_mm)
{
    ESP_RETURN_ON_FALSE(distance_mm, ESP_ERR_INVALID_ARG, TAG, "distance_mm == NULL");
    ESP_RETURN_ON_FALSE(samples > 0 && samples <= 9, ESP_ERR_INVALID_ARG, TAG,
                        "samples musi byc w zakresie 1-9");

    float buf[9];
    int got = 0;
    for (int i = 0; i < samples; i++) {
        float d;
        if (flora_sonar_measure(index, &d) == ESP_OK) {
            buf[got++] = d;
        }
    }
    ESP_RETURN_ON_FALSE(got > 0, ESP_ERR_TIMEOUT, TAG,
                        "[%d] zaden z %d pomiarow sie nie udal", index, samples);

    qsort(buf, got, sizeof(float), cmp_float);
    *distance_mm = buf[got / 2];
    return ESP_OK;
}
