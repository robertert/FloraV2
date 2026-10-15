/**
 * @file bringup_drive.c
 * @brief Test rozruchowy napedu: silniki, enkodery, diagnostyka sygnalu.
 *
 * Wydzielone z main.c po zakonczeniu rozruchu napedu (2026-10-15). Kod
 * zostaje, bo diagnostyka zbocz jest narzedziem wielokrotnego uzytku -
 * to ona wykryla urwany kanal A prawego enkodera na GPIO17.
 *
 * Uruchomienie: bringup_drive_run(ina) z app_main().
 * UWAGA: kola musza byc w powietrzu.
 */
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_timer.h"

#include "bringup_drive.h"
#include "flora_board.h"
#include "flora_motors.h"
#include "flora_encoders.h"
#include "flora_ina226.h"

static const char *TAG = "naped";

static flora_ina226_handle_t s_ina;   /* przekazywany do bringup_drive_run */

/* Moc uzywana w testach. 30% wystarczy, zeby kolo ruszylo bez obciazenia. */
#define TEST_POWER        30.0f
#define TEST_DURATION_MS  1500
#define SETTLE_MS         600
#define MIN_COUNTS_MOVED  50

static int s_checks_passed;
static int s_checks_failed;

static void check(bool ok, const char *what, const char *hint)
{
    if (ok) {
        s_checks_passed++;
        ESP_LOGI(TAG, "  [ OK ] %s", what);
    } else {
        s_checks_failed++;
        ESP_LOGE(TAG, "  [BLAD] %s", what);
        if (hint) {
            ESP_LOGE(TAG, "         podpowiedz: %s", hint);
        }
    }
}

static float log_power(const char *moment)
{
    if (!s_ina) {
        return NAN;   /* brak ukladu: milczymy, zamiast powtarzac to samo ostrzezenie */
    }
    flora_ina226_reading_t r;
    if (flora_ina226_read(s_ina, &r) != ESP_OK) {
        ESP_LOGW(TAG, "  %s: odczyt INA226 nie powiodl sie", moment);
        return NAN;
    }
    ESP_LOGI(TAG, "  %s: %.2f V, %.3f A, %.2f W (bocznik %.3f mV)",
             moment, r.bus_voltage_v, r.current_a, r.power_w, r.shunt_voltage_mv);
    return r.current_a;
}

/**
 * Prog, powyzej ktorego uznajemy ze sterownik faktycznie ciagnie prad.
 * Sam MDD10A na logike bierze wiecej niz 20 mA, wiec ponizej tej wartosci
 * mozna powiedziec z cala pewnoscia, ze przez bocznik nic nie plynie.
 */
#define CURRENT_ALIVE_A  0.020f

/**
 * Jeden przebieg: zadaje moc, mierzy przyrost zliczen i prad.
 *
 * @param expect_left   spodziewany kierunek lewego kola: +1, -1 albo 0 (ma stac)
 */
static void run_leg(const char *name, float left_power, float right_power,
                    int expect_left, int expect_right)
{
    ESP_LOGI(TAG, "--- %s (L=%.0f%%, P=%.0f%%) ---", name, left_power, right_power);

    flora_encoders_sample_t before, after;
    ESP_ERROR_CHECK(flora_encoders_read(&before));

    float idle_a = log_power("przed ruchem");

    ESP_ERROR_CHECK(flora_motors_set(left_power, right_power));
    vTaskDelay(pdMS_TO_TICKS(TEST_DURATION_MS / 2));
    float load_a = log_power("pod obciazeniem");
    vTaskDelay(pdMS_TO_TICKS(TEST_DURATION_MS / 2));

    /* Czy przez bocznik w ogole cokolwiek poplynelo. */
    bool current_flowed = !isnan(idle_a) && !isnan(load_a)
                          && fabsf(load_a - idle_a) > CURRENT_ALIVE_A;
    bool current_known = !isnan(idle_a) && !isnan(load_a);

    ESP_ERROR_CHECK(flora_encoders_read(&after));
    ESP_ERROR_CHECK(flora_motors_stop());

    int64_t dl = after.left_counts  - before.left_counts;
    int64_t dr = after.right_counts - before.right_counts;
    float speed_l = 0.0f, speed_r = 0.0f;
    flora_encoders_speed_mm_s(&before, &after, &speed_l, &speed_r);

    ESP_LOGI(TAG, "  lewe:  %+8" PRId64 " zliczen (%+7.1f mm, %+6.1f mm/s)",
             dl, after.left_mm - before.left_mm, speed_l);
    ESP_LOGI(TAG, "  prawe: %+8" PRId64 " zliczen (%+7.1f mm, %+6.1f mm/s)",
             dr, after.right_mm - before.right_mm, speed_r);

    struct { int64_t delta; int expect; const char *wheel; } wheels[] = {
        { dl, expect_left,  "lewe"  },
        { dr, expect_right, "prawe" },
    };

    for (int i = 0; i < 2; i++) {
        char what[96];
        if (wheels[i].expect == 0) {
            snprintf(what, sizeof(what), "%s kolo stoi", wheels[i].wheel);
            check(llabs(wheels[i].delta) < MIN_COUNTS_MOVED, what,
                  "kolo kreci sie mimo zerowej mocy - zamienione kanaly PWM albo DIR");
            continue;
        }

        snprintf(what, sizeof(what), "%s kolo sie krecilo", wheels[i].wheel);
        bool moved = llabs(wheels[i].delta) >= MIN_COUNTS_MOVED;

        /* Prad rozdziela dwie zupelnie rozne awarie: albo silnik sie nie
         * kreci (problem zasilania lub sygnalu), albo kreci sie, ale enkoder
         * nie liczy. Bez tego rozroznienia podpowiedz byla zgadywaniem. */
        const char *hint;
        if (current_known && !current_flowed) {
            /* Uwaga: to NIE dowodzi, ze silnik stoi. Jesli enkoder liczy
             * zbocza, silnik sie kreci, a zerowy prad oznacza tylko tyle,
             * ze bocznik INA226 lezy poza torem pradu silnikow. */
            hint = "przez bocznik nie poplynal prad. Jesli silniki sie kreca, "
                   "to bocznik INA226 nie jest w torze pradu - musi byc wpiety "
                   "SZEREGOWO (IN+ od baterii, IN- do odbiornikow), a MDD10A "
                   "nie moze brac 12V z punktu przed modulem. "
                   "Jesli silniki stoja - sprawdz wspolna mase i zasilanie MDD10A";
        } else if (current_known) {
            hint = "prad plynie, wiec silnik dostaje moc, ale enkoder nie liczy: "
                   "sprawdz zasilanie enkodera z 3,3V (nie z 5V), piny A i B "
                   "oraz czy masa enkodera jest wspolna z ESP32";
        } else {
            hint = "brak zliczen: sprawdz zasilanie enkodera z 3,3V, piny A/B, "
                   "zasilanie silnika i czy MDD10A ma 12V";
        }
        check(moved, what, hint);

        if (moved) {
            snprintf(what, sizeof(what), "%s kolo w dobrym kierunku", wheels[i].wheel);
            bool right_way = (wheels[i].delta > 0) == (wheels[i].expect > 0);
            check(right_way, what,
                  "kreci sie odwrotnie: ustaw invert=true dla tego kola "
                  "(w flora_motors gdy wirnik jedzie zle, w flora_encoders gdy "
                  "jedzie dobrze a zliczenia maleja)");
        }
    }

    vTaskDelay(pdMS_TO_TICKS(SETTLE_MS));
}


/* ================= DIAGNOSTYKA ================= */

/** Odczytuje surowe poziomy na czterech pinach enkoderow. */
static void diag_pin_levels(const char *when)
{
    ESP_LOGI(TAG, "  %-18s L_A=%d L_B=%d | P_A=%d P_B=%d", when,
             gpio_get_level(FLORA_PIN_ENC_L_A), gpio_get_level(FLORA_PIN_ENC_L_B),
             gpio_get_level(FLORA_PIN_ENC_R_A), gpio_get_level(FLORA_PIN_ENC_R_B));
}

/**
 * Liczy zbocza na pinach enkoderow programowo, niezaleznie od PCNT.
 *
 * To jest sedno diagnozy: rozdziela "na pinie nic sie nie dzieje" (problem
 * sprzetowy: brak enkodera, brak zasilania, zly pin) od "sygnal jest, ale
 * PCNT go nie liczy" (problem konfiguracji sterownika).
 */
static void diag_edge_count(const char *name, float left_power, float right_power)
{
    const gpio_num_t pins[4] = {
        FLORA_PIN_ENC_L_A, FLORA_PIN_ENC_L_B,
        FLORA_PIN_ENC_R_A, FLORA_PIN_ENC_R_B,
    };
    const char *labels[4] = { "L_A", "L_B", "P_A", "P_B" };

    int  last[4];
    int  edges[4] = { 0 };
    for (int i = 0; i < 4; i++) {
        last[i] = gpio_get_level(pins[i]);
    }

    flora_encoders_sample_t before, after;
    flora_encoders_read(&before);
    flora_motors_set(left_power, right_power);

    /* Petla ciasna, ale krotka - nie zaglodzi watchdoga (limit 5 s). */
    int64_t deadline = esp_timer_get_time() + 500000;  /* 500 ms */
    uint32_t samples = 0;
    while (esp_timer_get_time() < deadline) {
        for (int i = 0; i < 4; i++) {
            int now = gpio_get_level(pins[i]);
            if (now != last[i]) {
                edges[i]++;
                last[i] = now;
            }
        }
        samples++;
    }

    flora_motors_stop();
    flora_encoders_read(&after);

    ESP_LOGI(TAG, "  %s (%.0f probek/pin):", name, (float)samples);
    for (int i = 0; i < 4; i++) {
        ESP_LOGI(TAG, "    %s: %6d zbocz programowo", labels[i], edges[i]);
    }
    ESP_LOGI(TAG, "    PCNT: lewy %+" PRId64 ", prawy %+" PRId64,
             after.left_counts - before.left_counts,
             after.right_counts - before.right_counts);

    /* Kanal martwy przy zywym blizniaku to odlaczony przewod, nie zepsuty
     * enkoder. Bez obu kanalow PCNT nie policzy nic sensownego, bo zliczenia
     * w gore i w dol kasuja sie nawzajem. */
    for (int pair = 0; pair < 2; pair++) {
        int a = edges[pair * 2], b = edges[pair * 2 + 1];
        const char *side = pair == 0 ? "lewym" : "prawym";
        if (a == 0 && b > 0) {
            ESP_LOGE(TAG, "    -> w %s enkoderze kanal A (GPIO%d) milczy, a B pracuje: "
                          "odlaczony przewod A", side,
                     pair == 0 ? FLORA_PIN_ENC_L_A : FLORA_PIN_ENC_R_A);
        } else if (b == 0 && a > 0) {
            ESP_LOGE(TAG, "    -> w %s enkoderze kanal B (GPIO%d) milczy, a A pracuje: "
                          "odlaczony przewod B", side,
                     pair == 0 ? FLORA_PIN_ENC_L_B : FLORA_PIN_ENC_R_B);
        }
    }

    vTaskDelay(pdMS_TO_TICKS(400));
}

/** Pelna diagnostyka toru enkoderow. */
static void diag_encoders(void)
{
    ESP_LOGI(TAG, "=== DIAGNOSTYKA ENKODEROW ===");
    ESP_LOGI(TAG, "piny: L_A=%d L_B=%d P_A=%d P_B=%d (podciagniecia wewnetrzne wlaczone)",
             FLORA_PIN_ENC_L_A, FLORA_PIN_ENC_L_B,
             FLORA_PIN_ENC_R_A, FLORA_PIN_ENC_R_B);

    /* Stan spoczynkowy. Przy wlaczonych podciagnieciach:
     *   same jedynki        -> pin wisi w powietrzu ALBO enkoder stoi na stanie wysokim
     *   same zera           -> zwarcie do masy albo enkoder trzyma stan niski
     *   mieszanka           -> cos jest podlaczone i ma rozne stany
     * Samo to nie rozstrzyga, ale razem z testem dynamicznym juz tak. */
    ESP_LOGI(TAG, "  poziomy w spoczynku (3 odczyty):");
    for (int i = 0; i < 3; i++) {
        diag_pin_levels("spoczynek");
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    diag_edge_count("lewy silnik 40%",  40.0f,  0.0f);
    diag_edge_count("prawy silnik 40%",  0.0f, 40.0f);
    diag_edge_count("oba silniki 60%",  60.0f, 60.0f);

    ESP_LOGI(TAG, "  interpretacja:");
    ESP_LOGI(TAG, "    0 zbocz programowo + silnik sie kreci -> enkoder nie dociera do pinu");
    ESP_LOGI(TAG, "      (silnik bez enkodera, brak zasilania 3,3V enkodera,");
    ESP_LOGI(TAG, "       brak wspolnej masy, albo przewody A/B nie na tych pinach)");
    ESP_LOGI(TAG, "    zbocza sa, ale PCNT = 0 -> blad konfiguracji licznika");
    ESP_LOGI(TAG, "=============================");
}


/** Inicjalizuje naped i enkodery. Wolane z bringup_drive_run. */
static esp_err_t drive_hw_init(void)
{
    flora_motors_cfg_t motors_cfg = FLORA_MOTORS_DEFAULT_CFG();
    motors_cfg.left  = (flora_motor_cfg_t){ .pwm_gpio = FLORA_PIN_MOTOR_L_PWM,
                                            .dir_gpio = FLORA_PIN_MOTOR_L_DIR,
                                            .invert   = false };
    motors_cfg.right = (flora_motor_cfg_t){ .pwm_gpio = FLORA_PIN_MOTOR_R_PWM,
                                            .dir_gpio = FLORA_PIN_MOTOR_R_DIR,
                                            .invert   = true };
    ESP_RETURN_ON_ERROR(flora_motors_init(&motors_cfg), TAG, "naped");

    /* TODO (kalibracja kierunku, nierozstrzygniete 2026-10-14):
     * Oba enkodery licza UJEMNIE przy komendzie "w przod". Surowe znaki sa
     * przeciwne, czyli wzajemnie silniki kreca sie dobrze. Po zamontowaniu
     * kol w podwoziu sprawdzic przebieg "jazda prosto":
     *   jedzie w przod -> odwrocic invert w enc_cfg
     *   jedzie w tyl   -> odwrocic invert w motors_cfg */
    flora_encoders_cfg_t enc_cfg = FLORA_ENCODERS_DEFAULT_CFG();
    enc_cfg.left  = (flora_encoder_cfg_t){ .a_gpio = FLORA_PIN_ENC_L_A,
                                           .b_gpio = FLORA_PIN_ENC_L_B,
                                           .invert = false };
    enc_cfg.right = (flora_encoder_cfg_t){ .a_gpio = FLORA_PIN_ENC_R_A,
                                           .b_gpio = FLORA_PIN_ENC_R_B,
                                           .invert = true };
    enc_cfg.mm_per_count = FLORA_MM_PER_COUNT;
    ESP_RETURN_ON_ERROR(flora_encoders_init(&enc_cfg), TAG, "enkodery");
    return ESP_OK;
}

void bringup_drive_run(flora_ina226_handle_t ina)
{
    s_ina = ina;
    if (drive_hw_init() != ESP_OK) {
        ESP_LOGE(TAG, "nie udalo sie zainicjalizowac napedu, pomijam test");
        return;
    }
    s_checks_passed = 0;
    s_checks_failed = 0;

    ESP_LOGW(TAG, "========================================================");
    ESP_LOGW(TAG, " TEST NAPEDU - kola musza byc w powietrzu!");
    ESP_LOGW(TAG, " Start za 5 s");
    ESP_LOGW(TAG, "========================================================");
    for (int i = 5; i > 0; i--) {
        ESP_LOGW(TAG, " %d...", i);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_ERROR_CHECK(flora_encoders_reset());
    diag_encoders();
    ESP_ERROR_CHECK(flora_encoders_reset());

    run_leg("lewe kolo w przod",  TEST_POWER, 0.0f, +1, 0);
    run_leg("lewe kolo w tyl",   -TEST_POWER, 0.0f, -1, 0);
    run_leg("prawe kolo w przod", 0.0f,  TEST_POWER, 0, +1);
    run_leg("prawe kolo w tyl",   0.0f, -TEST_POWER, 0, -1);
    run_leg("jazda prosto",  TEST_POWER,  TEST_POWER, +1, +1);
    run_leg("jazda w tyl",  -TEST_POWER, -TEST_POWER, -1, -1);
    run_leg("obrot w miejscu", TEST_POWER, -TEST_POWER, +1, -1);

    ESP_LOGI(TAG, "========================================================");
    if (s_checks_failed == 0) {
        ESP_LOGI(TAG, " WYNIK: wszystko przeszlo (%d sprawdzen)", s_checks_passed);
    } else {
        ESP_LOGE(TAG, " WYNIK: %d przeszlo, %d nie przeszlo", s_checks_passed, s_checks_failed);
    }
    ESP_LOGI(TAG, "========================================================");
}
