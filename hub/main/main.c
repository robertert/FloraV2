/**
 * Doniczka-robot, jednostka dolna (bryla).
 *
 * Test rozruchowy peryferiow napedu: silniki MDD10A, enkodery na PCNT,
 * pomiar pradu INA226. Zadaniem tego kodu jest powiedziec, czy wszystko jest
 * podlaczone poprawnie, ZANIM zacznie sie pisac PID i odometrie.
 *
 * UWAGA: robot musi stac na podstawkach, kola w powietrzu.
 */
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"
#include "driver/i2c_master.h"

#include "flora_board.h"
#include "flora_motors.h"
#include "flora_encoders.h"
#include "flora_ina226.h"

static const char *TAG = "dol";

/* Moc uzywana w testach. 30% wystarczy, zeby kolo ruszylo bez obciazenia,
 * a jest na tyle malo, ze pomylka w okablowaniu nic nie urwie. */
#define TEST_POWER        30.0f
#define TEST_DURATION_MS  1500
#define SETTLE_MS         600

/* Ponizej tylu zliczen uznajemy, ze kolo w ogole nie drgnelo. */
#define MIN_COUNTS_MOVED  50

static flora_ina226_handle_t s_ina;
static i2c_master_bus_handle_t s_i2c_bus;

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

static esp_err_t i2c_bus_setup(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port          = I2C_NUM_0,
        .sda_io_num        = FLORA_PIN_I2C_SDA,
        .scl_io_num        = FLORA_PIN_I2C_SCL,
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    return i2c_new_master_bus(&bus_cfg, &s_i2c_bus);
}

/**
 * Wypisuje, co odpowiada na magistrali, i zbiera adresy do tablicy.
 * @return liczba znalezionych urzadzen
 */
static int i2c_scan(uint8_t *found, int cap)
{
    ESP_LOGI(TAG, "skanowanie I2C (SDA=%d, SCL=%d):", FLORA_PIN_I2C_SDA, FLORA_PIN_I2C_SCL);
    int n = 0;
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        if (i2c_master_probe(s_i2c_bus, addr, 50) == ESP_OK) {
            ESP_LOGI(TAG, "  znaleziono 0x%02X", addr);
            if (n < cap) {
                found[n] = addr;
            }
            n++;
        }
    }
    if (n == 0) {
        ESP_LOGW(TAG, "  cisza na magistrali - sprawdz zasilanie ukladu, podciagniecia i kable");
    }
    return n;
}

/**
 * Szuka INA226 wsrod znalezionych adresow.
 *
 * Adres ukladu zalezy od zworek A0/A1 i waha sie w zakresie 0x40-0x4F,
 * wiec zamiast zakladac domyslne 0x40 sprawdzamy identyfikator producenta
 * pod kazdym adresem, ktory w ogole odpowiada.
 */
static esp_err_t ina226_autodetect(const uint8_t *addrs, int n)
{
    flora_ina226_cfg_t cfg = FLORA_INA226_DEFAULT_CFG();
    cfg.shunt_milliohm = FLORA_SHUNT_MILLIOHM;

    for (int i = 0; i < n; i++) {
        if (addrs[i] < 0x40 || addrs[i] > 0x4F) {
            continue;  /* poza zakresem adresow INA226 */
        }
        cfg.i2c_address = addrs[i];
        if (flora_ina226_init(s_i2c_bus, &cfg, &s_ina) == ESP_OK) {
            return ESP_OK;
        }
    }
    s_ina = NULL;
    return ESP_ERR_NOT_FOUND;
}

/** Loguje stan zasilania i zwraca zmierzony prad (NAN gdy brak ukladu). */
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
            hint = "przez bocznik nie poplynal prad - sterownik nie pobiera mocy. "
                   "Sprawdz w tej kolejnosci: WSPOLNA MASA miedzy ESP32 a MDD10A "
                   "(przy zasilaniu ESP z USB to najczestsza przyczyna), "
                   "zasilanie 12V samego MDD10A za bocznikiem INA226, "
                   "silniki w zaciskach M1A/M1B i M2A/M2B";
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

static void bringup_task(void *arg)
{
    ESP_LOGW(TAG, "========================================================");
    ESP_LOGW(TAG, " TEST NAPEDU - kola musza byc w powietrzu!");
    ESP_LOGW(TAG, " Start za 5 s, odlacz zasilanie jesli robot stoi na ziemi");
    ESP_LOGW(TAG, "========================================================");
    for (int i = 5; i > 0; i--) {
        ESP_LOGW(TAG, " %d...", i);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_LOGI(TAG, "=== stan spoczynkowy ===");
    log_power("na postoju");
    ESP_ERROR_CHECK(flora_encoders_reset());

    /* Kazde kolo osobno: wylapuje zamienione kanaly i zle kierunki.
     * Drugie kolo ma w tym czasie stac - jesli rusza, cos jest przepiete. */
    run_leg("lewe kolo w przod",  TEST_POWER, 0.0f, +1, 0);
    run_leg("lewe kolo w tyl",   -TEST_POWER, 0.0f, -1, 0);
    run_leg("prawe kolo w przod", 0.0f,  TEST_POWER, 0, +1);
    run_leg("prawe kolo w tyl",   0.0f, -TEST_POWER, 0, -1);

    /* Razem: tak jak robot naprawde jezdzi. */
    run_leg("jazda prosto",  TEST_POWER,  TEST_POWER, +1, +1);
    run_leg("jazda w tyl",  -TEST_POWER, -TEST_POWER, -1, -1);
    run_leg("obrot w miejscu", TEST_POWER, -TEST_POWER, +1, -1);

    ESP_LOGI(TAG, "=== po tescie ===");
    log_power("na postoju");

    ESP_LOGI(TAG, "========================================================");
    if (s_checks_failed == 0) {
        ESP_LOGI(TAG, " WYNIK: wszystko przeszlo (%d sprawdzen)", s_checks_passed);
    } else {
        ESP_LOGE(TAG, " WYNIK: %d przeszlo, %d nie przeszlo", s_checks_passed, s_checks_failed);
    }
    ESP_LOGI(TAG, "========================================================");

    /* Telemetria w kolko, zeby dalo sie recznie pokrecic kolami i patrzec
     * na zliczenia - najszybszy sposob na sprawdzenie kierunkow bez silnikow. */
    ESP_LOGI(TAG, "przechodze w tryb podgladu: zakrec kolem recznie i patrz na zliczenia");
    flora_encoders_sample_t prev;
    ESP_ERROR_CHECK(flora_encoders_read(&prev));
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        flora_encoders_sample_t now;
        if (flora_encoders_read(&now) != ESP_OK) {
            continue;
        }
        float sl = 0.0f, sr = 0.0f;
        flora_encoders_speed_mm_s(&prev, &now, &sl, &sr);

        flora_ina226_reading_t r = { 0 };
        bool have_power = s_ina && flora_ina226_read(s_ina, &r) == ESP_OK;

        if (have_power) {
            ESP_LOGI(TAG, "L %+8" PRId64 " (%+6.1f mm/s)  P %+8" PRId64 " (%+6.1f mm/s)  %.2f V  %.3f A",
                     now.left_counts, sl, now.right_counts, sr, r.bus_voltage_v, r.current_a);
        } else {
            ESP_LOGI(TAG, "L %+8" PRId64 " (%+6.1f mm/s)  P %+8" PRId64 " (%+6.1f mm/s)  (brak INA226)",
                     now.left_counts, sl, now.right_counts, sr);
        }
        prev = now;
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "doniczka-robot, jednostka dolna, rozruch peryferiow");

    ESP_ERROR_CHECK(i2c_bus_setup());

    uint8_t i2c_addrs[16];
    int i2c_found = i2c_scan(i2c_addrs, sizeof(i2c_addrs));

    if (ina226_autodetect(i2c_addrs, i2c_found) != ESP_OK) {
        /* Brak INA226 nie blokuje testu napedu - tracimy tylko pomiar pradu. */
        ESP_LOGW(TAG, "nie znalazlem INA226 na magistrali, test pojdzie bez pomiaru pradu");
    }

    flora_motors_cfg_t motors_cfg = FLORA_MOTORS_DEFAULT_CFG();
    motors_cfg.left  = (flora_motor_cfg_t){ .pwm_gpio = FLORA_PIN_MOTOR_L_PWM,
                                            .dir_gpio = FLORA_PIN_MOTOR_L_DIR,
                                            .invert   = false };
    /* Silniki montowane lustrzanie, wiec prawy jedzie odwrotnie.
     * Jesli test pokaze zly kierunek, przestaw te flage. */
    motors_cfg.right = (flora_motor_cfg_t){ .pwm_gpio = FLORA_PIN_MOTOR_R_PWM,
                                            .dir_gpio = FLORA_PIN_MOTOR_R_DIR,
                                            .invert   = true };
    ESP_ERROR_CHECK(flora_motors_init(&motors_cfg));

    flora_encoders_cfg_t enc_cfg = FLORA_ENCODERS_DEFAULT_CFG();
    enc_cfg.left  = (flora_encoder_cfg_t){ .a_gpio = FLORA_PIN_ENC_L_A,
                                           .b_gpio = FLORA_PIN_ENC_L_B,
                                           .invert = false };
    enc_cfg.right = (flora_encoder_cfg_t){ .a_gpio = FLORA_PIN_ENC_R_A,
                                           .b_gpio = FLORA_PIN_ENC_R_B,
                                           .invert = true };
    enc_cfg.mm_per_count = FLORA_MM_PER_COUNT;
    ESP_ERROR_CHECK(flora_encoders_init(&enc_cfg));

    ESP_LOGI(TAG, "przelozenie %.0f:1, %.4f mm na zliczenie, %.0f zliczen na obrot kola",
             FLORA_GEAR_RATIO, FLORA_MM_PER_COUNT, FLORA_COUNTS_PER_WHEEL_REV);

    xTaskCreate(bringup_task, "bringup", 4096, NULL, 5, NULL);
}
