/**
 * @file flora_board.h
 * @brief Mapa pinow jednostki dolnej (bryla): ESP32-S3 DevKitC-1 N16R8.
 *
 * Jedno miejsce prawdy dla pinoutu. Zmieniasz tu, nie w sterownikach.
 *
 * Piny, ktorych NIE wolno uzywac na tej plytce:
 *   0, 3, 45, 46  - strapping (stan przy boot decyduje o trybie startu)
 *   19, 20        - USB D-/D+
 *   35, 36, 37    - PSRAM (wariant N16R8)
 *   43, 44        - UART0 (konsola, idf.py monitor)
 *   38, 48        - dioda RGB
 */
#pragma once

#include "driver/gpio.h"

/* ---- Naped: Cytron MDD10A (PWM + DIR, logika 3,3V) ---- */
#define FLORA_PIN_MOTOR_L_PWM GPIO_NUM_4
#define FLORA_PIN_MOTOR_L_DIR GPIO_NUM_5
#define FLORA_PIN_MOTOR_R_PWM GPIO_NUM_6
#define FLORA_PIN_MOTOR_R_DIR GPIO_NUM_7

/* ---- Enkodery Halla JGB37-520, 11 PPR na wale silnika ---- */
#define FLORA_PIN_ENC_L_A GPIO_NUM_15
#define FLORA_PIN_ENC_L_B GPIO_NUM_16
#define FLORA_PIN_ENC_R_A GPIO_NUM_17
#define FLORA_PIN_ENC_R_B GPIO_NUM_18

/* ---- I2C (INA226) ---- */
#define FLORA_PIN_I2C_SDA GPIO_NUM_8
#define FLORA_PIN_I2C_SCL GPIO_NUM_9

/* ---- UART do glowicy masztu ---- */
#define FLORA_PIN_HEAD_TX GPIO_NUM_11
#define FLORA_PIN_HEAD_RX GPIO_NUM_12

/* ---- Ultradzwieki JSN-SR04T (wspolny TRIG, echo przez dzielnik) ---- */
#define FLORA_PIN_US_TRIG GPIO_NUM_13
#define FLORA_PIN_US_ECHO_1 GPIO_NUM_14
#define FLORA_PIN_US_ECHO_2 GPIO_NUM_21
#define FLORA_PIN_US_ECHO_3 GPIO_NUM_47

/* ---- Pozostale peryferia ---- */
#define FLORA_PIN_SOIL_ADC GPIO_NUM_1     /**< wilgotnosc gleby, ADC1 kanal 0 */
#define FLORA_PIN_WATER_LEVEL GPIO_NUM_10 /**< XKC-Y25, wyjscie podciagniete do 3,3V */
#define FLORA_PIN_DS18B20 GPIO_NUM_41     /**< 1-Wire, + 4,7k do 3,3V */
#define FLORA_PIN_MAST_SWITCH GPIO_NUM_39 /**< IRF4905 + NPN, klucz 12V masztu */
#define FLORA_PIN_PUMP GPIO_NUM_40        /**< D4184, pompka perystaltyczna */

/* ---- Parametry mechaniczne napedu ----
 * Enkoder daje 11 impulsow na obrot WALU SILNIKA. PCNT w trybie kwadraturowym
 * zlicza 4 zbocza na impuls, wiec na obrot wyjscia przekladni przypada:
 *     11 * 4 * przelozenie
 * Przy 270:1 to 11880 zliczen na obrot kola.
 */
#define FLORA_ENCODER_PPR 11      /**< impulsow na obrot walu silnika */
#define FLORA_ENCODER_QUAD_MULT 4 /**< dekodowanie x4 (oba zbocza, oba kanaly) */
#define FLORA_GEAR_RATIO 270.0f
#define FLORA_WHEEL_DIAMETER_MM 130.0f

#define FLORA_COUNTS_PER_WHEEL_REV \
    (FLORA_ENCODER_PPR * FLORA_ENCODER_QUAD_MULT * FLORA_GEAR_RATIO)

/** Droga w mm na jedno zliczenie enkodera. */
#define FLORA_MM_PER_COUNT \
    ((3.14159265f * FLORA_WHEEL_DIAMETER_MM) / FLORA_COUNTS_PER_WHEEL_REV)

/* ---- Kalibracja czujnika wilgotnosci gleby ----
 * Zmierzone 2026-10-15 na egzemplarzu zamontowanym w robocie.
 * Czujnik pojemnosciowy: sucho daje WYZSZY odczyt niz mokro.
 *
 * Uwaga przy interpretacji: punkt "mokro" zdjety w czystej wodzie oznacza
 * pelne zanurzenie, czego gleba nigdy nie osiaga. W ziemi odczyty beda
 * siegac realnie 60-80%, i to jest poprawne - to skala wzgledna, a nie
 * objetosciowa zawartosc wody.
 */
#define FLORA_SOIL_RAW_DRY 3340 /**< w powietrzu */
#define FLORA_SOIL_RAW_WET 1125 /**< zanurzony w wodzie (zmierzone 1120-1130) */

/* ---- Klucze w NVS ----
 * Nazwy sa po angielsku, jak reszta identyfikatorow, i trzymane obok
 * wartosci domyslnych - bo to wlasnie ta para musi sie zgadzac. Stale
 * zamiast literalow, zeby literowka nie przechodzila po cichu jako
 * "klucza nie ma, biore domyslna".
 *
 * Limit NVS to 15 znakow na klucz i na nazwe przestrzeni.
 */
#define FLORA_NVS_NS_CALIB "calib"
#define FLORA_NVS_SOIL_DRY "soil_dry"
#define FLORA_NVS_SOIL_WET "soil_wet"
#define FLORA_NVS_GEAR_RATIO "gear_ratio"
#define FLORA_NVS_ENC_SWAPPED "enc_swapped"
#define FLORA_NVS_MOTOR_L_INV "motor_l_inv"
#define FLORA_NVS_MOTOR_R_INV "motor_r_inv"
#define FLORA_NVS_ENC_L_INV "enc_l_inv"
#define FLORA_NVS_ENC_R_INV "enc_r_inv"
#define FLORA_NVS_GYRO_BIAS_X "gyro_bias_x"
#define FLORA_NVS_GYRO_BIAS_Y "gyro_bias_y"
#define FLORA_NVS_GYRO_BIAS_Z "gyro_bias_z"

/* ---- Siec: WiFi i MQTT ----
 * ================== DO UZUPELNIENIA ==================
 * Na razie wpisane na sztywno. Sluza jako wartosci DOMYSLNE - aplikacja
 * czyta je z NVS, wiec po pierwszym zapisaniu zmiana nie wymaga juz
 * rekompilacji. Tym samym wzorcem co kalibracja gleby.
 *
 * Uwaga: te stale trafiaja do binarki jawnym tekstem. Przed oddaniem
 * repozytorium na zewnatrz przenies je do NVS i wyczysc stad.
 * =====================================================
 */
#define FLORA_WIFI_SSID_DEFAULT "iPhone Robert"
#define FLORA_WIFI_PASS_DEFAULT "SWER1234"
#define FLORA_MQTT_URI_DEFAULT "mqtt://192.168.1.10:1883"
#define FLORA_MQTT_BASE_TOPIC "flora/robot1"

#define FLORA_NVS_NS_NET "net"
#define FLORA_NVS_WIFI_SSID "wifi_ssid"
#define FLORA_NVS_WIFI_PASS "wifi_pass"
#define FLORA_NVS_MQTT_URI "mqtt_uri"

/* ---- Zasilanie ---- */
#define FLORA_SHUNT_MILLIOHM 10.0f /**< INA226 z rezystorem R010 */
#define FLORA_BATTERY_CELLS 4      /**< LiFePO4 4S, 12,8V nominalnie */
