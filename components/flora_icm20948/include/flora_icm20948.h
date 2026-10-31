/**
 * @file flora_icm20948.h
 * @brief IMU 9-osiowy ICM-20948: akcelerometr, zyroskop, magnetometr.
 *
 * Zwraca dane w jednostkach fizycznych, bez fuzji - wyznaczanie kursu
 * i orientacji to zadanie warstwy wyzej.
 *
 * MAGNETOMETR: w ICM-20948 jest to osobny uklad AK09916, podlaczony do
 * wewnetrznej magistrali pomocniczej. Sterownik wlacza tryb przelotowy
 * (bypass), dzieki ktoremu AK09916 pojawia sie wprost na glownej
 * magistrali pod adresem 0x0C. To znacznie prostsze i pewniejsze niz
 * obsluga wbudowanego kontrolera I2C, ale ma jeden skutek uboczny:
 * adres 0x0C jest wtedy zajety na calej magistrali.
 *
 * MONTAZ: magnetometr reaguje na kazdy kawalek zelaza i na pole od
 * przewodow z duzym pradem. W tym projekcie maszt ma blokade obrotu
 * wlasnie po to, zeby orientacja czujnika wzgledem bryly byla stala -
 * inaczej kalibracja twardego zelaza nie ma sensu.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Zakres akcelerometru. Mniejszy zakres to lepsza rozdzielczosc. */
typedef enum {
    FLORA_ICM_ACCEL_2G = 0,
    FLORA_ICM_ACCEL_4G,
    FLORA_ICM_ACCEL_8G,
    FLORA_ICM_ACCEL_16G,
} flora_icm_accel_fs_t;

/** Zakres zyroskopu w stopniach na sekunde. */
typedef enum {
    FLORA_ICM_GYRO_250DPS = 0,
    FLORA_ICM_GYRO_500DPS,
    FLORA_ICM_GYRO_1000DPS,
    FLORA_ICM_GYRO_2000DPS,
} flora_icm_gyro_fs_t;

typedef struct {
    uint8_t  address;        /**< 0x68 albo 0x69 */
    uint8_t  mag_address;    /**< AK09916, zwykle 0x0C */
    uint32_t speed_hz;

    flora_icm_accel_fs_t accel_fs;
    flora_icm_gyro_fs_t  gyro_fs;

    /** Wlacza magnetometr. Bez niego adres 0x0C pozostaje wolny. */
    bool enable_magnetometer;
} flora_icm20948_cfg_t;

/**
 * Zakresy dobrane pod wolno jezdzacego robota: 4 g z zapasem na wstrzasy
 * od nierownosci, 500 dps znacznie powyzej predkosci obrotu w miejscu.
 * Ciasniejsze zakresy to lepsza rozdzielczosc tam, gdzie naprawde pracuje.
 */
#define FLORA_ICM20948_DEFAULT_CFG()            \
    {                                           \
        .address             = 0x68,            \
        .mag_address         = 0x0C,            \
        .speed_hz            = 400000,          \
        .accel_fs            = FLORA_ICM_ACCEL_4G,   \
        .gyro_fs             = FLORA_ICM_GYRO_500DPS,\
        .enable_magnetometer = true,            \
    }

typedef struct flora_icm20948_s *flora_icm20948_handle_t;

typedef struct {
    float accel_g[3];     /**< X, Y, Z w g */
    float gyro_dps[3];    /**< X, Y, Z w stopniach na sekunde */
    float temperature_c;  /**< z czujnika w ukladzie, nie z otoczenia */
} flora_icm20948_data_t;

typedef struct {
    float field_ut[3];    /**< X, Y, Z w mikroteslach */
    bool  overflow;       /**< pole poza zakresem - odczyt niewiarygodny */
} flora_icm20948_mag_t;

esp_err_t flora_icm20948_init(i2c_master_bus_handle_t bus,
                              const flora_icm20948_cfg_t *cfg,
                              flora_icm20948_handle_t *out);
esp_err_t flora_icm20948_deinit(flora_icm20948_handle_t h);

/** Akcelerometr, zyroskop i temperatura jednym odczytem. */
esp_err_t flora_icm20948_read(flora_icm20948_handle_t h, flora_icm20948_data_t *out);

/**
 * @brief Magnetometr.
 *
 * @return ESP_ERR_NOT_SUPPORTED gdy wylaczony w konfiguracji,
 *         ESP_ERR_NOT_FINISHED gdy nowy pomiar jeszcze nie jest gotowy.
 */
esp_err_t flora_icm20948_read_mag(flora_icm20948_handle_t h, flora_icm20948_mag_t *out);

/**
 * @brief Wyznacza offset zyroskopu przy nieruchomym czujniku.
 *
 * Zyroskopy maja dryf zera, ktory scalkowany zamienia sie w narastajacy
 * blad kursu - kilka stopni na minute to typowa wartosc. Wywolaj przy
 * calkowicie nieruchomym robocie; wynik jest potem odejmowany od odczytow.
 *
 * @param samples liczba probek do usrednienia (np. 200)
 */
esp_err_t flora_icm20948_calibrate_gyro(flora_icm20948_handle_t h, int samples);

/** Wyliczony offset zyroskopu w stopniach na sekunde. */
void flora_icm20948_get_gyro_bias(flora_icm20948_handle_t h, float bias_dps[3]);

/**
 * @brief Wpisuje offset zyroskopu z zewnatrz.
 *
 * Do wgrania wartosci zapamietanej w NVS, bez ponownej kalibracji. Glowica
 * masztu traci zasilanie przy kazdym uspieniu, a kalibracja wymaga 1-2 s
 * bezruchu, czego nie da sie zagwarantowac - wiec offset liczy sie raz
 * i odtwarza przy starcie.
 */
void flora_icm20948_set_gyro_bias(flora_icm20948_handle_t h, const float bias_dps[3]);

bool    flora_icm20948_has_magnetometer(flora_icm20948_handle_t h);
uint8_t flora_icm20948_who_am_i(flora_icm20948_handle_t h);

#ifdef __cplusplus
}
#endif
