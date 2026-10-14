/**
 * @file flora_msgs.h
 * @brief Kontrakt wiadomosci miedzy node a hubem.
 *
 * TEN PLIK MUSI BYC IDENTYCZNY PO OBU STRONACH KABLA. Dlatego zyje w monorepo
 * i jest dolaczany przez oba projekty. Nie jest czescia generycznego
 * flora_link - ten zna tylko typ (uint8_t) i surowy bufor.
 *
 * Przy KAZDEJ zmianie ukladu struktury podbij FLORA_MSGS_VERSION i przekaz go
 * jako `proto_version` w konfiguracji flora_link. Inaczej starsza plytka
 * przyjmie nowa ramke jako poprawna i zinterpretuje smieci.
 */
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Wersja kontraktu. Podbij przy zmianie ukladu ktorejkolwiek struktury. */
#define FLORA_MSGS_VERSION 1

/** Typy wiadomosci. Wartosci jawne, bo ida po kablu. */
typedef enum {
    FLORA_MSG_PING     = 0x01,  /**< brak payloadu, oczekuje FLORA_MSG_PONG */
    FLORA_MSG_PONG     = 0x02,  /**< flora_pong_t */
    FLORA_MSG_SENSORS  = 0x10,  /**< flora_sensors_t, node -> hub */
    FLORA_MSG_CMD      = 0x20,  /**< flora_cmd_t, hub -> node */
} flora_msg_type_t;

/** Odpowiedz na ping. Pozwala sprawdzic, ze druga strona zyje i od kiedy. */
typedef struct __attribute__((packed)) {
    uint32_t uptime_ms;
} flora_pong_t;

/**
 * Odczyt z czujnikow. Wartosci staloprzecinkowe, bo float po kablu to
 * proszenie sie o klopoty z wyrownaniem i rozmiarem.
 */
typedef struct __attribute__((packed)) {
    uint32_t uptime_ms;     /**< czas od startu node'a */
    uint16_t moisture_raw;  /**< surowy odczyt ADC wilgotnosci */
    int16_t  temp_c_x10;    /**< temperatura w dziesietnych stopnia, 235 = 23.5 C */
    uint16_t lux;           /**< natezenie swiatla */
    uint16_t vbat_mv;       /**< napiecie zasilania w mV */
} flora_sensors_t;

/** Komenda z huba do node'a. */
typedef struct __attribute__((packed)) {
    uint8_t  code;   /**< flora_cmd_code_t */
    uint8_t  _pad;   /**< jawne wyrownanie, zeby uklad byl oczywisty */
    uint32_t arg;
} flora_cmd_t;

typedef enum {
    FLORA_CMD_NOP           = 0x00,
    FLORA_CMD_REBOOT        = 0x01,
    FLORA_CMD_SET_INTERVAL  = 0x02,  /**< arg = ms miedzy pomiarami */
} flora_cmd_code_t;

/*
 * Straznicy rozmiaru. Jesli ktos wstawi pole w srodku struktury albo zmieni
 * typ, kompilacja padnie tutaj, a nie po cichu na kablu.
 */
_Static_assert(sizeof(flora_pong_t)    == 4,  "flora_pong_t zmienil rozmiar, podbij FLORA_MSGS_VERSION");
_Static_assert(sizeof(flora_sensors_t) == 12, "flora_sensors_t zmienil rozmiar, podbij FLORA_MSGS_VERSION");
_Static_assert(sizeof(flora_cmd_t)     == 6,  "flora_cmd_t zmienil rozmiar, podbij FLORA_MSGS_VERSION");

#ifdef __cplusplus
}
#endif
