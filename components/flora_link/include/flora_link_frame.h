/**
 * @file flora_link_frame.h
 * @brief Ramkowanie i parser protokolu - warstwa bez zaleznosci od sprzetu.
 *
 * Wydzielone z flora_link celowo: ten plik kompiluje sie zwyklym gcc na
 * hoscie, wiec maszyne stanow i CRC da sie przetestowac bez plytki, bez
 * UART-u i bez ESP-IDF. Parser jest czysta funkcja nad bajtami.
 *
 * Format ramki:
 *
 *   +------+------+------+-------+------+------+-----------+--------+
 *   | 0xA5 | ver  | type | flags | seq  | len  | payload   | crc16  |
 *   |  1B  |  1B  |  1B  |  1B   |  1B  |  1B  | 0..64 B   |  2B BE |
 *   +------+------+------+-------+------+------+-----------+--------+
 *            \_____________ CRC liczony od tad _________/
 *
 * CRC-16/CCITT-FALSE (wielomian 0x1021, wartosc poczatkowa 0xFFFF, bez
 * odbicia). Wektor kontrolny: "123456789" daje 0x29B1.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FLORA_FRAME_START        0xA5
#define FLORA_FRAME_HEADER_LEN   6
#define FLORA_FRAME_CRC_LEN      2
#define FLORA_FRAME_MAX_PAYLOAD  64
#define FLORA_FRAME_MAX_LEN      (FLORA_FRAME_HEADER_LEN + FLORA_FRAME_MAX_PAYLOAD + FLORA_FRAME_CRC_LEN)

/** Bity pola flags. */
#define FLORA_FLAG_ACK_REQ  (1u << 0)  /**< nadawca oczekuje potwierdzenia */
#define FLORA_FLAG_IS_ACK   (1u << 1)  /**< to jest potwierdzenie */

/** Odebrana, zweryfikowana ramka. */
typedef struct {
    uint8_t version;
    uint8_t type;
    uint8_t flags;
    uint8_t seq;
    uint8_t len;
    uint8_t payload[FLORA_FRAME_MAX_PAYLOAD];
} flora_frame_t;

/** Wynik podania bajtu do parsera. */
typedef enum {
    FLORA_PARSE_IDLE,            /**< ramka w budowie, nic do zgloszenia */
    FLORA_PARSE_FRAME,           /**< gotowa, poprawna ramka */
    FLORA_PARSE_ERR_VERSION,     /**< niezgodna wersja protokolu */
    FLORA_PARSE_ERR_TYPE,        /**< nieznany typ wiadomosci */
    FLORA_PARSE_ERR_LENGTH,      /**< dlugosc nie pasuje do typu */
    FLORA_PARSE_ERR_FLAGS,       /**< niedozwolona kombinacja flag */
    FLORA_PARSE_ERR_CRC,         /**< suma kontrolna sie nie zgadza */
} flora_parse_result_t;

/**
 * Zwraca oczekiwana dlugosc payloadu dla typu, albo -1 dla nieznanego.
 * Parser dostaje ja przez wskaznik, zeby nie zalezec od flora_msgs.
 */
typedef int (*flora_frame_size_fn)(uint8_t type);

typedef enum {
    FLORA_ST_START, FLORA_ST_VER, FLORA_ST_TYPE, FLORA_ST_FLAGS,
    FLORA_ST_SEQ, FLORA_ST_LEN, FLORA_ST_PAYLOAD, FLORA_ST_CRC_HI, FLORA_ST_CRC_LO,
} flora_parse_state_t;

typedef struct {
    flora_parse_state_t state;
    uint8_t  expect_version;
    flora_frame_size_fn size_of;   /**< moze byc NULL - wtedy tylko gorna granica */

    flora_frame_t frame;
    uint16_t payload_got;
    uint16_t crc_received;

    /* Bufor na naglowek i payload, do przeliczenia CRC. */
    uint8_t  buf[FLORA_FRAME_MAX_LEN];
    uint16_t buf_len;
} flora_parser_t;

/** CRC-16/CCITT-FALSE. Na celu uzywa wersji z ROM-u, na hoscie przenosnej. */
uint16_t flora_frame_crc16(const uint8_t *data, size_t len);

void flora_parser_init(flora_parser_t *p, uint8_t expect_version,
                       flora_frame_size_fn size_of);

/**
 * @brief Podaje jeden bajt do parsera.
 *
 * Przy kazdym bledzie parser wraca do szukania bajtu startu. Bufor
 * odrzuconej ramki jest przeszukiwany ponownie od nastepnego bajtu, zeby
 * nie zgubic prawdziwego 0xA5, ktory wpadl w srodek uszkodzonych danych -
 * bez tego seria zaklocen od silnikow potrafi polknac kolejna dobra ramke.
 */
flora_parse_result_t flora_parser_feed(flora_parser_t *p, uint8_t byte);

/**
 * @brief Sklada ramke do bufora.
 *
 * @param out   bufor o pojemnosci co najmniej FLORA_FRAME_MAX_LEN
 * @return dlugosc zlozonej ramki, albo 0 przy zlych argumentach
 */
size_t flora_frame_build(uint8_t *out, uint8_t version, uint8_t type,
                         uint8_t flags, uint8_t seq,
                         const void *payload, uint8_t len);

#ifdef __cplusplus
}
#endif
