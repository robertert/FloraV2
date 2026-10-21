#include <string.h>

#include "flora_link_frame.h"

#ifdef ESP_PLATFORM
#include "esp_rom_crc.h"
#endif

uint16_t flora_frame_crc16(const uint8_t *data, size_t len)
{
#ifdef ESP_PLATFORM
    /* Wersja z ROM-u: zero flasha i szybsza. Sprawdzone, ze to dokladnie
     * CCITT-FALSE - wektor "123456789" daje 0x29B1. */
    return esp_rom_crc16_be(0xFFFF, data, (uint32_t)len);
#else
    /* Przenosna wersja bitowa do testow na hoscie. Ten sam wariant. */
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
    }
    return crc;
#endif
}

void flora_parser_init(flora_parser_t *p, uint8_t expect_version,
                       flora_frame_size_fn size_of)
{
    memset(p, 0, sizeof(*p));
    p->state          = FLORA_ST_START;
    p->expect_version = expect_version;
    p->size_of        = size_of;
}

/**
 * Porzuca biezaca ramke.
 *
 * Jesli bajt, ktory wywolal blad, sam jest bajtem startu, traktujemy go jako
 * poczatek nowej ramki zamiast go skasowac. Bez tego gubilismy cala nastepna
 * ramke: po pojedynczym smieciowym 0xA5 parser czeka na wersje, dostaje
 * prawdziwy 0xA5, odrzuca go jako zla wersje i konsumuje - a ramka bez bajtu
 * startu juz nigdy sie nie zsynchronizuje. Wykryl to test na hoscie.
 */
static flora_parse_result_t fail(flora_parser_t *p, flora_parse_result_t why, uint8_t byte)
{
    p->buf_len     = 0;
    p->payload_got = 0;
    p->state = (byte == FLORA_FRAME_START) ? FLORA_ST_VER : FLORA_ST_START;
    return why;
}

flora_parse_result_t flora_parser_feed(flora_parser_t *p, uint8_t byte)
{
    /* Bufor na naglowek i payload - potrzebny do przeliczenia CRC.
     * Bajt startu do niego nie wchodzi, bo nie jest objety suma. */
    if (p->state != FLORA_ST_START && p->buf_len < sizeof(p->buf)) {
        p->buf[p->buf_len++] = byte;
    }

    switch (p->state) {
        case FLORA_ST_START:
            if (byte == FLORA_FRAME_START) {
                p->buf_len     = 0;
                p->payload_got = 0;
                p->state       = FLORA_ST_VER;
            }
            return FLORA_PARSE_IDLE;

        case FLORA_ST_VER:
            /* Niezgodna wersja to prawie zawsze przeflashowana jedna plytka,
             * wiec odrzucamy od razu, zanim zinterpretujemy reszte. */
            if (byte != p->expect_version) {
                p->frame.version = byte;   /* zachowujemy do zgloszenia w logu */
                return fail(p, FLORA_PARSE_ERR_VERSION, byte);
            }
            p->frame.version = byte;
            p->state = FLORA_ST_TYPE;
            return FLORA_PARSE_IDLE;

        case FLORA_ST_TYPE:
            /* Nieznany typ odrzucamy natychmiast. Dzieki temu uszkodzona
             * ramka ginie w obrebie naglowka i nie polknie kolejnych bajtow,
             * wsrod ktorych moze byc poczatek prawdziwej ramki. */
            if (p->size_of && p->size_of(byte) < 0) {
                return fail(p, FLORA_PARSE_ERR_TYPE, byte);
            }
            p->frame.type = byte;
            p->state = FLORA_ST_FLAGS;
            return FLORA_PARSE_IDLE;

        case FLORA_ST_FLAGS:
            /* Potwierdzenie z zadaniem potwierdzenia to petla bez konca. */
            if ((byte & (FLORA_FLAG_ACK_REQ | FLORA_FLAG_IS_ACK)) ==
                (FLORA_FLAG_ACK_REQ | FLORA_FLAG_IS_ACK)) {
                return fail(p, FLORA_PARSE_ERR_FLAGS, byte);
            }
            p->frame.flags = byte;
            p->state = FLORA_ST_SEQ;
            return FLORA_PARSE_IDLE;

        case FLORA_ST_SEQ:
            p->frame.seq = byte;
            p->state = FLORA_ST_LEN;
            return FLORA_PARSE_IDLE;

        case FLORA_ST_LEN: {
            if (byte > FLORA_FRAME_MAX_PAYLOAD) {
                return fail(p, FLORA_PARSE_ERR_LENGTH, byte);
            }
            /* Kazdy payload ma staly rozmiar zalezny od typu. Sprawdzenie
             * zgodnosci wylapuje przekrecony bit, ktory dal dlugosc nadal
             * mieszczaca sie w zakresie - samo "nie wiecej niz 64" by go
             * przepuscilo. Przy okazji ramka ginie juz tutaj, w szostym
             * bajcie, zamiast po polknieciu calego falszywego payloadu. */
            if (p->size_of) {
                int expect = p->size_of(p->frame.type);
                if (expect >= 0 && byte != (uint8_t)expect) {
                    return fail(p, FLORA_PARSE_ERR_LENGTH, byte);
                }
            }
            p->frame.len = byte;
            p->state = (byte == 0) ? FLORA_ST_CRC_HI : FLORA_ST_PAYLOAD;
            return FLORA_PARSE_IDLE;
        }

        case FLORA_ST_PAYLOAD:
            p->frame.payload[p->payload_got++] = byte;
            if (p->payload_got >= p->frame.len) {
                p->state = FLORA_ST_CRC_HI;
            }
            return FLORA_PARSE_IDLE;

        case FLORA_ST_CRC_HI:
            p->crc_received = (uint16_t)byte << 8;
            p->state = FLORA_ST_CRC_LO;
            return FLORA_PARSE_IDLE;

        case FLORA_ST_CRC_LO: {
            p->crc_received |= byte;
            /* Dwa ostatnie bajty w buforze to samo CRC - nie wchodza do sumy. */
            uint16_t calc = flora_frame_crc16(p->buf, (size_t)(p->buf_len - 2));
            if (calc != p->crc_received) {
                return fail(p, FLORA_PARSE_ERR_CRC, byte);
            }
            p->state   = FLORA_ST_START;
            p->buf_len = 0;
            return FLORA_PARSE_FRAME;
        }
    }
    return fail(p, FLORA_PARSE_ERR_CRC, byte);
}

size_t flora_frame_build(uint8_t *out, uint8_t version, uint8_t type,
                         uint8_t flags, uint8_t seq,
                         const void *payload, uint8_t len)
{
    if (!out || len > FLORA_FRAME_MAX_PAYLOAD || (len > 0 && !payload)) {
        return 0;
    }

    out[0] = FLORA_FRAME_START;
    out[1] = version;
    out[2] = type;
    out[3] = flags;
    out[4] = seq;
    out[5] = len;
    if (len > 0) {
        memcpy(&out[FLORA_FRAME_HEADER_LEN], payload, len);
    }

    /* CRC obejmuje wszystko poza bajtem startu. */
    size_t covered = (size_t)(FLORA_FRAME_HEADER_LEN - 1) + len;
    uint16_t crc = flora_frame_crc16(&out[1], covered);
    out[FLORA_FRAME_HEADER_LEN + len]     = (uint8_t)(crc >> 8);
    out[FLORA_FRAME_HEADER_LEN + len + 1] = (uint8_t)(crc & 0xFF);

    return (size_t)FLORA_FRAME_HEADER_LEN + len + FLORA_FRAME_CRC_LEN;
}
