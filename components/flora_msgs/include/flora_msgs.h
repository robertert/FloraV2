/**
 * @file flora_msgs.h
 * @brief Kontrakt wiadomosci miedzy bryla a glowica masztu.
 *
 * TEN PLIK MUSI BYC IDENTYCZNY PO OBU STRONACH KABLA. Dlatego zyje w monorepo
 * i jest dolaczany przez oba projekty. Nie jest czescia generycznego
 * flora_link - ten zna tylko typ (uint8_t) i surowy bufor.
 *
 * Przy KAZDEJ zmianie ukladu struktury podbij FLORA_MSGS_VERSION. Inaczej
 * starsza plytka przyjmie nowa ramke jako poprawna i zinterpretuje smieci.
 *
 * KOLEJNOSC BAJTOW: natywna, czyli little-endian - obie koncowki to ESP32-S3,
 * wiec zwykly memcpy spakowanej struktury wystarczy i jest prostszy niz
 * reczna serializacja. Nie uzywaj tego ukladu do narzedzi na hoscie
 * big-endian bez tlumaczenia.
 *
 * BEZ FLOAT NA KABLU: wartosci staloprzecinkowe. Uklad w pamieci jest wtedy
 * oczywisty, a zapis samodokumentujacy sie przez nazwy pol.
 *
 * IDEMPOTENTNOSC KOMEND: dostarczanie jest "co najmniej raz". Jesli glowica
 * wykona komende i straci zasilanie przed wyslaniem ACK, bryla ponowi, a
 * glowica wykona ja ponownie. Dlatego kazda komenda musi byc w formie
 * absolutnej (USTAW_TEMPO = 20), nigdy przyrostowej (ZWIEKSZ_O 5).
 */
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Wersja kontraktu. Podbij przy zmianie ukladu ktorejkolwiek struktury. */
#define FLORA_MSGS_VERSION 2

/** Typy wiadomosci. Wartosci jawne, bo ida po kablu. */
typedef enum {
    /* --- wspolne --- */
    FLORA_MSG_ACK         = 0x01,  /**< bez payloadu; potwierdzany numer jest w naglowku ramki */
    FLORA_MSG_PING        = 0x02,  /**< bez payloadu */
    FLORA_MSG_PONG        = 0x03,  /**< flora_pong_t */

    /* --- glowica -> bryla --- */
    FLORA_MSG_HELLO       = 0x10,  /**< flora_hello_t, po starcie glowicy */
    FLORA_MSG_ATTITUDE    = 0x11,  /**< flora_attitude_t, 20 Hz */
    FLORA_MSG_SUN         = 0x12,  /**< flora_sun_t, 1 Hz */
    FLORA_MSG_WEATHER     = 0x13,  /**< flora_weather_t, 0,1 Hz */
    FLORA_MSG_HEAD_STATUS = 0x14,  /**< flora_head_status_t, 0,2 Hz */

    /* --- bryla -> glowica --- */
    FLORA_MSG_CMD         = 0x20,  /**< flora_cmd_t, wymaga potwierdzenia */
    FLORA_MSG_TIME_SYNC   = 0x21,  /**< flora_time_sync_t */
} flora_msg_type_t;

/** Bity mapy obecnosci czujnikow w HELLO i HEAD_STATUS. */
#define FLORA_SENSOR_VEML_N   (1u << 0)
#define FLORA_SENSOR_VEML_E   (1u << 1)
#define FLORA_SENSOR_VEML_S   (1u << 2)
#define FLORA_SENSOR_VEML_W   (1u << 3)
#define FLORA_SENSOR_BME280   (1u << 4)
#define FLORA_SENSOR_IMU      (1u << 5)
#define FLORA_SENSOR_MAG      (1u << 6)
#define FLORA_SENSOR_RAIN     (1u << 7)

/** Bity jakosci w ATTITUDE. */
#define FLORA_ATT_HEADING_VALID  (1u << 0)
#define FLORA_ATT_MAG_VALID      (1u << 1)
#define FLORA_ATT_SUN_FUSED      (1u << 2)  /**< kurs wsparty kompasem slonecznym */
#define FLORA_ATT_TILTED         (1u << 3)  /**< przechyl zbyt duzy, kurs mniej pewny */

/* ================= WSPOLNE ================= */

typedef struct __attribute__((packed)) {
    uint32_t uptime_ms;
} flora_pong_t;

/* ================= GLOWICA -> BRYLA ================= */

/** Wysylane po starcie glowicy. Bryla nie ufa danym przed odebraniem tego. */
typedef struct __attribute__((packed)) {
    uint8_t  msgs_version;     /**< FLORA_MSGS_VERSION glowicy */
    uint8_t  sensors_present;  /**< mapa bitowa FLORA_SENSOR_* */
    uint16_t _pad;
    uint32_t fw_build;         /**< znacznik budowy, do rozpoznania wersji */
} flora_hello_t;

/**
 * Orientacja. Najczestsza wiadomosc w systemie.
 *
 * Znacznik czasu jest zdejmowany W CHWILI ODCZYTU IMU, nie przy wysylaniu -
 * filtr Kalmana na dole potrzebuje wiedziec, kiedy probka faktycznie powstala,
 * a opoznienie transmisji i szeregowania wnosi jitter, ktory inaczej
 * wszedlby wprost w krok predykcji.
 *
 * Odejmowanie znacznikow dziala poprawnie przez zawiniecie licznika, bo
 * arytmetyka na uint32 jest modulo - ale pamietaj, ze zawija co ~71 minut.
 */
typedef struct __attribute__((packed)) {
    uint32_t timestamp_us;   /**< zegar glowicy w chwili odczytu IMU */
    uint16_t heading_cdeg;   /**< kurs, centystopnie, 0..35999 */
    int16_t  rate_z_cdps;    /**< predkosc katowa wokol pionu, centy-dps (+-327 dps) */
    int16_t  pitch_cdeg;     /**< pochylenie, centystopnie */
    int16_t  roll_cdeg;      /**< przechyl, centystopnie */
    uint8_t  quality;        /**< 0..100, pewnosc kursu */
    uint8_t  flags;          /**< FLORA_ATT_* */
} flora_attitude_t;

/**
 * Kompas sloneczny.
 *
 * Natezenia sa dzielone przez 4, zeby pelne slonce (ponad 100 000 lx)
 * zmiescilo sie w uint16 - zakres siega wtedy 262 140 lx.
 */
typedef struct __attribute__((packed)) {
    uint32_t timestamp_us;
    uint16_t azimuth_cdeg;   /**< kierunek na slonce, centystopnie */
    int16_t  elevation_cdeg; /**< ocena wysokosci nad horyzontem */
    uint16_t lux_div4[4];    /**< N, E, S, W - natezenie podzielone przez 4 */
    uint8_t  valid;          /**< 0 gdy za ciemno, zeby wyznaczyc kierunek */
    uint8_t  _pad;
} flora_sun_t;

typedef struct __attribute__((packed)) {
    uint32_t timestamp_us;
    uint32_t pressure_pa;
    int16_t  temp_cc;        /**< temperatura, setne stopnia */
    uint16_t humidity_cpct;  /**< wilgotnosc, setne procenta */
    uint8_t  rain;           /**< 0 sucho, 255 maksymalny sygnal */
    uint8_t  _pad[3];
} flora_weather_t;

typedef struct __attribute__((packed)) {
    uint32_t uptime_ms;
    uint8_t  sensors_ok;     /**< mapa FLORA_SENSOR_*, czujniki faktycznie odpowiadajace */
    uint8_t  _pad;
    uint16_t error_count;
} flora_head_status_t;

/* ================= BRYLA -> GLOWICA ================= */

typedef enum {
    FLORA_CMD_NOP            = 0x00,
    FLORA_CMD_REBOOT         = 0x01,  /**< idempotentna z natury */
    FLORA_CMD_SET_ATT_HZ     = 0x02,  /**< arg = docelowe Hz dla ATTITUDE */
    FLORA_CMD_CALIBRATE_GYRO = 0x03,  /**< glowica musi byc nieruchoma */
    FLORA_CMD_SET_SUN_GAIN   = 0x04,  /**< arg = indeks czulosci VEML */
} flora_cmd_code_t;

typedef struct __attribute__((packed)) {
    uint8_t  code;           /**< flora_cmd_code_t */
    uint8_t  _pad;
    uint32_t arg;
} flora_cmd_t;

typedef struct __attribute__((packed)) {
    uint32_t bottom_us;      /**< zegar bryly, do powiazania osi czasu */
} flora_time_sync_t;

/* ================= STRAZNICY ROZMIARU ================= */
/*
 * Zmiana ukladu wywali kompilacje tutaj, zamiast objawic sie smieciami
 * na kablu. Te same liczby sluza do walidacji dlugosci w parserze.
 */
_Static_assert(sizeof(flora_pong_t)        ==  4, "flora_pong_t: podbij FLORA_MSGS_VERSION");
_Static_assert(sizeof(flora_hello_t)       ==  8, "flora_hello_t: podbij FLORA_MSGS_VERSION");
_Static_assert(sizeof(flora_attitude_t)    == 14, "flora_attitude_t: podbij FLORA_MSGS_VERSION");
_Static_assert(sizeof(flora_sun_t)         == 18, "flora_sun_t: podbij FLORA_MSGS_VERSION");
_Static_assert(sizeof(flora_weather_t)     == 16, "flora_weather_t: podbij FLORA_MSGS_VERSION");
_Static_assert(sizeof(flora_head_status_t) ==  8, "flora_head_status_t: podbij FLORA_MSGS_VERSION");
_Static_assert(sizeof(flora_cmd_t)         ==  6, "flora_cmd_t: podbij FLORA_MSGS_VERSION");
_Static_assert(sizeof(flora_time_sync_t)   ==  4, "flora_time_sync_t: podbij FLORA_MSGS_VERSION");

/**
 * @brief Oczekiwany rozmiar payloadu dla typu wiadomosci.
 *
 * Parser odrzuca ramke, w ktorej dlugosc nie zgadza sie z typem. Przekrecony
 * bit w polu dlugosci czesto daje wartosc nadal mieszczaca sie w zakresie,
 * wiec samo sprawdzenie gornej granicy jej nie wylapie.
 *
 * @return rozmiar w bajtach, albo -1 dla nieznanego typu
 */
static inline int flora_msgs_payload_size(uint8_t type)
{
    switch (type) {
        case FLORA_MSG_ACK:         return 0;
        case FLORA_MSG_PING:        return 0;
        case FLORA_MSG_PONG:        return (int)sizeof(flora_pong_t);
        case FLORA_MSG_HELLO:       return (int)sizeof(flora_hello_t);
        case FLORA_MSG_ATTITUDE:    return (int)sizeof(flora_attitude_t);
        case FLORA_MSG_SUN:         return (int)sizeof(flora_sun_t);
        case FLORA_MSG_WEATHER:     return (int)sizeof(flora_weather_t);
        case FLORA_MSG_HEAD_STATUS: return (int)sizeof(flora_head_status_t);
        case FLORA_MSG_CMD:         return (int)sizeof(flora_cmd_t);
        case FLORA_MSG_TIME_SYNC:   return (int)sizeof(flora_time_sync_t);
        default:                    return -1;
    }
}

#ifdef __cplusplus
}
#endif
