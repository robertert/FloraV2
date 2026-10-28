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
#define FLORA_MSGS_VERSION 3

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
    FLORA_MSG_OBSTACLES   = 0x15,  /**< flora_obstacles_t, kamera przednia glowicy */

    /* --- modul kamery znacznikow -> bryla (przez glowice) ---
     * Typy sa globalnie unikalne w calym systemie, wiec glowica przekazuje
     * te ramki w dol bez zadnego pola adresu - bryla rozpoznaje nadawce
     * po samym typie wiadomosci. */
    FLORA_MSG_TAGS        = 0x30,  /**< flora_tags_t */
    FLORA_MSG_CAM_STATUS  = 0x31,  /**< flora_cam_status_t */

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

/**
 * Przeszkody widziane przez kamere przednia glowicy.
 *
 * Pole widzenia podzielone na sektory, w kazdym odleglosc do najblizszej
 * przeszkody. Taki zapis laczy sie wprost z ultradzwiekami, ktore juz sa
 * w bryle, i wystarcza do omijania - w przeciwienstwie do samej flagi
 * "wolno / zablokowane".
 *
 * ZNACZENIE WARTOSCI W SEKTORZE:
 *   0        nieznane - kamera nie potrafila ocenic (cien, przeswietlenie)
 *   1..253   odleglosc w centymetrach
 *   254      wolne do konca zasiegu
 *   255      zarezerwowane
 * Zero i 254 to NIE to samo: pierwsze znaczy "nie wiem", drugie "czysto".
 * Mylenie ich przy omijaniu konczy sie wjechaniem w przeszkode, ktorej
 * kamera nie rozpoznala.
 */
#define FLORA_OBSTACLE_SECTORS 16
#define FLORA_OBSTACLE_UNKNOWN 0
#define FLORA_OBSTACLE_CLEAR   254

typedef struct __attribute__((packed)) {
    uint32_t timestamp_us;
    uint16_t fov_cdeg;       /**< szerokosc pola widzenia, centystopnie */
    uint8_t  sectors[FLORA_OBSTACLE_SECTORS];  /**< od lewej do prawej */
    uint8_t  valid;          /**< 0 gdy caly obraz bezuzyteczny */
    uint8_t  _pad;
} flora_obstacles_t;

/* ================= MODUL ZNACZNIKOW -> BRYLA ================= */

/** Ile znacznikow miesci sie w jednej ramce. */
#define FLORA_TAGS_MAX 4

/**
 * Pojedyncza detekcja znacznika AprilTag.
 *
 * Wysylamy SUROWE POMIARY, nie wyliczona pozycje robota: filtr Kalmana na
 * dole sam wazy kazda detekcje wedlug `quality` i odleglosci, a modul
 * kamery nie musi znac mapy palikow. Wyliczanie pozycji w module
 * odebraloby filtrowi informacje o niepewnosci.
 */
typedef struct __attribute__((packed)) {
    uint16_t id;             /**< identyfikator znacznika */
    uint16_t distance_mm;    /**< odleglosc do znacznika */
    int16_t  bearing_cdeg;   /**< kat wzgledem osi kamery, centystopnie */
    uint8_t  quality;        /**< 0..100, pewnosc detekcji */
    uint8_t  _pad;
} flora_tag_t;

typedef struct __attribute__((packed)) {
    uint32_t    timestamp_us;  /**< zegar MODULU KAMERY, nie bryly */
    uint8_t     count;         /**< ile pozycji w tags[] jest wypelnionych */
    uint8_t     _pad[3];
    flora_tag_t tags[FLORA_TAGS_MAX];
} flora_tags_t;

typedef struct __attribute__((packed)) {
    uint32_t uptime_ms;
    uint16_t frames_processed; /**< klatek od ostatniego raportu */
    uint16_t error_count;
    uint8_t  camera_ok;
    uint8_t  _pad[3];
} flora_cam_status_t;

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
_Static_assert(sizeof(flora_obstacles_t)   == 24, "flora_obstacles_t: podbij FLORA_MSGS_VERSION");
_Static_assert(sizeof(flora_tag_t)         ==  8, "flora_tag_t: podbij FLORA_MSGS_VERSION");
_Static_assert(sizeof(flora_tags_t)        == 40, "flora_tags_t: podbij FLORA_MSGS_VERSION");
_Static_assert(sizeof(flora_cam_status_t)  == 12, "flora_cam_status_t: podbij FLORA_MSGS_VERSION");

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
        case FLORA_MSG_OBSTACLES:   return (int)sizeof(flora_obstacles_t);
        case FLORA_MSG_TAGS:        return (int)sizeof(flora_tags_t);
        case FLORA_MSG_CAM_STATUS:  return (int)sizeof(flora_cam_status_t);
        default:                    return -1;
    }
}

#ifdef __cplusplus
}
#endif
