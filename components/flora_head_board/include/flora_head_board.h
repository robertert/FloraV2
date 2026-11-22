/**
 * @file flora_head_board.h
 * @brief Mapa pinow glowicy masztu: ESP32-S3 z gniazdem kamery.
 *
 * ================== DO POTWIERDZENIA ==================
 * Numery pinow I2C sa WSTEPNE. Plytki S3 z gniazdem kamery (Freenove
 * i podobne) maja wiekszosc wyprowadzen zajeta przez interfejs DVP
 * kamery, wiec wolne piny zaleza od konkretnego modelu.
 *
 * Zanim uruchomisz - sprawdz w dokumentacji swojej plytki, ktore GPIO
 * sa faktycznie wolne, i popraw ponizej. Zle piny oznaczaja albo cisze
 * na magistrali, albo konflikt z kamera.
 *
 * Przy wyborze omijaj:
 *   - piny DVP kamery (zwykle kilkanascie, w tym XCLK, PCLK, VSYNC, HREF, D0-D7)
 *   - 35, 36, 37    PSRAM w wariantach N*R8
 *   - 19, 20        USB
 *   - 43, 44        UART0 (konsola)
 *   - 0, 3, 45, 46  strapping
 * ======================================================
 */
#pragma once

#include "driver/gpio.h"
#include "driver/uart.h"

/* ---- Magistrala I2C czujnikow glowicy ---- */
#define FLORA_HEAD_I2C_SDA      GPIO_NUM_21   /**< potwierdzone na plytce 31.10.2026 */
#define FLORA_HEAD_I2C_SCL      GPIO_NUM_47   /**< potwierdzone na plytce 31.10.2026 */
#define FLORA_HEAD_I2C_FREQ_HZ  100000        /**< 100 kHz: bezpiecznie przy dluzszych przewodach pod kopula */

/* ---- UART w dol masztu (do bryly) ---- */
#define FLORA_HEAD_UART_TX      GPIO_NUM_42   /**< TODO potwierdzic */
#define FLORA_HEAD_UART_RX      GPIO_NUM_41   /**< TODO potwierdzic */
#define FLORA_HEAD_UART_PORT    UART_NUM_1

/* ---- UART w gore (do modulu kamery znacznikow) ----
 * Glowica prowadzi DWA lacza. Modul znacznikow nie ma wlasnego kabla
 * w dol - jest tylko jeden - wiec gada z glowica, a ta przekazuje jego
 * ramki do bryly.
 *
 * Odcinek jest krotki, w obrebie masztu, wiec zaklocenia sa mniejszym
 * problemem niz na dlugim kablu i mozna tu isc szybciej niz 57600.
 */
#define FLORA_HEAD_CAM_TX       GPIO_NUM_38   /**< TODO potwierdzic */
#define FLORA_HEAD_CAM_RX       GPIO_NUM_39   /**< TODO potwierdzic */
#define FLORA_HEAD_CAM_PORT     UART_NUM_2

/* ---- Interfejs DVP kamery ----
 * Pinout plytki Freenove ESP32-S3 WROVER CAM. Oznaczenia Y2..Y9 z
 * dokumentacji Freenove odpowiadaja liniom danych D0..D7.
 *
 * SPRAWDZ, czy masz te sama plytke - kazdy inny modul S3 z gniazdem kamery
 * ma swoj pinout, a wtedy esp_camera_init zglasza blad przy starcie.
 *
 * Te piny sa ZAJETE przez kamere i nie wolno ich uzyc do niczego innego:
 *   4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 15, 16, 17, 18
 * Stad I2C czujnikow na 21/47 i lacza UART na 38-42.
 *
 * SCCB (4/5) to OSOBNA magistrala, obslugiwana przez sterownik kamery.
 * Nie ma nic wspolnego z I2C czujnikow glowicy.
 *
 * KAMERA (od 21.11.2026): obiektyw 68 stopni, patrzy poziomo, wykrywa
 * znaczniki AprilTag (head_tags.c, stale FLORA_HEAD_TAG_* nizej).
 * Obiektyw 160 stopni w dol i wykrywanie przeszkod sa teraz w module
 * tagcam (flora_cam_board.h).
 */
#define FLORA_HEAD_DVP_D0      GPIO_NUM_11   /**< Y2 */
#define FLORA_HEAD_DVP_D1      GPIO_NUM_9    /**< Y3 */
#define FLORA_HEAD_DVP_D2      GPIO_NUM_8    /**< Y4 */
#define FLORA_HEAD_DVP_D3      GPIO_NUM_10   /**< Y5 */
#define FLORA_HEAD_DVP_D4      GPIO_NUM_12   /**< Y6 */
#define FLORA_HEAD_DVP_D5      GPIO_NUM_18   /**< Y7 */
#define FLORA_HEAD_DVP_D6      GPIO_NUM_17   /**< Y8 */
#define FLORA_HEAD_DVP_D7      GPIO_NUM_16   /**< Y9 */
#define FLORA_HEAD_DVP_XCLK    GPIO_NUM_15
#define FLORA_HEAD_DVP_PCLK    GPIO_NUM_13
#define FLORA_HEAD_DVP_VSYNC   GPIO_NUM_6
#define FLORA_HEAD_DVP_HREF    GPIO_NUM_7
#define FLORA_HEAD_DVP_SDA     GPIO_NUM_4    /**< SIOD */
#define FLORA_HEAD_DVP_SCL     GPIO_NUM_5    /**< SIOC */
#define FLORA_HEAD_DVP_PWDN    (-1)          /**< niewyprowadzony na Freenove */
#define FLORA_HEAD_DVP_RESET   (-1)

/**
 * Czy glowica ma w ogole podnosic WiFi i serwer podgladu obrazu.
 *
 * DOMYSLNIE NIE. Obraz po sieci byl potrzebny do wycelowania kamery,
 * zebrania zbioru i sprawdzenia detekcji - wszystko to jest zrobione, a
 * utrzymywanie tego kosztuje trzy razy:
 *
 *  - ENERGIA. Robot jest solarny, a nadajace radio to najwiekszy pojedynczy
 *    pobor na glowicy. Maszt i tak bywa odcinany od zasilania przez bryle.
 *  - TEMPO WIZJI. Serwer HTTP ma wyzszy priorytet niz zadanie wykrywania
 *    przeszkod i walczy z nim o te sama kamere. Zmierzone: przy otwartej
 *    stronie czas na klatke skakal z 233 ms na 1728 ms.
 *  - CIEPLO. Modul kamery grzeje sie sam, a radio doklada swoje.
 *
 * Wpisz 1, gdy bedziesz chcial znowu zebrac klatki albo ustawic wysokosc
 * kamery przez http://<ip>/height. Flaga jest czytana przy starcie, wiec
 * zmiana wymaga przeflashowania - swiadomie, bo to ustawienie serwisowe,
 * nie tryb pracy. Gdyby kiedys mialo byc przelaczane zdalnie, wlasciwym
 * miejscem jest kanal komend z bryly (flora_cmd_code_t i head_cmd.c).
 */
/* TYMCZASOWO 1 (22.11.2026): podglad przy testach znacznikow. Wroc na 0. */
#define FLORA_HEAD_CAM_NET_DEFAULT 1

/**
 * Wlasne WiFi i MQTT glowicy (head_net.c): logi, info o starcie, komendy
 * serwisowe i OTA, niezaleznie od UART-u w maszcie.
 *
 * 1 na czas uruchamiania i testow - bez tego glowice debuguje sie tylko
 * kablem. Na prace w terenie mozna dac 0: radio to najwiekszy pojedynczy
 * pobor na glowicy. Po wylaczeniu OTA glowicy tez nie dziala, wiec powrot
 * do 1 wymaga kabla.
 */
#define FLORA_HEAD_NET_DEFAULT 1

/**
 * Zegar wejsciowy czujnika.
 *
 * CIEPLO, NIE WYDAJNOSC, decyduje o tej liczbie. Modul OV5640 przy 20 MHz
 * i ciaglym strumieniowaniu robi sie goracy w dotyku w kilka minut, bo
 * czujnik pracuje caly czas z pelna szybkoscia niezaleznie od tego, ile
 * klatek faktycznie odbieramy. Polowa zegara to polowa pracy wewnetrznej
 * i wyraznie nizsza temperatura, a przy wykrywaniu przeszkod i tak
 * potrzebujemy 4 klatek na sekunde, nie trzydziestu.
 *
 * Jesli czujnik nie wystartuje przy tej wartosci, aplikacja powinna
 * sprobowac 20 MHz i zapisac to w logu - tak robi bringup_cam.c.
 */
#define FLORA_HEAD_DVP_XCLK_HZ 10000000

/** Makro wypelniajace flora_camera_pins_t pinami tej plytki. */
#define FLORA_HEAD_DVP_PINS()                   \
    {                                           \
        .pin_pwdn = FLORA_HEAD_DVP_PWDN,        \
        .pin_reset = FLORA_HEAD_DVP_RESET,      \
        .pin_xclk = FLORA_HEAD_DVP_XCLK,        \
        .pin_sccb_sda = FLORA_HEAD_DVP_SDA,     \
        .pin_sccb_scl = FLORA_HEAD_DVP_SCL,     \
        .pin_d7 = FLORA_HEAD_DVP_D7, .pin_d6 = FLORA_HEAD_DVP_D6, \
        .pin_d5 = FLORA_HEAD_DVP_D5, .pin_d4 = FLORA_HEAD_DVP_D4, \
        .pin_d3 = FLORA_HEAD_DVP_D3, .pin_d2 = FLORA_HEAD_DVP_D2, \
        .pin_d1 = FLORA_HEAD_DVP_D1, .pin_d0 = FLORA_HEAD_DVP_D0, \
        .pin_vsync = FLORA_HEAD_DVP_VSYNC,      \
        .pin_href = FLORA_HEAD_DVP_HREF,        \
        .pin_pclk = FLORA_HEAD_DVP_PCLK,        \
        .xclk_freq_hz = FLORA_HEAD_DVP_XCLK_HZ, \
    }

/* ---- Znaczniki AprilTag (od 21.11.2026 na glowicy, wczesniej tagcam) ----
 * Dotycza kamery z obiektywem 68 stopni patrzacej poziomo - to ona jest
 * teraz wpieta w glowice. Obiektyw 160 stopni poszedl do modulu tagcam
 * (wykrywanie przeszkod, flora_cam_board.h).
 *
 * Do ustalenia po wybraniu rozmiaru obszaru roboczego: od niego zalezy
 * fizyczny rozmiar znacznikow na palikach, a od tego zasieg wykrywania.
 */
/**
 * Bok czarnego kwadratu znacznika. 300 mm wybrane pod maksymalny zasieg:
 * AprilTag potrzebuje okolo 8 pikseli na bok, zeby zdekodowac 36h11, co
 * przy ogniskowej 297 px daje
 *     zasieg = 297 * 300 / 8 = 11,1 m
 * Przy 150 mm byloby to 5,6 m.
 */
/* TYMCZASOWO 152 (22.11.2026): wydruk testowy A4. Docelowe paliki: 300. */
#define FLORA_HEAD_TAG_SIZE_MM   152
/**
 * Zasieg wyliczony z geometrii, do potwierdzenia pomiarem w polu -
 * swiatlo, ostrosc obiektywu i kontrast wydruku moga go skrocic.
 */
#define FLORA_HEAD_TAG_MAX_RANGE_MM  11000

/**
 * Decymacja przed szukaniem krawedzi. JEDEN znaczy pelna rozdzielczosc,
 * czyli maksymalny zasieg kosztem tempa - detektor pracuje na
 * czterokrotnie wiekszym obrazie niz przy decymacji 2.
 *
 * Spodziewane tempo to 0,3-0,5 klatki na sekunde. Przy robocie jadacym
 * ponizej 0,2 m/s znaczy to poprawke pozycji co 2-3 sekundy i kilkadziesiat
 * centymetrow przejechanych miedzy nimi - filtr Kalmana pokrywa te luke
 * odometria. Biblioteka przyjmuje tez wartosci niecalkowite, wiec 1,5 jest
 * sensownym kompromisem, gdyby tempo okazalo sie za niskie.
 */
#define FLORA_HEAD_TAG_QUAD_DECIMATE 1.0f

/**
 * ASYMETRIA DOKLADNOSCI, wazna dla filtra na dole.
 *
 * Azymut liczymy z polozenia srodka znacznika, wiec jego rozdzielczosc to
 * 0,19 stopnia na CALYM zasiegu. Odleglosc liczymy z pozornego rozmiaru,
 * a ten maleje z kwadratem - blad jednego piksela w pomiarze boku daje:
 *
 *    1 m  ->   11 mm bledu
 *    3 m  ->   98 mm
 *    6 m  ->  379 mm
 *    9 m  ->  826 mm
 *   11 m  -> 1209 mm
 *
 * Czyli na granicy zasiegu odleglosc ma ponad metr niepewnosci, a azymut
 * jest nadal dokladny. Filtr powinien ufac KIERUNKOWI znacznie bardziej
 * niz ODLEGLOSCI - dlatego pole `quality` w flora_tag_t maleje razem
 * z pozornym rozmiarem.
 */

/**
 * Ogniskowa w pikselach, wyliczona z katalogowego kata widzenia obiektywu.
 *
 * Obiektyw ma 68 stopni PO PRZEKATNEJ. Przy obrazie 320x240 daje to
 *   FOV poziomy = 2 * atan( tan(34 st) / (przekatna/szerokosc) ) = 56,7 st
 *   ogniskowa   = 160 / tan(28,35 st) = 297 px
 *
 * To WARTOSC WYJSCIOWA, nie koncowa. Katalogowy kat jest podawany
 * z tolerancja, a montaz czujnika wzgledem obiektywu tez wnosi blad.
 * Uscislij pomiarem: postaw znacznik na zmierzonej odleglosci, odczytaj
 * z logu pozorny bok w pikselach i policz
 *     ogniskowa = bok_px * odleglosc_mm / bok_znacznika_mm
 * Przy tym obiektywie dystorsja jest niewielka (to nie szerokokatny),
 * wiec jeden pomiar wystarczy.
 */
#define FLORA_HEAD_TAG_FOCAL_PX      297.0f

/**
 * Kamera glowicy daje obraz ODBITY LUSTRZANIE (lewo-prawo). Sprawdzone
 * 22.11.2026 na klatce z /raw: znacznik 36h11 nie dekodowal sie wcale,
 * a po odbiciu dal id 0 z decision_margin 134. Odbity znacznik to dla
 * detektora inny wzor, wiec head_tags.c odbija klatke przy zamianie na
 * szarosc. Tylko dla znacznikow - flora_camera i podglad zostaja bez zmian.
 * 0 po wymianie modulu kamery na taki, ktory daje obraz prosty.
 */
#define FLORA_HEAD_TAG_MIRROR        1

/* ---- Klucze NVS glowicy ----
 * Wlasne, nie z flora_board.h - ten opisuje bryle, a glowica ma osobna
 * partycje NVS we wlasnym flashu. Limit NVS to 15 znakow na klucz.
 */
#define FLORA_HEAD_NVS_NS        "head"
#define FLORA_HEAD_NVS_GYRO_X    "gyro_bias_x"
#define FLORA_HEAD_NVS_GYRO_Y    "gyro_bias_y"
#define FLORA_HEAD_NVS_GYRO_Z    "gyro_bias_z"
#define FLORA_HEAD_NVS_GYRO_OK   "gyro_cal_ok"
/* Kalibracja magnetometru (flora_magcal): hard iron, soft iron, odniesienie. */
#define FLORA_HEAD_NVS_MAG_OX    "mag_ox"
#define FLORA_HEAD_NVS_MAG_OY    "mag_oy"
#define FLORA_HEAD_NVS_MAG_W00   "mag_w00"
#define FLORA_HEAD_NVS_MAG_W01   "mag_w01"
#define FLORA_HEAD_NVS_MAG_W11   "mag_w11"
#define FLORA_HEAD_NVS_MAG_HREF  "mag_href"
#define FLORA_HEAD_NVS_MAG_DIP   "mag_dip"
#define FLORA_HEAD_NVS_MAG_OZ    "mag_oz"
/* Klucz "kalibracja wazna" zmienia sie razem z modelem, zeby stara nie
 * wczytala sie po aktualizacji: "mag_cal_ok" - stare osie AK09916,
 * "mag_cal_ok2" - 2D na surowych x, y (bez oz, przeciek przechylu),
 * "mag_cal_ok3" - hard iron 3D z inklinacji miejsca (flora_magcal.h). */
#define FLORA_HEAD_NVS_MAG_OK    "mag_cal_ok3"

/**
 * Inklinacja magnetyczna miejsca pracy [st], dodatnia. Z niej kalibracja
 * wylicza pionowe przesuniecie zera magnetometru (flora_magcal.h).
 * Musi byc dokladna do ~0,5 st: 1 st bledu to ~2 st kursu na 10 st
 * pochylosci. TYMCZASOWO Krakow - podmienic na wartosc z kalkulatora
 * NOAA (https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml,
 * pole "Inclination") dla wspolrzednych ogrodu, jak deklinacje w bryle.
 */
#define FLORA_HEAD_MAG_INCL_DEG  66.5f
/* Poziom akcelerometru: przekrzywienie plytki wzgledem masztu [st]. */
#define FLORA_HEAD_NVS_LVL_PITCH "lvl_pitch"
#define FLORA_HEAD_NVS_LVL_ROLL  "lvl_roll"
#define FLORA_HEAD_NVS_LVL_OK    "lvl_ok"

/* ---- Adresy na magistrali ---- */
#define FLORA_HEAD_ADDR_MUX     0x70  /**< TCA9548A, A0-A2 do masy */
#define FLORA_HEAD_ADDR_VEML    0x10  /**< VEML7700, adres staly - stad multiplekser */
#define FLORA_HEAD_ADDR_BME280  0x76  /**< 0x77 gdy SDO podciagniete */
#define FLORA_HEAD_ADDR_ICM     0x68  /**< 0x69 gdy AD0 w stanie wysokim */
#define FLORA_HEAD_ADDR_AK09916 0x0C  /**< magnetometr w ICM-20948, widoczny po wlaczeniu bypass */

/* ---- Kompas sloneczny: ktory VEML na ktorym kanale multipleksera ----
 * Krzyz z przegrodami, czujniki patrza w cztery strony. Przypisanie
 * kanalow do kierunkow zalezy od montazu w uchwycie - sprawdzic przy
 * pierwszym uruchomieniu, zaslaniajac po jednym czujniku.
 */
#define FLORA_HEAD_SUN_CH_N     0
#define FLORA_HEAD_SUN_CH_E     1
#define FLORA_HEAD_SUN_CH_S     6   /**< kanaly 2 i 3 nieuzywane w montazu */
#define FLORA_HEAD_SUN_CH_W     7
#define FLORA_HEAD_SUN_COUNT    4
