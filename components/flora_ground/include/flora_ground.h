/**
 * @file flora_ground.h
 * @brief Wykrywanie wolnej przestrzeni wokol robota z obrazu RGB565.
 *
 * Odpowiada na jedno pytanie: "gdzie moge jechac i jak daleko", w postaci
 * 16 odleglosci w centymetrach, gotowych do wpisania w `flora_obstacles_t`.
 *
 * ZASADA, ktora pozwala temu dzialac na kostce, betonie, tarasie i trawie
 * bez uczenia czegokolwiek: nie wiemy, jak wyglada przeszkoda. W KAZDEJ
 * klatce od nowa uczymy sie, jak wyglada PODLOZE tutaj i teraz, i szukamy
 * miejsc, w ktorych przestaje tak wygladac. Robot nie wie, po czym jedzie -
 * wie tylko, ze to, co pod nim, jest dobre.
 *
 * DLACZEGO BARWA, A NIE JASNOSC: na jednolitym betonie cien ma wiekszy
 * kontrast niz prawdziwa przeszkoda. Chromatycznosc znormalizowana
 * (udzial kanalu w sumie kanalow) jest z definicji odporna na jasnosc, wiec
 * zacieniony beton zostaje betonem.
 *
 * DLACZEGO SIATKA POLARNA: kamera patrzy prosto w dol, wiec obraz to widok
 * z gory na kolo podloza wokol robota, a odleglosc od srodka kadru jest
 * odlegloscia od robota. Tniemy wiec kadr jak tarcze: PIERSCIENIE to
 * odleglosc, WYCINKI to kierunek.
 *
 * ================== ZERO ZALEZNOSCI OD ESP-IDF ==================
 * Ten komponent nie zna ani esp_err_t, ani ESP_LOG, ani FreeRTOS, i nie
 * alokuje pamieci sam. To jest decyzja projektowa, nie przypadek: dzieki
 * niej TEN SAM plik .c kompiluje sie zwyklym gcc na komputerze i daje sie
 * przepuscic przez zebrany zbior klatek. Bez tego port prototypu z Pythona
 * do C rozjechalby sie po cichu i nikt by tego nie zauwazyl.
 *
 * Pamiec podaje wolajacy: `flora_ground_arena_size()` mowi ile, a na ESP
 * nalezy ja wziac z PSRAM (heap_caps_malloc), bo same tablice przegladowe
 * to kilkaset kB.
 * ================================================================
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Tyle, ile FLORA_OBSTACLE_SECTORS w kontrakcie wiadomosci. */
#define FLORA_GROUND_SECTORS   16
#define FLORA_GROUND_MAX_RINGS 16

/** Znaczenia takie same jak w `flora_obstacles_t`, zeby nie tlumaczyc. */
#define FLORA_GROUND_UNKNOWN 0
#define FLORA_GROUND_CLEAR   254

typedef struct {
    uint16_t width, height;     /**< rozmiar klatki RGB565 */
    uint8_t  rings;             /**< pierscieni na wycinek, max FLORA_GROUND_MAX_RINGS */

    /** Promien wewnetrzny: obrys bryly robota, ktory kamera widzi pod soba. */
    float r_in_px;
    /** Promien zewnetrzny: dalej dystorsja 160 stopni zabija rozdzielczosc. */
    float r_out_px;

    /** Szerokosc wachlarza i kierunek jazdy w obrazie, w stopniach. */
    float fan_deg;
    float forward_deg;

    /**
     * Geometria nadiru: `rho = h * tan(r / f)` dla obiektywu
     * rownoodleglosciowego. Przy 160 stopniach po przekatnej i obrazie
     * 320x240 wychodzi f okolo 143 px/rad. Wysokosc obiektywu nad ziemia
     * trzeba ZMIERZYC - bez niej odleglosci sa tylko indeksami pierscieni.
     */
    float cam_height_mm;
    float focal_px_per_rad;

    /** Minimalny udzial pikseli komorki zgodnych z wzorcem podloza, 0..1. */
    float thr;
    /**
     * Jaki udzial pikseli pierscienia uznajemy za podloze, 0..1. Reszta
     * rozkladu barw to przeszkoda.
     *
     * Zmierzone na zbiorze z 1-2 listopada 2026, przy pominieciu wlasnego
     * wycinka w odniesieniu:
     *   0,90 - zero falszywek na 12 klatkach czystej podlogi, ale szeroka
     *          przeszkoda znika calkowicie
     *   0,85 - obie syntetyczne przeszkody wykryte, jedna falszywka na 12
     *   0,75 - dwie falszywki na 12
     * Wybrane 0,85, bo plan stawia prog przegapionej przeszkody na 2%, a
     * falszywego zatrzymania na 10%: osiem procent falszywek miesci sie w
     * budzecie, a przegapiona cegla nie.
     *
     * ZNANE OGRANICZENIE: przeszkoda wypelniajaca wiecej niz jakies 15%
     * pierscienia nadal potrafi sie ukryc, bo wchodzi do odniesienia przez
     * sasiednie wycinki. Przy nadirze i zasiegu metra znaczy to obiekt
     * szerszy niz mniej wiecej trzy wycinki na danym promieniu.
     */
    float ring_cover;
    /** Suma kanalow ponizej tego to piksel bez wiarygodnej barwy. */
    uint16_t dark_sum;
    /** Kanal powyzej tego to przester. */
    uint8_t bright;
    /** Ile ostatnich wzorcow podloza pamietac (zmiana powierzchni). */
    uint8_t memory;
} flora_ground_cfg_t;

/** Wartosci sprawdzone na zbiorze z 1-2 listopada 2026. */
#define FLORA_GROUND_DEFAULT_CFG()        \
    {                                     \
        .width  = 320,                    \
        .height = 240,                    \
        .rings  = 12,                     \
        .r_in_px  = 40.0f,                \
        .r_out_px = 150.0f,               \
        .fan_deg     = 160.0f,            \
        .forward_deg = 0.0f,              \
        .focal_px_per_rad = 143.0f,       \
        .thr        = 0.70f,              \
        .ring_cover = 0.85f,              \
        .dark_sum   = 90,                 \
        .bright     = 246,                \
        .memory     = 3,                  \
    }

typedef struct {
    /** Gotowe do `flora_obstacles_t.sectors`: cm, 0 nieznane, 254 czysto. */
    uint8_t sectors[FLORA_GROUND_SECTORS];
    /** Indeks pierscienia, w ktorym zaczyna sie przeszkoda; 255 gdy brak. */
    uint8_t first_ring[FLORA_GROUND_SECTORS];
    /** Zero gdy klatka bezuzyteczna - idzie wprost w pole `valid`. */
    uint8_t valid;

    /**
     * Decyzja na komorke, indeks `ring * FLORA_GROUND_SECTORS + sector`:
     * 0 poza kadrem, 1 nie wiem, 2 podloze, 3 przeszkoda. Do diagnostyki i
     * do rysowania nakladki - bez tego nie da sie dojsc, czemu detektor
     * czegos nie widzi.
     */
    uint8_t cell_verdict[FLORA_GROUND_SECTORS * FLORA_GROUND_MAX_RINGS];

    uint16_t unusable_cells;
    /** Promile pikseli odrzuconych na progach jasnosci. */
    uint16_t lost_px_permille;
    uint32_t process_us;
} flora_ground_result_t;

/** Ile pamieci roboczej potrzeba przy tej konfiguracji. */
size_t flora_ground_arena_size(const flora_ground_cfg_t *cfg);

/**
 * @brief Buduje tablice przegladowe. Wolac raz.
 *
 * @param arena bufor o rozmiarze z flora_ground_arena_size(), nalezy do
 *        wolajacego i musi zyc tak dlugo jak detektor. Na ESP z PSRAM.
 * @return 0 gdy dobrze, ujemne gdy konfiguracja bez sensu
 */
int flora_ground_init(const flora_ground_cfg_t *cfg, void *arena, size_t arena_len);

/**
 * @brief Przetwarza jedna klatke.
 *
 * @param rgb565 width*height*2 bajtow, big-endian jak oddaje esp32-camera
 * @return 0 gdy dobrze
 */
int flora_ground_process(const uint8_t *rgb565, flora_ground_result_t *out);

/** Zapomina wzorce podloza. Do wolania przy zmianie sceny albo po pauzie. */
void flora_ground_reset(void);

/** Srodek pierscienia w milimetrach na ziemi; 0 gdy nie podano wysokosci. */
float flora_ground_ring_mm(uint8_t ring);

#ifdef __cplusplus
}
#endif
