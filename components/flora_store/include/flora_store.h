/**
 * @file flora_store.h
 * @brief Trwale ustawienia i kalibracje w NVS.
 *
 * Po co: wszystkie kalibracje robota siedza dzis w kodzie i kazda zmiana
 * wymaga rekompilacji i wgrania. Punkty kalibracyjne gleby, przelozenie
 * przekladni, flagi odwrocenia silnikow i enkoderow, przypisanie kanalow
 * multipleksera, offset zyroskopu - w terenie, z robotem na trawniku,
 * przekompilowanie zeby poprawic prog wilgotnosci to zla droga.
 *
 * TRZY DECYZJE, KTORE WARTO ZNAC:
 *
 * 1. Wartosc domyslna NIE jest zapisywana przy odczycie. Odczyt, ktory po
 *    cichu pisze do flasha, to zaskakujacy efekt uboczny i niepotrzebne
 *    zuzycie pamieci. Brak klucza zwraca domyslna i tyle.
 *
 * 2. Zapis jest zatwierdzany od razu. Kalibracje pisze sie rzadko, wiec
 *    pewnosc, ze wartosc przetrwa nagle odciecie zasilania, jest warta
 *    wiecej niz oszczedzanie cykli kasowania.
 *
 * 3. Liczby zmiennoprzecinkowe ida jako u32 przez przepisanie bitow - NVS
 *    nie ma typu float. W zrzucie zawartosci zobaczysz je jako U32
 *    z pozornie bezsensowna wartoscia; to normalne.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Limit NVS: 15 znakow plus zakonczenie. Dluzsze klucze sa odrzucane. */
#define FLORA_STORE_MAX_KEY_LEN 15

typedef struct flora_store_s *flora_store_t;

/**
 * @brief Inicjalizuje podsystem NVS. Wolaj RAZ, jako jedno z pierwszych.
 *
 * Zawiera kanoniczna obsluge sytuacji, w ktorej partycja jest pelna albo
 * zapisana w starszym formacie: kasuje ja i inicjalizuje ponownie.
 *
 * Ten sam NVS jest potrzebny przez WiFi (kalibracja PHY) i NimBLE, wiec
 * wywolanie musi poprzedzac ich uruchomienie.
 */
esp_err_t flora_store_init(void);

/**
 * @brief Otwiera przestrzen nazw do odczytu i zapisu.
 *
 * @param ns  nazwa, do 15 znakow
 */
esp_err_t flora_store_open(const char *ns, flora_store_t *out);
void      flora_store_close(flora_store_t h);

/* --- Odczyt z wartoscia domyslna. `*out` jest zawsze wypelniane. --- */

esp_err_t flora_store_get_i32  (flora_store_t h, const char *key, int32_t  *out, int32_t  def);
esp_err_t flora_store_get_u32  (flora_store_t h, const char *key, uint32_t *out, uint32_t def);
esp_err_t flora_store_get_bool (flora_store_t h, const char *key, bool     *out, bool     def);
esp_err_t flora_store_get_float(flora_store_t h, const char *key, float    *out, float    def);

/**
 * @param cap  pojemnosc bufora razem z zakonczeniem
 */
esp_err_t flora_store_get_str  (flora_store_t h, const char *key, char *out, size_t cap,
                                const char *def);

/**
 * @param len  wejscie: pojemnosc bufora; wyjscie: faktyczny rozmiar
 * @return ESP_ERR_NOT_FOUND gdy klucza nie ma (bufor nietkniety)
 */
esp_err_t flora_store_get_blob (flora_store_t h, const char *key, void *out, size_t *len);

/* --- Zapis. Kazdy zatwierdza od razu. --- */

esp_err_t flora_store_set_i32  (flora_store_t h, const char *key, int32_t  v);
esp_err_t flora_store_set_u32  (flora_store_t h, const char *key, uint32_t v);
esp_err_t flora_store_set_bool (flora_store_t h, const char *key, bool     v);
esp_err_t flora_store_set_float(flora_store_t h, const char *key, float    v);
esp_err_t flora_store_set_str  (flora_store_t h, const char *key, const char *v);
esp_err_t flora_store_set_blob (flora_store_t h, const char *key, const void *v, size_t len);

/* --- Porzadki i diagnoza --- */

bool      flora_store_has(flora_store_t h, const char *key);
esp_err_t flora_store_erase_key(flora_store_t h, const char *key);

/** Kasuje CALA przestrzen nazw. Do przywracania ustawien fabrycznych. */
esp_err_t flora_store_erase_namespace(flora_store_t h);

/**
 * @brief Wypisuje zawartosc przestrzeni nazw do logu.
 *
 * Pierwsza rzecz przy pytaniu "czy ta kalibracja w ogole sie zapisala".
 */
esp_err_t flora_store_dump(flora_store_t h);

/** Wypisuje zajetosc partycji NVS - ile wpisow wolnych, uzytych, uszkodzonych. */
esp_err_t flora_store_report_space(void);

#ifdef __cplusplus
}
#endif
