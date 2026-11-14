/**
 * @file flora_ota.h
 * @brief Aktualizacja firmware przez siec - wspolna dla bryly i glowicy.
 *
 * Opis przebiegu (manifest, suma SHA-256, proba i powrot) w flora_ota.c.
 */
#pragma once

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /** Poczatek nazwy wersji, np. "hub-v". Tylko "<prefiks>X.Y.Z" przejdzie. */
    const char *version_prefix;
    /** Katalog na serwerze: https://<host>/ota/<dir>/. */
    const char *dir;
    /** Temat stanu aktualizacji, wzgledny wobec bazy MQTT (np. "tele/ota"). */
    const char *topic;
    /** Czy wolno teraz zaczac (np. robot stoi). NULL = zawsze wolno. */
    bool (*may_start)(void);
    /** Wolane tuz przed startem pobierania: zatrzymaj sprzet. NULL = nic. */
    void (*make_safe)(void);
} flora_ota_cfg_t;

/**
 * Zapamietuje konfiguracje (wskazniki musza zyc do konca programu) i, jesli
 * biezaca wersja startuje pierwszy raz po OTA, zaczyna odliczanie do jej
 * potwierdzenia. Wolac raz, wczesnie po starcie.
 */
void flora_ota_init(const flora_ota_cfg_t *cfg);

/** "<wersja>" albo "<wersja> force"; wersja to "latest" lub "<prefiks>X.Y.Z". */
esp_err_t flora_ota_start(const char *arg);

/** true w trakcie pobierania i instalacji. */
bool flora_ota_in_progress(void);

#ifdef __cplusplus
}
#endif
