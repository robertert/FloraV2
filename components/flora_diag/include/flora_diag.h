/**
 * @file flora_diag.h
 * @brief Logi przez MQTT i informacja o starcie - wspolne dla bryly i glowicy.
 *
 * Opis dzialania (ktore linie ida na MQTT, bufor startu) w flora_diag.c.
 */
#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /** Tematy wzgledne wobec bazy MQTT. NULL bierze "tele/log" i "tele/boot". */
    const char *log_topic;
    const char *boot_topic;
    /** Raport pamieci co 10 s (np. "head/sys"); NULL = bez raportu. */
    const char *sys_topic;
} flora_diag_cfg_t;

/**
 * Podpina przechwytywanie logow. Wolac zaraz po NVS, przed startem sieci -
 * linie czekaja w kolejce i wychodza po polaczeniu z brokerem.
 */
esp_err_t flora_diag_init(const flora_diag_cfg_t *cfg);

/** "<tag|*> <E|W|I|D|V>" - poziom logow wysylanych na MQTT. Nie trafia do NVS. */
esp_err_t flora_diag_set_level(const char *arg);

/**
 * Publikuje info o starcie (retain): wersja, powod restartu, licznik startow.
 * Dodatkowo <boot_topic>_prev: slad POPRZEDNIEGO uruchomienia z pamieci RTC
 * (wersja, czas pracy, czy bylo WiFi i broker, minimum sterty) - takze
 * takiego, ktore nigdy nie dotarlo do brokera, np. wersji wycofanej po OTA.
 */
void flora_diag_publish_boot(void);

#ifdef __cplusplus
}
#endif
