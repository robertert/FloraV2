/**
 * @file hub_diag.h
 * @brief Zdalna diagnostyka: logi przez MQTT i informacja o starcie.
 */
#pragma once

#include "esp_err.h"

/**
 * @brief Liczy start, zapamietuje powod restartu i podpina logi pod MQTT.
 *
 * Wolac jak najwczesniej, zaraz po flora_store_init() - ostrzezenia i bledy
 * z inicjalizacji sprzetu (np. brak czujnika) czekaja w kolejce i wychodza
 * na tele/log, gdy tylko robot polaczy sie z brokerem.
 *
 * Na MQTT ida zawsze W i E, caly start od poziomu I (bufor do pierwszego
 * polaczenia) i to, co wlaczy hub_diag_set_level. Konsola USB jak dotad.
 */
esp_err_t hub_diag_init(void);

/**
 * @brief Wiecej logow na MQTT na czas szukania bledu (cmd/log).
 *
 * @param arg "<tag> <E|W|I|D|V>" - jeden modul, np. "flora_sonar D",
 *            "* <poziom>" - wszystko (kasuje reguly dla tagow), "* W" = domyslnie.
 *            Najwyzej 8 tagow naraz; nie zapisywane, po restarcie W.
 * @return ESP_ERR_NOT_SUPPORTED gdy poziom wyciety przy kompilacji
 *         (CONFIG_LOG_MAXIMUM_LEVEL), ESP_ERR_NO_MEM gdy brak miejsca na tag
 */
esp_err_t hub_diag_set_level(const char *arg);

/**
 * @brief Publikuje tele/boot: wersje, date budowania, powod restartu,
 *        licznik startow. Z flaga retain, wolac po kazdym polaczeniu.
 */
void hub_diag_publish_boot(void);
