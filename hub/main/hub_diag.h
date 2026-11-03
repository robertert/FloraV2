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
 * Na MQTT ida tylko poziomy W i E. Konsola USB dostaje wszystko jak dotad.
 */
esp_err_t hub_diag_init(void);

/**
 * @brief Publikuje tele/boot: wersje, date budowania, powod restartu,
 *        licznik startow. Z flaga retain, wolac po kazdym polaczeniu.
 */
void hub_diag_publish_boot(void);
