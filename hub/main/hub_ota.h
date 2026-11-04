/**
 * @file hub_ota.h
 * @brief Aktualizacja firmware przez siec (OTA) z serwera robota.
 */
#pragma once

#include <stdbool.h>

#include "esp_err.h"

/**
 * @brief Po starcie: jesli to pierwsze uruchomienie nowej wersji, pilnuje
 *        jej potwierdzenia.
 *
 * Nowa wersja startuje "na probe". Potwierdza sie sama po minucie stabilnego
 * polaczenia z brokerem; bez polaczenia przez 5 min wraca do poprzedniej.
 * Awaria przed potwierdzeniem cofa sie w bootloaderze. Wolac wczesnie
 * w app_main - odliczanie biegnie od startu.
 */
void hub_ota_check_pending(void);

/**
 * @brief Startuje aktualizacje w tle (komenda cmd/ota).
 *
 * @param arg  "latest", "hub-vX.Y.Z", albo jedno z nich z sufiksem " force"
 *             (instaluje takze te sama wersje)
 * @return ESP_OK gdy zadanie ruszylo - wynik przychodzi na tele/ota;
 *         ESP_ERR_INVALID_ARG zly argument; ESP_ERR_INVALID_STATE naped albo
 *         pompka pracuja, trwa inna aktualizacja, albo nowa wersja czeka
 *         jeszcze na potwierdzenie
 */
esp_err_t hub_ota_start(const char *arg);

/** true w trakcie pobierania i instalacji - naped i pompka sa wtedy blokowane. */
bool hub_ota_in_progress(void);
