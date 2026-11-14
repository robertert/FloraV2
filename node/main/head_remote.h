/**
 * @file head_remote.h
 * @brief Komendy serwisowe glowicy z brokera (headcmd/...).
 *
 *   headcmd/log     "<tag|*> <E|W|I|D|V>"   poziom logow na MQTT
 *   headcmd/reboot  cokolwiek               restart glowicy
 *   headcmd/ota     "latest" | "head-vX.Y.Z" [force]
 *   headcmd/config/<klucz>  wifi_ssid, wifi_pass, mqtt_uri, mqtt_user,
 *                           mqtt_pass; "-" kasuje. Dziala po restarcie.
 *
 * Odpowiedz: headack/<komenda> = "ok" albo nazwa bledu. Osobny prefiks niz
 * cmd/ bryly, bo obie jednostki siedza na tej samej bazie tematow.
 */
#pragma once

#include "esp_err.h"

esp_err_t head_remote_start(void);
