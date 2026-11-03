/**
 * @file hub_config.h
 * @brief Ustawienia bryly zapisywane do NVS przez MQTT.
 */
#pragma once

#include "esp_err.h"

/**
 * @brief Zapisuje jedno ustawienie do NVS.
 *
 * Klucze (temat cmd/config/<klucz>, payload to wartosc):
 *   wifi_ssid, wifi_pass, mqtt_uri,
 *   mqtt_user, mqtt_pass            - dzialaja po restarcie (cmd/reboot)
 *   soil_dry, soil_wet              - dzialaja od razu
 *
 * Wartosc "-" dla kluczy tekstowych kasuje klucz (wraca wartosc
 * wkompilowana), np. mqtt_user "-" wylacza logowanie do brokera.
 *
 * Zly adres albo haslo nie odcinaja robota na dobre: przy nieudanym
 * polaczeniu bringup_net_connect() wraca do wartosci wkompilowanych.
 *
 * @return ESP_ERR_NOT_SUPPORTED dla nieznanego klucza,
 *         ESP_ERR_INVALID_ARG dla zlej wartosci
 */
esp_err_t hub_config_set(const char *key, const char *value);

/** Publikuje biezace ustawienia na "config" (retain, bez hasla). */
void hub_config_publish(void);
