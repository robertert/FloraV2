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
 *   drive_mode (pwm|pid), pid_kp, pid_ki, pid_kd, pid_kff, pid_ks,
 *   pid_acc, pid_vmax               - dzialaja od razu, patrz hub_drive.h
 *                                     ("-" wraca do domyslnej)
 *   pump_ml_s                       - dziala od razu, patrz hub_sensorcal.h
 *   ina_shunt (mOhm)                - dziala po restarcie
 *   track_mm, wheel_mm, wheel_ratio, cam_x_mm, cam_y_mm,
 *   site_lat, site_lon, mag_decl    - dzialaja od razu, patrz hub_params.h
 *   guard_en, pilot_en, water_en, mission_en, sleep_en, mdd_key_en
 *   oraz progi g_*, p_*, w_*, s_*, e_* - funkcje autonomiczne, domyslnie
 *                                     wylaczone, patrz hub_autocfg.h
 *
 * Wartosc "-" dla kluczy tekstowych kasuje klucz (wraca wartosc
 * wkompilowana, czyli serwer i login robot1 z flora_board.h).
 *
 * Zly adres albo haslo nie odcinaja robota na dobre: przy nieudanym
 * polaczeniu bringup_net_connect() wraca do wartosci wkompilowanych.
 *
 * @return ESP_ERR_NOT_SUPPORTED dla nieznanego klucza,
 *         ESP_ERR_INVALID_ARG dla zlej wartosci
 */
esp_err_t hub_config_set(const char *key, const char *value);

/** Publikuje biezace ustawienia (retain, bez hasla): config (siec, gleba),
 *  config_drive (naped, PID), config_cal (pompka, INA226), config_nav
 *  (geometria, hub_params.h), config_tags/<id> (mapa palikow) oraz
 *  config_auto, config_guard, config_pilot, config_water, config_seek,
 *  config_energy (hub_autocfg.h). */
void hub_config_publish(void);
