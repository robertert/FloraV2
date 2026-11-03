/**
 * @file hub_remote.h
 * @brief Proste sterowanie bryla przez MQTT (tematy pod cmd/).
 */
#pragma once

#include "esp_err.h"

/**
 * @brief Subskrybuje cmd/# i uruchamia straznika napedu (dead-man).
 *
 * Wymaga flora_mqtt_init() i hub_telemetry_start() - steruje sprzetem,
 * ktory zainicjalizowala telemetria.
 *
 * Komendy (payload tekstowy), odpowiedz na ack/<nazwa>: "ok" albo blad:
 *   cmd/drive        fwd | back | left | right | stop
 *   cmd/drive_power  0..100, moc dla cmd/drive (domyslnie 30)
 *   cmd/pump         on | off | liczba ms
 *   cmd/mast         on | off
 *   cmd/stall_clear  dowolny
 *   cmd/reboot       dowolny, restart po 0,5 s
 *   cmd/config/<k>   zapis ustawienia do NVS, patrz hub_config.h
 */
esp_err_t hub_remote_start(void);
