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
 *   cmd/drive_power  0..100, moc dla cmd/drive (domyslnie 30);
 *                    w trybie pid procent pid_vmax
 *   cmd/pid_step     "<mm_s> <ms>", skok predkosci do strojenia (tylko tryb pid)
 *   cmd/pose_reset   "<x_mm> <y_mm> <th_deg>" albo pusty = (0, 0, 0), patrz hub_nav.h
 *   cmd/mag_cal      "auto [s]" | "manual [s]", kalibracja magnetometru, hub_headcal.h
 *   cmd/head_gyro_cal dowolny, kalibracja offsetu zyro glowicy (robot nieruchomo)
 *   cmd/level_cal    "auto" | "flat" | "1" | "2", poziom akcelerometru glowicy
 *   cmd/track_cal    "[obroty]", rozstaw kol z obrotu w miejscu, hub_headcal.h
 *   cmd/wheel_cal    "go [mm]" potem "<mm z tasmy>", srednica i roznica kol
 *   cmd/tag          "<id> <x_mm> <y_mm>" | "<id> -" | "clear", mapa palikow, hub_nav.h
 *   cmd/rec          on | off, nagrywanie przejazdu na rec/data, hub_rec.h
 *   cmd/pump_cal     "<ml>" zmierzone po ostatniej dawce, patrz hub_sensorcal.h
 *   cmd/ina_cal      "<A>" z multimetru, bocznik INA226 (po restarcie)
 *   cmd/pump         on | off | liczba ms | "ml <objetosc>" (po pump_cal)
 *   cmd/mast         on | off
 *   cmd/sonar        a | b | c | all, test dalmierzy po kolei (tylko wybrany, szybciej)
 *   cmd/stall_clear  dowolny
 *   cmd/reboot       dowolny, restart po 0,5 s
 *   cmd/config/<k>   zapis ustawienia do NVS, patrz hub_config.h
 *   cmd/ota          "latest" albo "hub-vX.Y.Z" [+ " force"], patrz hub_ota.h
 */
esp_err_t hub_remote_start(void);
