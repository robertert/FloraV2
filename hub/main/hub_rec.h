/**
 * @file hub_rec.h
 * @brief Nagrywanie przejazdu: surowe wejscia nawigacji na MQTT.
 *
 * cmd/rec on | off. Gdy wlaczone, hub_nav zapisuje w kazdym takcie wejscia
 * flora_navcore_step (enkodery, ATTITUDE, slonce, znaczniki), a rekorder
 * co 200 ms wysyla je paczka na rec/data (qos 0). Format linii
 * i numeracja wiadomosci: components/flora_nav/include/flora_navlog.h.
 *
 * Po stronie komputera:
 *   tools/rec/record.py   zapis do pliku (przez mosquitto_sub),
 *   components/flora_nav/host/replay.c   ten sam EKF na nagraniu.
 *
 * Ruch: ~50 linii E i 20 A na sekunde, ok. 2 kB/s. Pelny bufor (np. brak
 * brokera) gubi linie zamiast blokowac nawigacje - licznik "drops" na
 * tele/rec mowi, czy nagranie jest kompletne.
 */
#pragma once

#include <stdbool.h>
#include "esp_err.h"

esp_err_t hub_rec_init(void);

/** "on" | "off". */
esp_err_t hub_rec_cmd(const char *arg);

bool hub_rec_active(void);

/** Dopisuje linie (format z flora_navlog.h). Nie blokuje; bez nagrywania nic nie robi. */
void hub_rec_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

/** Naglowek parametrow i mapy palikow - przy starcie nagrania i po kazdej zmianie. */
void hub_rec_header(void);
