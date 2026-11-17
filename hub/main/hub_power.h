/**
 * @file hub_power.h
 * @brief Energia bryly: napiecie baterii, tryby pracy, klucz sterownika
 *        silnikow i gleboki sen.
 *
 * Trzy rzeczy, kazda za wlasnym przelacznikiem (hub_autocfg.h), wszystkie
 * DOMYSLNIE WYLACZONE:
 *
 * 1. POLITYKA ENERGII (flora_energy) - liczona, gdy wlaczona jest
 *    ktorakolwiek z funkcji: straznik, misja, podlewanie albo sen. Daje
 *    tryb NORMAL / SAVE / SURVIVAL na tele/energy. Misja slucha zakazu
 *    jazdy, podlewanie zakazu podlewania.
 *
 * 2. SEN (sleep_en). W trybach i porach, w ktorych polityka kaze spac,
 *    bryla wylacza maszt i zasypia gleboko na zadany czas. Zasypia tylko,
 *    gdy nic sie nie dzieje: naped i pompka stoja, misja pozwala, nie ma
 *    OTA (ani wersji czekajacej na potwierdzenie), kalibracji ani
 *    nagrywania, od startu minelo AWAKE_MIN_S, a od ostatniej komendy
 *    z brokera AWAKE_AFTER_CMD_S.
 *    Wybudzenie to zwykly start: poza wraca z NVS (hub_nav), liczniki
 *    misji startuja od nowa.
 *    JAK WYLACZYC SPIACEGO ROBOTA: `mosquitto_pub -r -t
 *    flora/robot1/cmd/config/sleep_en -m 0` (retain) - robot odbierze to
 *    w oknie po najblizszym wybudzeniu. Potem skasowac retain pusta
 *    wiadomoscia.
 *    W trybie SURVIVAL maszt jest wylaczony takze na jawie.
 *
 * 3. KLUCZ MDD10A (mdd_key_en). Sterownik silnikow bierze ok. 50 mA
 *    w spoczynku, wiec stoi za kluczem 12 V (D-022). Przed jazda klucz
 *    jest zalaczany, po KEY_IDLE_S bez ruchu wylaczany (z pinami DIR
 *    w stanie niskim). Pierwsza komenda jazdy po przerwie czeka ok.
 *    100 ms na rozruch sterownika.
 *    Bez zamontowanego modulu klucza zostawic 0 - GPIO42 nie jest wtedy
 *    w ogole dotykany.
 */
#pragma once

#include <stdbool.h>
#include "esp_err.h"
#include "flora_energy.h"

/** Minimalny czas na jawie po starcie, zanim wolno zasnac. */
#define HUB_POWER_AWAKE_MIN_S        90
/** Tyle od ostatniej komendy z brokera robot nie zasypia. */
#define HUB_POWER_AWAKE_AFTER_CMD_S 120

esp_err_t hub_power_start(void);

/** Krok co 1 s (z zadania hub_auto). */
void hub_power_tick(void);

/** Ostatnie zmierzone napiecie szyny. false, gdy brak swiezego pomiaru. */
bool hub_power_bus_v(float *v);

/** Ostatni wynik polityki energii. */
void hub_power_get(flora_energy_out_t *out);

/**
 * Pozycja slonca dla miejsca pracy i biezacego czasu.
 * @return false bez czasu UTC (brak SNTP)
 */
bool hub_power_sun(float *elev_deg, float *azimuth_deg);

/**
 * Wolane przed kazdym zadaniem mocy na silniki (z zadania, nie z timera):
 * przy mdd_key_en zalacza klucz i czeka na rozruch sterownika. Bez
 * przelacznika nic nie robi.
 */
void hub_power_drive_wake(void);

/** Przyszla komenda z brokera - ktos jest przy robocie, nie zasypiac. */
void hub_power_note_activity(void);
