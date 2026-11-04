/**
 * @file hub_drive.h
 * @brief Petla predkosci kol: PID na kolo, zadana w mm/s.
 *
 * Dwa tryby:
 *   pwm - jak dotad, cmd/drive zadaje surowa moc, petla nic nie wystawia,
 *   pid - cmd/drive zadaje predkosc, petla co 20 ms wystawia moc.
 * Tryb i nastawy w NVS, zmieniane przez cmd/config/<klucz> i dzialaja od razu:
 *   drive_mode  pwm | pid
 *   pid_kp pid_ki pid_kd pid_kff pid_ks   nastawy (moc w %, predkosc w mm/s)
 *   pid_acc     limit przyspieszenia zadanej, mm/s^2 (0 = bez rampy)
 *   pid_vmax    mm/s odpowiadajace suwakowi mocy 100% w panelu
 *
 * Straznik utyku w flora_motors nadal pilnuje: po utyku flora_motors_set
 * odrzuca moc, a petla zeruje zadane i nie ponawia.
 *
 * STROJENIE (kola w powietrzu, potem na ziemi), wykres na tele/pid:
 *   1. cmd/config/drive_mode pid, nastawy na 0.
 *   2. ks: najmniejsza moc, przy ktorej kolo rusza (straznik ma min_power 15%).
 *   3. kff: cmd/pid_step "150 3000"; dobierz tak, by sam ff dawal ~zadana.
 *   4. kp w gore do lekkiego przeregulowania, potem ki usuwa uchyb.
 *      kd zwykle zostaje 0 - pomiar z enkodera przy 50 Hz jest szarpany.
 *   5. Powtorz na trawie, z pelnym zbiornikiem.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef enum {
    HUB_DRIVE_MODE_PWM,
    HUB_DRIVE_MODE_PID,
} hub_drive_mode_t;

typedef struct {
    hub_drive_mode_t mode;
    bool  active;        /**< petla wystawia moc (zadana != 0 albo hamuje) */
    float sp_l, sp_r;    /**< zadana po rampie, mm/s */
    float v_l, v_r;      /**< zmierzona, mm/s */
    float u_l, u_r;      /**< wyjscie, % mocy */
    float ff_l, ff_r;    /**< czlon feedforward */
    float i_l, i_r;      /**< czlon calkujacy */
} hub_drive_state_t;

/** Laduje nastawy z NVS i uruchamia timer. Wymaga zainicjalizowanych silnikow i enkoderow. */
esp_err_t hub_drive_start(void);

/** false, gdy naped nie wystartowal - wtedy reszta API zwraca ESP_ERR_INVALID_STATE. */
bool hub_drive_ready(void);

/** Zadana predkosc kol (mm/s), przycinana do +-pid_vmax. Tylko w trybie pid. */
esp_err_t hub_drive_set_speed(float left_mm_s, float right_mm_s);

/**
 * Natychmiastowe zatrzymanie, bez rampy, w kazdym trybie.
 * Zatrzymanie nigdy nie moze byc odrzucone - dlatego dziala zawsze.
 */
esp_err_t hub_drive_stop(void);

/** Skok predkosci obu kol na `ms`, potem stop. Do strojenia. */
esp_err_t hub_drive_step(float mm_s, uint32_t ms);

esp_err_t        hub_drive_set_mode(hub_drive_mode_t mode);
hub_drive_mode_t hub_drive_mode(void);

/** Ponownie czyta nastawy z NVS. Wolane po cmd/config/pid_*. */
void hub_drive_reload(void);

/** Biezace pid_vmax, mm/s. */
float hub_drive_vmax(void);

void hub_drive_get_state(hub_drive_state_t *out);

/** Krok testu skoku nie dluzszy niz to. */
#define HUB_DRIVE_STEP_MAX_MS 5000
