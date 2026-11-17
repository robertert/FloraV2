/**
 * @file hub_autocfg.h
 * @brief Przelaczniki i progi funkcji autonomicznych, w NVS.
 *
 * WSZYSTKO DOMYSLNIE WYLACZONE. Kazda funkcja ma wlasny przelacznik
 * i wlacza sie osobno, bez flashowania:
 *
 *     mosquitto_pub -t flora/robot1/cmd/config/guard_en -m 1
 *
 * Przy wszystkich przelacznikach na 0 robot zachowuje sie tak jak przed
 * dodaniem tych modulow. Zmiana dziala od razu ("-" wraca do domyslnej).
 *
 * PRZELACZNIKI (0 | 1), publikowane na config_auto:
 *   guard_en     straznik napedu (hub_safety.h); ktore straze - guard_mask
 *   pilot_en     jazda do punktu, cmd/goto i cmd/turn (hub_pilot.h)
 *   water_en     automatyczne podlewanie (hub_water.h)
 *   mission_en   misja: szukanie slonca (hub_mission.h); wymaga pilot_en,
 *                trybu pid, obszaru (cmd/area) i najlepiej guard_en
 *   sleep_en     gleboki sen i wylaczanie masztu wg napiecia (hub_power.h)
 *   mdd_key_en   klucz 12 V sterownika MDD10A na GPIO42 (hub_power.h);
 *                wlaczyc dopiero po zamontowaniu modulu klucza
 *
 * STRAZNIK, config_guard:
 *   guard_mask   suma bitow: 1 kamera, 2 sonary, 4 przechyl, 8 napiecie,
 *                16 stare dane z glowicy (domyslnie 31 = wszystko)
 *   g_front_mm   stop przed przeszkoda z przodu
 *   g_rear_mm    stop przed przeszkoda z tylu
 *   g_tilt_deg   stop przy przechyle
 *   g_v_stop     stop jazdy ponizej tego napiecia [V]
 *
 * PILOT, config_pilot:
 *   p_vmax       predkosc jazdy [mm/s] (nie wiecej niz pid_vmax)
 *   p_wmax       predkosc obrotu [st/s]
 *   p_tol_mm     tolerancja dojazdu
 *
 * PODLEWANIE, config_water:
 *   w_dry        ponizej tej wilgotnosci [%] podlewa
 *   w_wet        powyzej tej przestaje (musi byc > w_dry)
 *   w_dose_ml    jedna dawka
 *   w_soak_s     czas wsiakania miedzy dawkami
 *   w_day_ml     limit na dobe
 *
 * SZUKANIE SLONCA, config_seek (obszar osobno: cmd/area):
 *   s_len_mm     dlugosc probnego odcinka
 *   s_ratio      cien = jasnosc ponizej tej czesci niedawnego maksimum
 *   s_hold_s     tyle musi trwac cien
 *   s_dwell_s    najkrotszy postoj miedzy przejazdami
 *
 * ENERGIA, config_energy:
 *   e_v_save     ponizej: tryb oszczedny, bez jazdy [V]
 *   e_v_surv     ponizej: przetrwanie, maszt wylaczony [V] (< e_v_save)
 *   e_night_s    dlugosc snu w nocy [s]; 0 = bez snu nocnego
 *   e_day_s      dlugosc snu w dzien w trybie normalnym; 0 = bez snu
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef enum {
    HUB_AC_GUARD_EN,
    HUB_AC_PILOT_EN,
    HUB_AC_WATER_EN,
    HUB_AC_MISSION_EN,
    HUB_AC_SLEEP_EN,
    HUB_AC_MDD_KEY_EN,

    HUB_AC_GUARD_MASK,
    HUB_AC_G_FRONT_MM,
    HUB_AC_G_REAR_MM,
    HUB_AC_G_TILT_DEG,
    HUB_AC_G_V_STOP,

    HUB_AC_P_VMAX,
    HUB_AC_P_WMAX,
    HUB_AC_P_TOL_MM,

    HUB_AC_W_DRY,
    HUB_AC_W_WET,
    HUB_AC_W_DOSE_ML,
    HUB_AC_W_SOAK_S,
    HUB_AC_W_DAY_ML,

    HUB_AC_S_LEN_MM,
    HUB_AC_S_RATIO,
    HUB_AC_S_HOLD_S,
    HUB_AC_S_DWELL_S,

    HUB_AC_E_V_SAVE,
    HUB_AC_E_V_SURV,
    HUB_AC_E_NIGHT_S,
    HUB_AC_E_DAY_S,

    HUB_AC_COUNT
} hub_ac_key_t;

/** Obszar roboczy w ukladzie mapy [mm]. */
typedef struct {
    bool  valid;
    float xmin, ymin, xmax, ymax;
} hub_area_t;

/** Czyta NVS. Wolac raz, zaraz po flora_store_init(). */
void hub_autocfg_load(void);

/** Biezaca wartosc; bezpieczne z kazdego zadania i z timera. */
float hub_autocfg_get(hub_ac_key_t key);

/** Przelacznik wlaczony. */
bool hub_autocfg_on(hub_ac_key_t key);

/** Rosnie przy kazdej zmianie - modul porownuje i przeladowuje swoje cfg. */
uint32_t hub_autocfg_generation(void);

/**
 * Zapis jednego klucza z cmd/config.
 * @return ESP_ERR_NOT_SUPPORTED gdy klucz nie nalezy do tego modulu
 */
esp_err_t hub_autocfg_set(const char *key, const char *value);

/** cmd/area: "xmin ymin xmax ymax" w mm albo "-" (kasuje). Boki min. 1 m. */
esp_err_t hub_autocfg_area_cmd(const char *arg);

void hub_autocfg_area(hub_area_t *out);

/** Publikuje config_auto, config_guard, config_pilot, config_water,
 *  config_seek i config_energy (retain). */
void hub_autocfg_publish(void);
