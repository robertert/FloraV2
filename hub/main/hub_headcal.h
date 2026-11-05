/**
 * @file hub_headcal.h
 * @brief Kalibracje glowicy wywolywane z bryly: magnetometr i zyroskop.
 *
 * Magnetometr (hard + soft iron) wymaga 2-3 pelnych obrotow wokol pionu.
 * Dwa tryby:
 *   auto    bryla sama obraca robota w miejscu (~20 st/s) przez zadany czas;
 *           tryb pid napedu - zadana predkosc kol, tryb pwm - mala stala moc.
 *           Straznik utyku dziala jak zawsze; utyk przerywa obrot.
 *   manual  bez jazdy - glowice/robota obraca sie recznie (stol, platforma).
 * Wynik przychodzi z glowicy jako MAG_CAL_RESULT i idzie na head/mag_cal.
 *
 * UWAGA na miejsce kalibracji: z dala od samochodow, zbrojonego betonu,
 * metalowych mebli. Kalibracja ma usunac pole SAMEGO robota - pole otoczenia
 * zapisaloby sie jako jego czesc.
 */
#pragma once

#include <stdbool.h>
#include "esp_err.h"

/**
 * @param arg  "auto [s]" albo "manual [s]"; domyslnie auto 40 s / manual 60 s,
 *             zakres 10-120 s
 * @return ESP_ERR_INVALID_STATE gdy kalibracja juz trwa albo glowica nie odpowiada
 */
esp_err_t hub_headcal_mag(const char *arg);

/**
 * Kalibracja poziomu akcelerometru glowicy (przekrzywienie plytki na maszcie).
 * Robic PRZED kalibracja magnetometru - poziom wchodzi w kompensacje przechylu.
 *
 * @param arg  "auto"  - pomiar, obrot o 180 st wg kursu z glowicy, drugi pomiar;
 *                       srednia oddziela przekrzywienie plytki od pochylosci podloza,
 *             "flat"  - jeden pomiar, podloze uznane za plaskie (np. poziomica),
 *             "1"/"2" - kroki reczne: pomiar, obrot recznie o 180 st, pomiar
 */
esp_err_t hub_headcal_level(const char *arg);

/** Kalibracja offsetu zyroskopu glowicy. Robot musi stac nieruchomo ~2 s. */
esp_err_t hub_headcal_gyro(void);

bool hub_headcal_running(void);
