/**
 * @file hub_params.h
 * @brief Parametry geometrii i miejsca pracy: NVS zamiast stalych.
 *
 * Wartosci z flora_board.h sa domyslnymi, NVS ma pierwszenstwo. Zmiana
 * przez cmd/config/<klucz> dziala od razu (enkodery, EKF, kompas sloneczny),
 * bez flashowania - na trawie i z pelnym zbiornikiem kola maja inny
 * promien niz na podlodze, wiec to sie bedzie zmieniac.
 *
 * Klucze (cmd/config/<klucz>, "-" wraca do wkompilowanej):
 *   track_mm      rozstaw kol, 250-600; najlepiej z cmd/track_cal
 *   wheel_mm      srednia skuteczna srednica kol, 80-200; z cmd/wheel_cal
 *   wheel_ratio   srednica prawego / lewego, 0,9-1,1; z cmd/wheel_cal
 *   cam_x_mm, cam_y_mm   kamera znacznikow wzgledem srodka osi kol
 *   cam_yaw_deg   obrot osi kamery, dodatni w prawo, -45..45 (flora_board.h)
 *   cam_h_mm, tag_h_mm   wysokosc kamery i znacznikow, 0-2000 (odleglosc pozioma)
 *   site_lat, site_lon   miejsce pracy (kompas sloneczny)
 *   mag_decl      deklinacja magnetyczna, wschodnia dodatnia
 *
 * Wszystko publikowane (retain) na config_nav, z flagami *_set mowiacymi,
 * czy wartosc jest skalibrowana (NVS), czy jeszcze wkompilowana.
 */
#pragma once

#include <stdbool.h>
#include "esp_err.h"

typedef struct {
    float  track_mm;
    float  wheel_mm;
    float  wheel_ratio;
    float  cam_x_mm, cam_y_mm;
    float  cam_yaw_deg;      /**< dodawany do kata kazdego znacznika */
    float  cam_h_mm, tag_h_mm; /**< wysokosci nad ziemia, do odleglosci poziomej */
    double lat_deg, lon_deg;
    float  decl_deg;

    bool   track_set, wheel_set, cam_set, site_set;
} hub_params_t;

/** Czyta NVS. Wolac raz przy starcie, przed enkoderami i nawigacja. */
void hub_params_load(void);

/** Kopia biezacych parametrow (bezpieczna miedzy zadaniami). */
void hub_params_get(hub_params_t *out);

/** Droga na zliczenie lewego i prawego kola [mm] z wheel_mm i wheel_ratio. */
void hub_params_mm_per_count(const hub_params_t *p, float *left, float *right);

/**
 * Zapis jednego klucza z cmd/config.
 * @return ESP_ERR_NOT_SUPPORTED gdy klucz nie jest parametrem nawigacji
 */
esp_err_t hub_params_set(const char *key, const char *value);

/** Zapis wyniku kalibracji (bez parsowania) i natychmiastowe uzycie. */
esp_err_t hub_params_save_track(float track_mm);
esp_err_t hub_params_save_wheels(float wheel_mm, float wheel_ratio);

/** Publikuje config_nav (retain). */
void hub_params_publish(void);
