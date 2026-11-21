/**
 * @file head_tags.h
 * @brief Zadanie wykrywania znacznikow AprilTag na glowicy.
 *
 * Od 21.11.2026 (wczesniej modul tagcam). Kamera glowicy z obiektywem
 * 68 stopni patrzy poziomo na paliki; detektor flora_tags daje
 * identyfikator, odleglosc, azymut i pewnosc, a zadanie wysyla
 * FLORA_MSG_TAGS w dol do bryly. Pozycje liczy EKF w bryle.
 *
 * WYMAGA KAMERY W TRYBIE SUROWYM (RGB565, bringup_cam_offline albo podglad
 * w trybie raw). Klatka jest zamieniana na odcienie szarosci tutaj, zeby
 * podglad /raw i zbieranie klatek dzialaly jak dotad.
 */
#pragma once

#include "esp_err.h"
#include "flora_link.h"

/**
 * @param down lacze w dol do bryly; NULL wylacza wysylke, zostaje sam log
 * @return ESP_ERR_INVALID_STATE gdy kamera nie jest w trybie surowym,
 *         ESP_ERR_NO_MEM gdy brak PSRAM na bufor
 */
esp_err_t head_tags_start(flora_link_handle_t down);
