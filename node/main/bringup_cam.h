/**
 * @file bringup_cam.h
 * @brief Test rozruchowy kamery: podglad obrazu w przegladarce na komputerze.
 *
 * Do czego to sluzy: po wpieciu modulu kamery trzeba zobaczyc GOLYM OKIEM,
 * czy tasma siedzi w gniezdzie, czy obiektyw jest ostry i co faktycznie
 * widzi szerokokatny obiektyw 160 stopni. Log z detektora znacznikow tego
 * nie powie - "brak znacznikow" wyglada identycznie przy rozostrzonym
 * obrazie, zasloniete soczewce i zlym pinoucie.
 *
 * Jak to dziala: glowica wstaje jako stacja WiFi i uruchamia serwer HTTP,
 * ktory nadaje strumien MJPEG. Na komputerze wystarczy przegladarka -
 * zadnego programu po stronie PC nie trzeba.
 *
 *   http://<ip-glowicy>/         strona z zywym obrazem
 *   http://<ip-glowicy>/jpg      jedna klatka (do curl > plik.jpg)
 *
 * ================== WAZNE ==================
 * To test rozruchowy, nie element docelowej petli. Kamera jest tu
 * inicjalizowana w trybie JPEG i NIE MOZE rownolegle pracowac przez
 * flora_camera (odcienie szarosci do wykrywania znacznikow) - sterownik
 * esp32-camera przyjmuje tylko jedna konfiguracje na uklad. Wybierz jedno:
 * albo podglad, albo detekcja.
 * ===========================================
 */
#pragma once

#include <stdbool.h>

#include "esp_err.h"
#include "flora_camera.h"

/**
 * @brief Laczy z WiFi, startuje kamere w trybie JPEG i serwer podgladu.
 *
 * Dane dostepowe WiFi: z NVS (przestrzen "net", klucze jak w bryle),
 * domyslne z flora_board.h. Blokuje do 20 s na adres IP.
 *
 * Adres do wpisania w przegladarce lezy w logu po polaczeniu.
 *
 * @return ESP_OK gdy serwer nadaje; blad gdy nie ma sieci albo kamery -
 *         reszta glowicy dziala dalej, podglad jest dodatkiem
 */
esp_err_t bringup_cam_preview(void);

/** Czy glowica wstala w trybie surowym, czyli nadaje sie do wykrywania przeszkod. */
bool bringup_cam_is_raw_mode(void);

/**
 * @brief Pobiera klatke i ZATRZYMUJE dostep do kamery.
 *
 * Kamera jest zasobem dzielonym miedzy uchwytami HTTP a zadaniem wizji, a
 * do tego bywa uspiona dla chlodzenia - stad jeden wlasciciel i jedna
 * blokada zamiast wolania flora_camera_grab() z dwoch miejsc. Po powrocie
 * ESP_OK KONIECZNIE oddaj przez bringup_cam_frame_end().
 */
esp_err_t bringup_cam_frame_begin(flora_camera_frame_t *out);
void      bringup_cam_frame_end(flora_camera_frame_t *fr);
