/**
 * @file head_obstacles.h
 * @brief Zadanie wykrywania przeszkod: klatka -> FLORA_MSG_OBSTACLES.
 *
 * Caly algorytm siedzi w komponencie `flora_ground`, ktory nie zna ESP-IDF
 * i jest sprawdzony na komputerze na nagranych klatkach. To zadanie robi
 * tylko trzy rzeczy: pobiera klatke, wola detektor i wysyla wynik w dol
 * masztu. Dzieki temu zmiana algorytmu nie dotyka kodu zadaniowego, a
 * zmiana tempa czy priorytetu nie dotyka algorytmu.
 *
 * WYMAGA KAMERY W TRYBIE SUROWYM (RGB565). Sterownik przyjmuje jeden
 * format na uruchomienie, wiec w trybie podgladu JPEG to zadanie nie ma
 * czego przetwarzac i nie nalezy go uruchamiac.
 */
#pragma once

#include "esp_err.h"
#include "flora_link.h"

/**
 * @brief Startuje zadanie wykrywania przeszkod.
 *
 * @param down lacze w dol do bryly; NULL wylacza wysylke, zostaje sam log
 * @return ESP_ERR_NO_MEM gdy nie ma pamieci na tablice (szukamy w PSRAM),
 *         ESP_ERR_INVALID_STATE gdy kamera nie jest w trybie surowym
 */
esp_err_t head_obstacles_start(flora_link_handle_t down);
