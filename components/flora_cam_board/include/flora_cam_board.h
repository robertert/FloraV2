/**
 * @file flora_cam_board.h
 * @brief Mapa pinow modulu kamery znacznikow (trzeci modul, w maszcie).
 *
 * Modul ma JEDNO zadanie: szukac znacznikow AprilTag na palikach w rogach
 * obszaru roboczego i wysylac surowe detekcje w dol. Nie ma czujnikow
 * I2C ani nawigacji - te sa w glowicy.
 *
 * TOPOLOGIA: w maszcie biegnie tylko JEDEN kabel w dol, wiec ten modul nie
 * ma wlasnego lacza z bryla. Gada z glowica, a ta przekazuje jego ramki
 * dalej (patrz flora_link_relay).
 *
 *   MODUL ZNACZNIKOW  --UART--  GLOWICA  --UART--  BRYLA
 *
 * ================== DO POTWIERDZENIA ==================
 * Numery pinow sa WSTEPNE. Na plytkach z gniazdem kamery wiekszosc
 * wyprowadzen zajmuje interfejs DVP, wiec wolne piny zaleza od modelu.
 * Sprawdz dokumentacje swojej plytki przed uruchomieniem.
 * ======================================================
 */
#pragma once

#include "driver/gpio.h"
#include "driver/uart.h"

/* ---- UART w dol (do glowicy) ---- */
#define FLORA_CAM_UART_TX    GPIO_NUM_38   /**< TODO potwierdzic */
#define FLORA_CAM_UART_RX    GPIO_NUM_39   /**< TODO potwierdzic */
#define FLORA_CAM_UART_PORT  UART_NUM_1

/** Krotki odcinek w maszcie - mozna szybciej niz lacze w dol. */
#define FLORA_CAM_BAUD       115200

/* ---- Parametry wyszukiwania znacznikow ----
 * Do ustalenia po wybraniu rozmiaru obszaru roboczego: od niego zalezy
 * fizyczny rozmiar znacznikow na palikach, a od tego zasieg wykrywania.
 */
#define FLORA_CAM_TAG_SIZE_MM   150   /**< TODO: bok znacznika AprilTag */
#define FLORA_CAM_MAX_RANGE_MM  8000  /**< TODO: potwierdzic pomiarem */
