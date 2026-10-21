/**
 * @file bringup_link.h
 * @brief Test toru komunikacyjnego na petli zwrotnej.
 */
#pragma once

/**
 * @brief Sprawdza ramkowanie, potwierdzanie i statystyki bez drugiej plytki.
 *
 * Zajmuje i zwalnia UART1, wiec po powrocie port jest wolny.
 * Blokuje na okolo 1 s.
 */
void bringup_link_loopback(void);
