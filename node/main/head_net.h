/**
 * @file head_net.h
 * @brief Siec glowicy: wlasne WiFi i MQTT, niezalezne od UART-u w maszcie.
 *
 * Daje to samo co w bryle: logi (head/log), info o starcie (head/boot),
 * komendy serwisowe (headcmd/...) i aktualizacje OTA (head/ota). Dzieki temu
 * glowice da sie debugowac bez kabla USB i takze wtedy, gdy lacze z bryla
 * nie dziala.
 */
#pragma once

#include "esp_err.h"

/**
 * Wolac zaraz po NVS: podpina przechwytywanie logow (linie z rozruchu
 * czekaja w kolejce na polaczenie) i odliczanie do potwierdzenia nowej
 * wersji po OTA. Nie dotyka radia.
 */
void head_net_early(void);

/**
 * Laczy z WiFi i brokerem w osobnym zadaniu - nie blokuje, a brak sieci nie
 * zatrzymuje czujnikow ani lacza z bryla. Nic nie robi przy
 * FLORA_HEAD_NET_DEFAULT = 0.
 */
esp_err_t head_net_start(void);
