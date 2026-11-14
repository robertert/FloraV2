/**
 * @file hub_diag.c
 * @brief Logi przez MQTT i informacja o starcie - nakladka bryly.
 *
 * Cala logika jest w components/flora_diag (wspolna z glowica). Bryla
 * publikuje na tele/log i tele/boot.
 */
#include "hub_diag.h"
#include "flora_diag.h"

esp_err_t hub_diag_init(void)
{
    const flora_diag_cfg_t cfg = { .log_topic = "tele/log", .boot_topic = "tele/boot" };
    return flora_diag_init(&cfg);
}

esp_err_t hub_diag_set_level(const char *arg) { return flora_diag_set_level(arg); }

void hub_diag_publish_boot(void) { flora_diag_publish_boot(); }
