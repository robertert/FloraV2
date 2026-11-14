/**
 * @file hub_ota.c
 * @brief Aktualizacja firmware bryly przez siec - nakladka.
 *
 * Cala logika jest w components/flora_ota (wspolna z glowica). Tu zostaje
 * tylko to, co dotyczy sprzetu bryly: aktualizacja nie rusza w trakcie jazdy
 * ani podlewania, a przed pobieraniem zatrzymuje naped i pompke.
 */
#include "hub_ota.h"
#include "flora_motors.h"
#include "flora_ota.h"
#include "flora_pump.h"

static bool may_start(void)
{
    float l = 0, r = 0;
    flora_motors_get_last(&l, &r);
    return l == 0.0f && r == 0.0f && !flora_pump_is_running();
}

static void make_safe(void)
{
    flora_motors_stop();
    flora_pump_stop();
}

void hub_ota_check_pending(void)
{
    static const flora_ota_cfg_t cfg = {
        .version_prefix = "hub-v",
        .dir            = "hub",
        .topic          = "tele/ota",
        .may_start      = may_start,
        .make_safe      = make_safe,
    };
    flora_ota_init(&cfg);
}

esp_err_t hub_ota_start(const char *arg) { return flora_ota_start(arg); }

bool hub_ota_in_progress(void) { return flora_ota_in_progress(); }
