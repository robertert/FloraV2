/*
 * Sprawdzenie flora_energy na komputerze.
 *
 *   cc -O2 -Wall -I../include -o test test.c ../flora_energy.c -lm
 *   ./test            # kod wyjscia 0 = OK
 */
#include <math.h>
#include <stdio.h>

#include "flora_energy.h"

static int s_fail;

static void check(int ok, const char *what)
{
    printf("%s  %s\n", ok ? "OK  " : "FAIL", what);
    if (!ok) {
        s_fail++;
    }
}

static flora_energy_out_t run(flora_energy_t *e, float v, float seconds, bool load, bool night)
{
    flora_energy_out_t o = e->last;
    for (float t = 0.0f; t < seconds; t += 1.0f) {
        flora_energy_in_t in = { .dt_s = 1.0f, .v_valid = true, .bus_v = v,
                                 .load_high = load, .night_known = true, .night = night };
        o = flora_energy_update(e, &in);
    }
    return o;
}

int main(void)
{
    flora_energy_cfg_t cfg = FLORA_ENERGY_DEFAULT_CFG();
    flora_energy_t e;
    flora_energy_out_t o;

    flora_energy_init(&e, &cfg);
    o = run(&e, 13.2f, 5, false, false);
    check(o.mode == FLORA_ENERGY_NORMAL && o.allow_drive && o.allow_water && o.mast_on &&
          o.sleep_s == 0, "13,2 V w dzien: NORMAL, bez snu");
    o = run(&e, 13.2f, 5, false, true);
    check(o.sleep_s == cfg.sleep_night_s, "13,2 V w nocy: sen nocny");

    /* krotki przysiad nie zmienia trybu, dlugi tak */
    o = run(&e, 12.0f, 3, false, false);
    check(o.mode == FLORA_ENERGY_NORMAL, "3 s na 12,0 V: filtr trzyma NORMAL");
    o = run(&e, 13.2f, 120, false, false);
    o = run(&e, 12.4f, 300, false, false);
    check(o.mode == FLORA_ENERGY_SAVE && !o.allow_drive && o.allow_water && o.mast_on &&
          o.sleep_s == cfg.sleep_save_s, "12,4 V trwale: SAVE, bez jazdy, sen takze w dzien");

    /* histereza powrotu */
    o = run(&e, 12.75f, 300, false, false);
    check(o.mode == FLORA_ENERGY_SAVE, "12,75 V: jeszcze SAVE (histereza)");
    o = run(&e, 13.0f, 300, false, false);
    check(o.mode == FLORA_ENERGY_NORMAL, "13,0 V: NORMAL");

    /* SURVIVAL i wyjscie stopniami */
    o = run(&e, 12.0f, 300, false, false);
    check(o.mode == FLORA_ENERGY_SURVIVAL && !o.allow_drive && !o.allow_water && !o.mast_on &&
          o.sleep_s == cfg.sleep_survival_s, "12,0 V trwale: SURVIVAL, maszt off, dlugi sen");
    o = run(&e, 12.4f, 300, false, false);
    check(o.mode == FLORA_ENERGY_SURVIVAL, "12,4 V: jeszcze SURVIVAL");
    o = run(&e, 12.6f, 300, false, false);
    check(o.mode == FLORA_ENERGY_SAVE, "12,6 V: SAVE");
    o = run(&e, 13.1f, 300, false, false);
    check(o.mode == FLORA_ENERGY_NORMAL, "13,1 V: NORMAL");

    /* probki pod obciazeniem sa pomijane */
    o = run(&e, 11.5f, 600, true, false);
    check(o.mode == FLORA_ENERGY_NORMAL && fabsf(o.v_filt - 13.1f) < 0.05f,
          "11,5 V pod obciazeniem: pomijane");

    /* brak pomiaru */
    flora_energy_init(&e, &cfg);
    flora_energy_in_t in = { .dt_s = 1.0f, .v_valid = false };
    o = flora_energy_update(&e, &in);
    check(o.mode == FLORA_ENERGY_NORMAL && isnan(o.v_filt) && o.sleep_s == 0,
          "bez INA226: NORMAL, noc nieznana = bez snu");

    printf("WYNIK: %s\n", s_fail ? "BLEDY" : "OK");
    return s_fail ? 1 : 0;
}
