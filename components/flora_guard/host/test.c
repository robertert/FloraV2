/*
 * Sprawdzenie flora_guard na komputerze.
 *
 *   cc -O2 -Wall -I../include -o test test.c ../flora_guard.c -lm
 *   ./test            # kod wyjscia 0 = OK
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "flora_guard.h"

static int s_fail;

static void check(int ok, const char *what)
{
    printf("%s  %s\n", ok ? "OK  " : "FAIL", what);
    if (!ok) {
        s_fail++;
    }
}

/** Wejscie "wszystko dobrze": swieze dane, czysto, plasko, pelna bateria. */
static flora_guard_in_t good(uint32_t now)
{
    flora_guard_in_t in = {
        .now_ms = now,
        .att_valid = true, .att_age_ms = 50,
        .obs_valid = true, .obs_age_ms = 100,
        .bus_v_valid = true, .bus_v = 13.2f,
    };
    memset(in.sectors, FLORA_GUARD_SECTOR_CLEAR, sizeof in.sectors);
    for (int i = 0; i < FLORA_GUARD_SONARS; i++) {
        in.sonar_mm[i] = NAN;
    }
    return in;
}

int main(void)
{
    flora_guard_cfg_t cfg = FLORA_GUARD_DEFAULT_CFG();
    flora_guard_t g;
    flora_guard_out_t o;
    flora_guard_in_t in;

    /* --- stan czysty --- */
    flora_guard_init(&g, &cfg);
    in = good(1000);
    o = flora_guard_update(&g, &in);
    check(o.allow_fwd && o.allow_rev && o.allow_turn && o.autonomy_ok && o.reasons == 0,
          "czysto: wszystko dozwolone");
    check(flora_guard_permit(&o, 30, 30) == 1.0f, "czysto: pelna moc w przod");

    /* --- kamera: przeszkoda na wprost --- */
    in = good(2000);
    in.sectors[7] = 25;                      /* 25 cm, 5 st w lewo od osi */
    o = flora_guard_update(&g, &in);
    check(!o.allow_fwd && o.allow_rev && o.allow_turn, "przeszkoda z przodu: tylko przod stoi");
    check(o.reasons == FLORA_GUARD_CAM, "przeszkoda z przodu: powod CAM");
    check(fabsf(o.front_mm - 249.0f) < 2.0f, "przeszkoda z przodu: odleglosc ~249 mm");
    check(flora_guard_permit(&o, 30, 30) == 0.0f, "przeszkoda z przodu: przod odrzucony");
    check(flora_guard_permit(&o, -30, -30) == 1.0f, "przeszkoda z przodu: tyl dozwolony");
    check(flora_guard_permit(&o, -30, 30) == 1.0f, "przeszkoda z przodu: obrot dozwolony");
    check(flora_guard_permit(&o, 0, 0) == 1.0f, "zero zawsze dozwolone");

    /* --- podtrzymanie blokady po zniknieciu przeszkody --- */
    in = good(2200);
    o = flora_guard_update(&g, &in);
    check(!o.allow_fwd, "200 ms po przeszkodzie: nadal stop");
    in = good(2600);
    o = flora_guard_update(&g, &in);
    check(o.allow_fwd, "600 ms po przeszkodzie: wolne");

    /* --- kamera: przeszkoda z boku poza korytarzem --- */
    flora_guard_init(&g, &cfg);
    in = good(1000);
    in.sectors[0] = 30;                      /* 30 cm, 75 st w lewo: 290 mm w bok */
    o = flora_guard_update(&g, &in);
    check(o.allow_fwd, "przeszkoda z boku poza korytarzem: jedzie");

    /* --- kamera: daleka przeszkoda w korytarzu nie blokuje --- */
    in = good(1100);
    in.sectors[8] = 90;
    o = flora_guard_update(&g, &in);
    check(o.allow_fwd && fabsf(o.front_mm - 896.0f) < 3.0f, "daleka przeszkoda: jedzie, widzi ~896 mm");

    /* --- kamera: "nie wiem" na wprost i z boku --- */
    flora_guard_init(&g, &cfg);
    in = good(1000);
    in.sectors[8] = FLORA_GUARD_SECTOR_UNKNOWN;
    o = flora_guard_update(&g, &in);
    check(!o.allow_fwd, "nie wiem na wprost: stop");
    flora_guard_init(&g, &cfg);
    in = good(1000);
    in.sectors[0] = FLORA_GUARD_SECTOR_UNKNOWN;
    in.sectors[15] = FLORA_GUARD_SECTOR_UNKNOWN;
    o = flora_guard_update(&g, &in);
    check(o.allow_fwd, "nie wiem tylko na skrajach: jedzie");

    /* --- kamera: stara ramka albo brak --- */
    flora_guard_init(&g, &cfg);
    in = good(1000);
    in.obs_age_ms = 1500;
    o = flora_guard_update(&g, &in);
    check(!o.allow_fwd && o.allow_rev, "stara ramka przeszkod: przod stoi");
    flora_guard_init(&g, &cfg);
    in = good(1000);
    in.obs_valid = false;
    o = flora_guard_update(&g, &in);
    check(!o.allow_fwd, "brak ramki przeszkod: przod stoi");

    /* --- kamera wylaczona w masce: brak ramki nie przeszkadza --- */
    cfg.mask = FLORA_GUARD_ALL & ~FLORA_GUARD_CAM;
    flora_guard_init(&g, &cfg);
    in = good(1000);
    in.obs_valid = false;
    o = flora_guard_update(&g, &in);
    check(o.allow_fwd && o.reasons == 0, "CAM poza maska: brak ramki bez znaczenia");
    cfg.mask = FLORA_GUARD_ALL;

    /* --- sonary --- */
    flora_guard_init(&g, &cfg);
    in = good(1000);
    in.sonar_mm[FLORA_GUARD_SONAR_FRONT_R] = 280.0f;
    o = flora_guard_update(&g, &in);
    check(!o.allow_fwd && o.allow_rev && o.reasons == FLORA_GUARD_SONAR,
          "przedni sonar 280 mm: przod stoi");
    flora_guard_init(&g, &cfg);
    in = good(1000);
    in.sonar_mm[FLORA_GUARD_SONAR_REAR] = 250.0f;
    o = flora_guard_update(&g, &in);
    check(o.allow_fwd && !o.allow_rev && o.allow_turn, "tylny sonar 250 mm: tyl stoi");
    check(flora_guard_permit(&o, -20, -20) == 0.0f, "tylny sonar: cofanie odrzucone");
    flora_guard_init(&g, &cfg);
    in = good(1000);
    in.sonar_mm[FLORA_GUARD_SONAR_REAR] = 800.0f;
    o = flora_guard_update(&g, &in);
    check(o.allow_rev && o.rear_mm == 800.0f, "tylny sonar 800 mm: cofa");

    /* --- przechyl z histereza --- */
    flora_guard_init(&g, &cfg);
    in = good(1000);
    in.roll_deg = -23.0f;
    o = flora_guard_update(&g, &in);
    check(!o.allow_fwd && !o.allow_rev && !o.allow_turn && !o.autonomy_ok,
          "przechyl 23 st: wszystko stoi");
    in = good(2000);
    in.roll_deg = -17.0f;
    o = flora_guard_update(&g, &in);
    check(!o.allow_turn, "przechyl 17 st po 23: nadal stop (histereza)");
    in = good(3000);
    in.roll_deg = 5.0f;
    o = flora_guard_update(&g, &in);
    in = good(3600);
    in.roll_deg = 5.0f;
    o = flora_guard_update(&g, &in);
    check(o.allow_fwd && o.allow_turn && o.autonomy_ok, "przechyl 5 st: wolne po release_ms");

    /* --- napiecie: przysiad krotki i trwaly --- */
    flora_guard_init(&g, &cfg);
    in = good(1000);
    in.bus_v = 11.5f;
    o = flora_guard_update(&g, &in);
    check(o.allow_fwd, "napiecie 11,5 V przez chwile: jedzie");
    in = good(2000);
    in.bus_v = 12.5f;
    o = flora_guard_update(&g, &in);
    in = good(2500);
    in.bus_v = 11.5f;
    o = flora_guard_update(&g, &in);
    in = good(4000);
    in.bus_v = 11.5f;
    o = flora_guard_update(&g, &in);
    check(o.allow_fwd, "odliczanie startuje od nowa po powrocie napiecia");
    in = good(4600);
    in.bus_v = 11.5f;
    o = flora_guard_update(&g, &in);
    check(!o.allow_fwd && !o.allow_turn && (o.reasons & FLORA_GUARD_VOLTAGE),
          "napiecie 11,5 V przez 2 s: stop");
    in = good(6000);
    in.bus_v = 12.0f;
    o = flora_guard_update(&g, &in);
    check(!o.allow_fwd, "napiecie 12,0 V: nadal stop (histereza)");
    in = good(7000);
    in.bus_v = 12.4f;
    o = flora_guard_update(&g, &in);
    in = good(7600);
    in.bus_v = 12.4f;
    o = flora_guard_update(&g, &in);
    check(o.allow_fwd, "napiecie 12,4 V: wolne");

    /* --- stare ATTITUDE --- */
    flora_guard_init(&g, &cfg);
    in = good(1000);
    in.att_age_ms = 900;
    o = flora_guard_update(&g, &in);
    check(o.allow_fwd && !o.autonomy_ok && o.power_limit == 0.3f,
          "stare ATTITUDE: pelzanie, autonomia stoi");
    check(fabsf(flora_guard_permit(&o, 50, 50) - 0.3f) < 1e-6f, "stare ATTITUDE: mnoznik 0,3");

    /* --- zawijanie zegara --- */
    flora_guard_init(&g, &cfg);
    in = good(0xFFFFFF00u);
    in.sectors[8] = 10;
    o = flora_guard_update(&g, &in);
    in = good(0x00000050u);                  /* 336 ms pozniej, po zawinieciu */
    o = flora_guard_update(&g, &in);
    check(!o.allow_fwd, "zawijanie zegara: podtrzymanie trzyma");
    in = good(0x00000200u);
    o = flora_guard_update(&g, &in);
    check(o.allow_fwd, "zawijanie zegara: puszcza po release_ms");

    /* --- klasyfikacja ruchu --- */
    flora_guard_out_t f = { .allow_fwd = false, .allow_rev = true, .allow_turn = true,
                            .power_limit = 1.0f };
    check(flora_guard_permit(&f, 40, 20) == 0.0f, "luk w przod przy blokadzie przodu: odrzucony");
    check(flora_guard_permit(&f, 20, -22) == 1.0f, "obrot z drobnym postepem: dozwolony");
    check(flora_guard_permit(&f, -40, -20) == 1.0f, "luk w tyl: dozwolony");

    printf("WYNIK: %s\n", s_fail ? "BLEDY" : "OK");
    return s_fail ? 1 : 0;
}
