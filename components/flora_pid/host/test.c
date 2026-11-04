/*
 * Sprawdzenie flora_pid na komputerze, na symulowanym silniku.
 *
 * Model: silnik DC pierwszego rzedu z martwa strefa (tarcie statyczne).
 *   ponizej DEADZONE % mocy kolo stoi,
 *   powyzej predkosc ustalona = K * (moc - DEADZONE), stala czasowa TAU.
 * Liczby ustawione z grubsza pod naped robota (40% mocy ~ 86 mm/s,
 * zmierzone 27.10.2026), ale to nie jest model do strojenia - tylko
 * dowod, ze regulator zbiega i nie nabiera calki przy nasyceniu.
 *
 *   cc -O2 -Wall -I../include -o test test.c ../flora_pid.c -lm
 *   ./test            # sprawdzenia, kod wyjscia 0 = OK
 *   ./test --csv      # przebieg t,zadana,pomiar,moc do wykresu
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "flora_pid.h"

#define DT        0.02f
#define K         3.4f    /* mm/s na % mocy ponad martwa strefa */
#define DEADZONE 15.0f
#define TAU       0.15f

static float plant(float v, float u)
{
    float a = fabsf(u) - DEADZONE;
    float target = a > 0.0f ? (u > 0.0f ? K * a : -K * a) : 0.0f;
    return v + (target - v) * (DT / TAU);
}

static int s_fail;

static void check(int ok, const char *what)
{
    printf("%s  %s\n", ok ? "OK  " : "FAIL", what);
    if (!ok) {
        s_fail++;
    }
}

/** Przebieg skoku; zwraca predkosc koncowa. */
static float run_step(flora_pid_t *pid, float sp, float seconds, int csv, float *max_v)
{
    float v = 0.0f;
    *max_v = 0.0f;
    int n = (int)(seconds / DT);
    for (int i = 0; i < n; i++) {
        float u = flora_pid_update(pid, sp, v, DT);
        v = plant(v, u);
        if (v > *max_v) {
            *max_v = v;
        }
        if (csv) {
            printf("%.2f,%.1f,%.1f,%.1f\n", i * DT, sp, v, u);
        }
    }
    return v;
}

int main(int argc, char **argv)
{
    int csv = argc > 1 && strcmp(argv[1], "--csv") == 0;

    flora_pid_cfg_t cfg = FLORA_PID_DEFAULT_CFG();
    cfg.kff = 1.0f / K;
    cfg.ks  = DEADZONE;
    cfg.kp  = 0.15f;
    cfg.ki  = 1.0f;

    flora_pid_t pid;
    flora_pid_init(&pid, &cfg);

    if (csv) {
        printf("t,sp,v,u\n");
        float m;
        run_step(&pid, 150.0f, 3.0f, 1, &m);
        return 0;
    }

    float max_v;
    float v = run_step(&pid, 150.0f, 3.0f, 0, &max_v);
    check(fabsf(v - 150.0f) < 2.0f, "skok 150 mm/s zbiega do zadanej");
    check(max_v < 150.0f * 1.15f, "przeregulowanie ponizej 15%");

    /* Zadana nieosiagalna: wyjscie wisi na out_max, calka nie moze rosnac
     * bez konca - inaczej po zdjeciu przeszkody robot by wystrzelil. */
    flora_pid_reset(&pid);
    run_step(&pid, 1000.0f, 5.0f, 0, &max_v);
    check(pid.last.out == cfg.out_max, "nasycenie trzyma out_max");
    check(fabsf(pid.integ) <= cfg.i_max, "calka w limicie i_max");
    float integ_sat = pid.integ;
    run_step(&pid, 1000.0f, 5.0f, 0, &max_v);
    check(pid.integ == integ_sat, "calka nie rosnie przy nasyceniu (anti-windup)");

    /* Bez ff i bez I: sam P zostawia uchyb, ale stabilnie. */
    flora_pid_cfg_t p_only = FLORA_PID_DEFAULT_CFG();
    p_only.kp = 0.3f;
    flora_pid_init(&pid, &p_only);
    v = run_step(&pid, 100.0f, 3.0f, 0, &max_v);
    check(v > 0.0f && v < 100.0f, "sam P: ruch z uchybem ustalonym");

    /* Zmiana ki w locie nie skacze wyjsciem. */
    flora_pid_init(&pid, &cfg);
    run_step(&pid, 150.0f, 2.0f, 0, &max_v);
    float before = pid.last.out;
    flora_pid_cfg_t c2 = cfg;
    c2.ki = 5.0f;
    flora_pid_set_cfg(&pid, &c2);
    float vv = 150.0f;   /* w stanie ustalonym uchyb ~0 */
    float after = flora_pid_update(&pid, 150.0f, vv, DT);
    check(fabsf(after - before) < 1.0f, "zmiana ki bez skoku wyjscia");

    printf("%s\n", s_fail ? "WYNIK: BLEDY" : "WYNIK: OK");
    return s_fail ? 1 : 0;
}
