#!/bin/sh
# Wszystkie testy hostowe naraz - lokalnie i w CI (.github/workflows/tests.yml).
#
#   sh tools/host_tests.sh          # kod wyjscia 0 = wszystko OK
#   SAN=0 sh tools/host_tests.sh    # bez sanitizerow (szybciej, np. stary cc)
#
# Domyslnie z AddressSanitizer i UBSan: lapia wyjscie poza tablice, uzycie
# zwolnionej pamieci i niezdefiniowane zachowanie, ktorych sam wynik testu
# nie pokaze. NIE lapia przepelnienia stosu zadania na ESP - na komputerze
# stos ma megabajty (patrz README, wykrywanie przeszkod).
set -eu
cd "$(dirname "$0")/../components"

CC=${CC:-cc}
if [ "${SAN:-1}" = 1 ]; then
    CFLAGS="-O1 -g -Wall -fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover=undefined"
else
    CFLAGS="-O2 -Wall"
fi
OUT=$(mktemp -d)
trap 'rm -rf "$OUT"' EXIT
fail=0

run() {
    name=$1; shift
    printf '== %s\n' "$name"
    if "$@" >"$OUT/$name.log" 2>&1; then
        grep -E '^(FAIL|WYNIK)' "$OUT/$name.log" || true
    else
        cat "$OUT/$name.log"
        echo "!! $name: BLAD"
        fail=1
    fi
}

# shellcheck disable=SC2086
$CC $CFLAGS -Iflora_pid/include -o "$OUT/pid" flora_pid/host/test.c flora_pid/flora_pid.c -lm
run pid "$OUT/pid"

# shellcheck disable=SC2086
$CC $CFLAGS -Iflora_nav/include -o "$OUT/nav" flora_nav/host/test.c flora_nav/flora_navcore.c \
    flora_nav/flora_ekf.c flora_nav/flora_sun.c flora_nav/flora_sungate.c -lm
run nav "$OUT/nav"

# shellcheck disable=SC2086
$CC $CFLAGS -Iflora_heading/include -I../node/main -o "$OUT/heading" flora_heading/host/test.c \
    flora_heading/flora_magcal.c flora_heading/flora_hkf.c flora_heading/flora_magdist.c -lm
run heading "$OUT/heading"

# Funkcje autonomiczne (domyslnie wylaczone na robocie, hub_autocfg.h):
# straznik, jazda do punktu, podlewanie, energia, szukanie slonca, misja.
for c in guard; do
    # shellcheck disable=SC2086
    $CC $CFLAGS -Iflora_$c/include -o "$OUT/$c" flora_$c/host/test.c flora_$c/flora_$c.c -lm
    run $c "$OUT/$c"
done

# Nagranie -> odtworzenie: syntetyczny przejazd przez replay (flora_navcore).
NAVSRC="flora_nav/flora_navcore.c flora_nav/flora_ekf.c flora_nav/flora_sun.c flora_nav/flora_sungate.c"
# shellcheck disable=SC2086
$CC $CFLAGS -Iflora_nav/include -o "$OUT/simlog" flora_nav/host/simlog.c -lm
# shellcheck disable=SC2086
$CC $CFLAGS -Iflora_nav/include -o "$OUT/replay" flora_nav/host/replay.c $NAVSRC -lm
replay_check() {
    truth=$("$OUT/simlog" 2>&1 >"$OUT/sim.log" | sed -n 's/.*(\(-*[0-9]*\), \(-*[0-9]*\)).*/\1 \2/p')
    res=$("$OUT/replay" --end "$truth" "$OUT/sim.log")
    echo "$res"
    full=$(echo "$res" | awk '$1 == "full" { print $NF }')
    odom=$(echo "$res" | awk '$1 == "odom" { print $NF }')
    echo "$res" | grep -q 'luk w nagraniu 0, zlych linii 0' || { echo "FAIL  nagranie z bledami"; return 1; }
    [ "$full" -lt 100 ] || { echo "FAIL  full: blad $full mm"; return 1; }
    [ "$odom" -gt $((full * 10)) ] || { echo "FAIL  full nie lepszy od odometrii"; return 1; }
    "$OUT/simlog" --drop 40 >"$OUT/simd.log" 2>/dev/null
    "$OUT/replay" "$OUT/simd.log" | grep -q 'luk w nagraniu 1,' || { echo "FAIL  luka niewykryta"; return 1; }
    echo "WYNIK: OK (full $full mm, odom $odom mm)"
}
run replay replay_check

# Wykrywanie przeszkod: harness potrzebuje nagranych klatek (poza repo),
# wiec tu tylko sprawdzenie, ze rdzen sie kompiluje.
# shellcheck disable=SC2086
$CC $CFLAGS -Iflora_ground/include -o "$OUT/ground" flora_ground/host/runner.c flora_ground/flora_ground.c -lm
echo "== ground"
echo "WYNIK: kompiluje sie (bez danych testowych)"

if [ $fail = 0 ]; then
    echo "WSZYSTKO OK"
else
    echo "SA BLEDY"
    exit 1
fi
