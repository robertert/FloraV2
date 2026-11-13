#!/bin/sh
# Test toru nagranie -> odtworzenie bez robota: simlog pisze syntetyczny
# przejazd w formacie hub_rec, replay liczy go flora_navcore.
#   sh replay_test.sh      # kod wyjscia 0 = OK
set -e
cd "$(dirname "$0")"
CFLAGS="-O2 -Wall -I../include"
cc $CFLAGS -o simlog simlog.c -lm
cc $CFLAGS -o replay replay.c ../flora_navcore.c ../flora_ekf.c ../flora_sun.c ../flora_sungate.c -lm

truth=$(./simlog 2>&1 >sim.log | sed -n 's/.*(\(-*[0-9]*\), \(-*[0-9]*\)).*/\1 \2/p')
fail=0
check() { if [ "$1" = 1 ]; then echo "OK    $2"; else echo "FAIL  $2"; fail=1; fi; }
err() { awk -v v="$1" '$1 == v { print $NF }'; }

out=$(./replay --end "$truth" sim.log)
full=$(echo "$out" | err full)
odom=$(echo "$out" | err odom)
echo "$out" | head -1
echo "  blad zamkniecia: full $full mm, odom $odom mm (prawda $truth)"
check "$(echo "$out" | grep -c 'luk w nagraniu 0, zlych linii 0')" "nagranie czytane bez bledow"
check "$([ "$full" -lt 100 ] && echo 1)" "full: blad < 100 mm"
check "$([ "$odom" -gt "$((full * 10))" ] && echo 1)" "full co najmniej 10 x lepszy niz sama odometria"

fixed=$(./replay --end "$truth" --scale-r 1.01 --track 400 sim.log | err odom)
echo "  odom po poprawce kol i rozstawu: $fixed mm"
check "$([ "$fixed" -lt "$((odom / 2))" ] && echo 1)" "poprawiona kalibracja w replay zmniejsza blad odometrii"

./simlog --drop 40 >simd.log 2>/dev/null
check "$(./replay simd.log | grep -c 'luk w nagraniu 1,')" "zgubiona paczka wykryta"

rm -f sim.log simd.log
[ $fail = 0 ] && echo "WYNIK: OK" || { echo "WYNIK: BLEDY"; exit 1; }
