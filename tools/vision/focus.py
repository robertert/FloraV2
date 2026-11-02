#!/usr/bin/env python3
"""Ustawianie ostrosci obiektywu na liczbe, nie na oko.

Obiektyw szerokokatny ma ogromna glebie ostrosci, wiec roznica miedzy
ostrym a prawie ostrym jest na podgladzie ledwo widoczna - zwlaszcza przez
strumien po WiFi, ktory i tak sie zacina. Latwiej krecic tubusem i patrzec
na rosnaca liczbe.

MIARA: srednia roznica jasnosci miedzy sasiednimi pikselami, podzielona
przez srednia jasnosc. Ostry obraz ma wiecej drobnych szczegolow, wiec
wieksze roznice miedzy sasiadami. Dzielenie przez jasnosc sprawia, ze
wynik nie skacze, gdy automatyka ekspozycji zmieni naswietlenie.

DWIE LICZBY, bo przy 160 stopniach srodek i brzeg kadru ostrza sie przy
roznym polozeniu tubusu (krzywizna pola). Srodek jest wazniejszy do
oceny samego obiektywu, ale to PIERSCIEN na polowie promienia odpowiada
obszarowi, w ktorym wykrywamy przeszkody - jesli musisz wybrac, wybierz
lepszy pierscien.

Uzycie: polóz pod kamera cos drobno zadrukowanego (gazeta, linijka z
podzialka), wlacz tryb raw i krec tubusem powoli.

    ./focus.py --host 172.20.10.2
"""

import argparse
import sys
import time
import urllib.request

W, H = 320, 240


def sharpness(buf, x0, x1, y0, y1, ring=None):
    """Gradient poziomy w obszarze, znormalizowany jasnoscia."""
    grad = 0
    lum = 0
    n = 0
    cx, cy = W / 2, H / 2
    for y in range(y0, y1):
        row = y * W
        for x in range(x0, x1 - 1):
            if ring:
                d = ((x - cx) ** 2 + (y - cy) ** 2) ** 0.5
                if not (ring[0] <= d < ring[1]):
                    continue
            i = (row + x) * 2
            # Jasnosc przybliżamy gornym bajtem: R w 5 bitach plus gorne
            # bity G. Do porownania ostrosci to zupelnie wystarcza i jest
            # trzy razy szybsze niz pelne rozpakowanie RGB.
            a = buf[i]
            b = buf[i + 2]
            grad += abs(a - b)
            lum += a
            n += 1
    if n == 0 or lum == 0:
        return 0.0
    return 1000.0 * grad / lum


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", required=True)
    ap.add_argument("-i", "--interval", type=float, default=1.5)
    args = ap.parse_args()

    url = f"http://{args.host}/raw"
    best_c = best_r = 0.0
    print("Krec tubusem powoli. Szukasz MAKSIMUM obu liczb.")
    print("Gwiazdka znaczy nowy rekord od uruchomienia programu.\n")
    print(f"{'srodek':>8} {'pierscien':>10}   {'':20}")
    try:
        while True:
            t0 = time.time()
            try:
                with urllib.request.urlopen(url, timeout=5) as r:
                    buf = r.read()
            except OSError as e:
                print(f"  blad: {e}")
                time.sleep(1)
                continue
            if len(buf) != W * H * 2:
                print(f"  klatka {len(buf)} B - czy glowica jest w trybie raw?")
                return 1

            c = sharpness(buf, 100, 220, 70, 170)
            g = sharpness(buf, 0, W, 0, H, ring=(60, 110))
            mark = ""
            if c > best_c:
                best_c = c
                mark += " *srodek"
            if g > best_r:
                best_r = g
                mark += " *pierscien"
            bar = "#" * int(min(g, 60))
            print(f"{c:8.1f} {g:10.1f}   {bar}{mark}")

            took = time.time() - t0
            if took < args.interval:
                time.sleep(args.interval - took)
    except KeyboardInterrupt:
        print(f"\nnajlepsze: srodek {best_c:.1f}, pierscien {best_r:.1f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
