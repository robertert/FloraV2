#!/usr/bin/env python3
"""Generator syntetycznych klatek RGB565 do testow bez kamery.

Po co: potok z `ground.py`, a pozniej jego port do C, trzeba dac sie
sprawdzic na wejsciu o ZNANEJ odpowiedzi. Prawdziwa klatka z trawnika nie
ma znanej odpowiedzi, dopoki ktos jej nie zaetykietuje. Syntetyczna ma:
wiemy dokladnie, pod jakim azymutem i w jakim promieniu lezy przeszkoda,
wiec regresja w algorytmie wychodzi natychmiast.

Dwa przypadki, ktore ten generator umie odtworzyc i ktore sa w planie jako
ryzyka:
  - FUGI KOSTKI: regularne cienkie linie, ktore nie sa przeszkoda
  - ZMIANA POWIERZCHNI: pol kadru beton, pol kostka

Format wyjscia jest dokladnie taki, jak oddaje `/raw` z glowicy: surowe
RGB565 big-endian, bez naglowka.

Uzycie:

    ./synth.py /tmp/test.rgb565 --joints --obstacle 0,110,22
    ./synth.py /tmp/czysto.rgb565 --joints
    ./synth.py /tmp/granica.rgb565 --split
"""

import argparse
import math
import random
import sys

W, H = 320, 240


def pack(r, g, b):
    """RGB888 na dwa bajty RGB565 big-endian, jak w sterowniku kamery."""
    v = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)
    return (v >> 8) & 0xFF, v & 0xFF


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("out")
    ap.add_argument("--joints", action="store_true",
                    help="dorysuj fugi kostki: cienkie ciemne linie co 60 px")
    ap.add_argument("--split", action="store_true",
                    help="lewa polowa beton, prawa kostka - test zmiany powierzchni")
    ap.add_argument("--obstacle", action="append", default=[],
                    metavar="AZYMUT,PROMIEN,SREDNICA",
                    help="przeszkoda w stopniach (0 to w prawo), px od srodka, px")
    ap.add_argument("--shadow", type=float, default=0.0,
                    help="azymut klina cienia w stopniach; 0 wylacza")
    ap.add_argument("--noise", type=int, default=6)
    ap.add_argument("--seed", type=int, default=1)
    args = ap.parse_args()

    random.seed(args.seed)
    obstacles = []
    for spec in args.obstacle:
        az, rad, dia = (float(x) for x in spec.split(","))
        cx = (W - 1) / 2 + rad * math.cos(math.radians(az))
        cy = (H - 1) / 2 + rad * math.sin(math.radians(az))
        obstacles.append((cx, cy, dia / 2))

    CONCRETE = (178, 176, 170)      # szary beton, chromatycznosc prawie neutralna
    PAVING = (164, 150, 138)        # kostka, lekko cieplejsza
    JOINT = (96, 92, 88)
    BRICK = (152, 68, 48)           # cegla: wyraznie inna chromatycznosc

    cx0, cy0 = (W - 1) / 2, (H - 1) / 2
    buf = bytearray(W * H * 2)
    for y in range(H):
        for x in range(W):
            base = PAVING if (args.split and x > W / 2) else CONCRETE
            r, g, b = base

            if args.joints and (args.split is False or x > W / 2):
                if x % 60 < 3 or y % 60 < 3:
                    r, g, b = JOINT

            for ox, oy, orad in obstacles:
                if math.hypot(x - ox, y - oy) <= orad:
                    r, g, b = BRICK
                    break

            # Cien: mocny spadek jasnosci BEZ zmiany chromatycznosci. Jesli
            # algorytm go zglasza jako przeszkode, znaczy, ze opiera sie na
            # jasnosci, a nie na barwie, i trzeba to naprawic.
            if args.shadow:
                ang = math.degrees(math.atan2(y - cy0, x - cx0))
                d = (ang - args.shadow + 540) % 360 - 180
                if abs(d) < 25:
                    r, g, b = r // 3, g // 3, b // 3

            n = args.noise
            if n:
                r = min(255, max(0, r + random.randint(-n, n)))
                g = min(255, max(0, g + random.randint(-n, n)))
                b = min(255, max(0, b + random.randint(-n, n)))

            hb, lb = pack(r, g, b)
            i = (y * W + x) * 2
            buf[i] = hb
            buf[i + 1] = lb

    with open(args.out, "wb") as f:
        f.write(buf)

    print(f"{args.out}: {W}x{H} RGB565, {len(buf)} B")
    for az, spec in zip((s.split(",")[0] for s in args.obstacle), args.obstacle):
        print(f"  przeszkoda na azymucie {az} st, {spec.split(',')[1]} px od srodka")
    return 0


if __name__ == "__main__":
    sys.exit(main())
