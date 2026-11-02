#!/usr/bin/env python3
"""Prototyp wykrywania wolnej przestrzeni na surowych klatkach RGB565.

To jest prototyp algorytmu z planu, uruchamiany na Macu na klatkach zebranych
z glowicy. Pracuje na SIATCE POLARNEJ wokol srodka obrazu, bo kamera patrzy
prosto w dol, wiec odwzorowanie jest promieniowe, nie wierszowe.

CO ROBI, A CZEGO NIE: odpowiada, ktore komorki obrazu sa podlozem, i w ktorym
pierscieniu kazdego wycinka konczy sie wolna przestrzen. NIE podaje
centymetrow - zamiana pierscienia na odleglosc wymaga wysokosci kamery nad
ziemia i kalibracji miarka, a to ma sens dopiero przy docelowym montazu.
Dlatego wynik jest w PIERSCIENIACH i PIKSELACH. Ta sama logika, z tablica
geometrii na koncu, da potem `sectors[16]` w cm do `flora_obstacles_t`.

ZASADA DZIALANIA (agnostyczna wobec podloza, bo robot jezdzi po kostce,
betonie, tarasie, a pozniej po trawie): nie uczymy sie, jak wyglada
przeszkoda. W kazdej klatce uczymy sie od nowa, jak wyglada PODLOZE tutaj i
teraz - z pierscienia najblizej robota, po ktorym wlasnie jedzie - i szukamy
miejsca, w ktorym przestaje tak wygladac. Cechy sa oparte na CHROMATYCZNOSCI,
nie na jasnosci, bo na jednolitym betonie cien ma wiekszy kontrast niz
prawdziwa przeszkoda.

Zaleznosci: zadne. Sama biblioteka standardowa Pythona, zeby dalo sie to
uruchomic bez venva i bez pip.

Uzycie:

    ./ground.py data/20261101-kostka-slonce/*.rgb565
    ./ground.py kadr.rgb565 --overlay-dir /tmp/podglad --thr 0.45

Wynik: dla kazdej klatki plik `<nazwa>.overlay.png` (zielone = podloze,
czerwone = przeszkoda, szare = nie wiem) oraz linia JSON na stdout.
"""

import argparse
import binascii
import json
import math
import os
import struct
import sys
import zlib
from array import array
from collections import deque

# ---------------------------------------------------------------- RGB565 ---

# Kolejnosc bajtow i uklad bitow sa takie, jak dekoduje sterownik
# esp32-camera (`conversions/to_bmp.c`): pierwszy bajt piksela jest gorny.
#   R = hb & 0xF8
#   G = (hb & 0x07) << 5 | (lb & 0xE0) >> 3
#   B = (lb & 0x1F) << 3


def decode_rgb565(buf, n):
    """Rozpakowuje do trzech tablic bajtow. Jeden przebieg, bez numpy."""
    r, g, b = bytearray(n), bytearray(n), bytearray(n)
    for i in range(n):
        hb = buf[2 * i]
        lb = buf[2 * i + 1]
        r[i] = hb & 0xF8
        g[i] = ((hb & 0x07) << 5) | ((lb & 0xE0) >> 3)
        b[i] = (lb & 0x1F) << 3
    return r, g, b


# ------------------------------------------------------------ siatka polar ---

class PolarGrid:
    """Przypisanie pikseli do komorek (wycinek, pierscien). Liczone RAZ.

    Srodek obrazu to punkt pod kamera. Promien rosnie na zewnatrz, wiec
    pierscien 0 jest najblizej robota, a skan "od srodka na zewnatrz" to
    po prostu rosnacy indeks pierscienia.

    Wycinek 0 jest po lewej: azymut liczymy od -fan/2 do +fan/2 wokol
    kierunku jazdy, zgodnie z konwencja `sectors[]` "od lewej do prawej"
    z flora_msgs.h. Kierunek jazdy w obrazie ustawia --forward.
    """

    def __init__(self, w, h, sectors, rings, r_in, r_out, fan_deg, forward_deg):
        self.w, self.h = w, h
        self.sectors, self.rings = sectors, rings
        self.r_in, self.r_out = r_in, r_out
        self.cells = sectors * rings
        # -1 znaczy "piksel nie nalezy do zadnej komorki": albo jest pod
        # bryla robota (promien < r_in), albo za daleko, albo w naroznikach
        # kadru, gdzie dystorsja 160 stopni czyni obraz bezuzytecznym.
        self.cellmap = array("h", [-1]) * (w * h)

        cx, cy = (w - 1) / 2.0, (h - 1) / 2.0
        half = fan_deg / 2.0
        step = fan_deg / sectors
        ring_step = (r_out - r_in) / rings

        for y in range(h):
            dy = y - cy
            for x in range(w):
                dx = x - cx
                rad = math.hypot(dx, dy)
                if rad < r_in or rad >= r_out:
                    continue
                # Azymut w obrazie: 0 w prawo, rosnie w dol. Przeliczamy na
                # kat wzgledem kierunku jazdy.
                ang = math.degrees(math.atan2(dy, dx)) - forward_deg
                while ang < -180.0:
                    ang += 360.0
                while ang > 180.0:
                    ang -= 360.0
                if ang < -half or ang >= half:
                    continue
                sec = int((ang + half) / step)
                ring = int((rad - r_in) / ring_step)
                if sec >= sectors:
                    sec = sectors - 1
                if ring >= rings:
                    ring = rings - 1
                self.cellmap[y * w + x] = ring * sectors + sec

    def cell(self, sec, ring):
        return ring * self.sectors + sec


# ------------------------------------------------------------------ cechy ---

CHROMA_BINS = 12          # 12x12 kubelkow chromatycznosci na komorke

# PROGI JASNOSCI DECYDUJA, KTORE PIKSELE MAJA PRAWO GLOSOWAC.
#
# Chromatycznosc liczymy jako udzial kanalu w sumie kanalow, a przy malej
# sumie ten udzial jest zdominowany przez szum czujnika: piksel (12,9,11)
# i piksel (9,12,11) to ten sam prawie czarny punkt, ale wpadaja do roznych
# kubelkow barwy. Piksel zbyt ciemny albo przeswietlony nie jest wiec
# dowodem na przeszkode - jest brakiem informacji i ma trafic do licznika
# "nie wiem", bo to jest roznica miedzy 0 a 254 w kontrakcie.
#
# 90 na sumie trzech kanalow to srednio 30 na kanal. Ponizej tego w praktyce
# nie da sie wiarygodnie powiedziec, jakiego koloru jest piksel.
DARK_DEFAULT = 90         # suma kanalow ponizej tego to piksel bez barwy
BRIGHT_DEFAULT = 246      # kanal powyzej tego to przester


class CellFeatures:
    __slots__ = ("hist", "n", "dark", "bright", "y_sum", "y2_sum")

    def __init__(self):
        self.hist = [0] * (CHROMA_BINS * CHROMA_BINS)
        self.n = 0
        self.dark = 0
        self.bright = 0
        self.y_sum = 0
        self.y2_sum = 0

    def texture(self):
        """Odchylenie standardowe jasnosci w komorce."""
        if self.n < 8:
            return 0.0
        mean = self.y_sum / self.n
        var = self.y2_sum / self.n - mean * mean
        return math.sqrt(var) if var > 0 else 0.0

    def covered(self):
        """Czy komorka w ogole ma piksele w kadrze.

        Przy kamerze w nadirze zasieg w pikselach ZALEZY OD KIERUNKU: po
        dluzszej osi obrazu siega 160 px, po krotszej tylko 120. Komorki
        poza kadrem to nie jest "nie wiem", tylko koniec pola widzenia w
        tym kierunku - i skan ma sie na nich po prostu skonczyc, a nie
        zglaszac slepote.
        """
        return (self.n + self.dark + self.bright) >= 16

    def usable(self):
        """Komorka jest w kadrze, ale czy da sie ja ocenic.

        To zrodlo wartosci 0 (nieznane) w kontrakcie. Zero i "czysto" to
        NIE to samo i mylenie ich konczy sie wjechaniem w przeszkode.
        """
        total = self.n + self.dark + self.bright
        if total < 16:
            return False
        return (self.dark + self.bright) / total < 0.5


def extract(grid, r, g, b, dark=DARK_DEFAULT, bright=BRIGHT_DEFAULT):
    cells = [CellFeatures() for _ in range(grid.cells)]
    cm = grid.cellmap
    scale = CHROMA_BINS - 1
    for i in range(grid.w * grid.h):
        c = cm[i]
        if c < 0:
            continue
        f = cells[c]
        ri, gi, bi = r[i], g[i], b[i]
        if ri > bright or gi > bright or bi > bright:
            f.bright += 1
            continue
        s = ri + gi + bi
        if s < dark:
            f.dark += 1
            continue
        # Chromatycznosc znormalizowana: niewrazliwa na jasnosc, wiec
        # ten sam beton w sloncu i w cieniu wpada do tego samego kubelka.
        rb = (ri * scale) // s
        gb = (gi * scale) // s
        f.hist[gb * CHROMA_BINS + rb] += 1
        y = (ri + 2 * gi + bi) >> 2
        f.y_sum += y
        f.y2_sum += y * y
        f.n += 1
    return cells


# ----------------------------------------------------------- wzorzec ziemi ---

class GroundRef:
    """Wzorzec podloza: MASKA kubelkow chromatycznosci, ktore sa podlozem.

    DLACZEGO MASKA, A NIE PODOBIENSTWO HISTOGRAMOW: porownywanie
    statystyk calej komorki (przeciecie histogramow, wariancja, tekstura)
    przepada na fugach kostki. Fuga to ciemna linia szerokosci 3 px w
    komorce szerokiej 20 px: statystyka komorki zmienia sie wyraznie,
    choc 85% jej powierzchni to nadal podloze.

    Decyzja musi wiec byc liczona PER PIKSEL, a dopiero potem agregowana
    jako UDZIAL powierzchni. Wtedy fuga daje 15% niezgodnych pikseli i
    przechodzi, a cegla daje 90% i nie przechodzi. Udzial powierzchni
    przelicza sie pozniej tablica geometrii na rozmiar fizyczny w cm,
    czyli jest to ta sama bramka na rozmiar, ktora jest w planie.
    """

    def __init__(self, mask, total, tex_median):
        self.mask = mask
        self.total = total
        self.tex_median = tex_median

    @staticmethod
    def from_hist(hist, total, cover):
        """Wzorzec z gotowego histogramu - uzywane przy pominieciu wycinka."""
        if total < 64:
            return None
        order = sorted(range(len(hist)), key=lambda k: hist[k], reverse=True)
        mask = bytearray(len(hist))
        acc = 0
        for k in order:
            if hist[k] == 0:
                break
            mask[k] = 1
            acc += hist[k]
            if acc >= cover * total:
                break
        return GroundRef(bytes(mask), total, 0.0)

    @staticmethod
    def from_dominant(cells, idxs, cover):
        """Wzorzec z DOMINUJACYCH barw zbioru komorek.

        Do wzorca pierscienia nie wolno uzyc tej samej reguly co do wzorca
        spod robota. Tam zakladamy, ze CALY obszar jest podlozem, bo robot
        po nim jedzie. Pierscien takiego zalozenia nie ma - lezy w nim
        przeszkoda, ktorej szukamy, i gdyby wpuscic ja do wzorca, uznalaby
        sama siebie za podloze. Sprawdzone: przy regule "kazdy kubelek,
        ktory wystapil" detektor przestawal widziec cokolwiek.

        Bierzemy wiec kubelki od najliczniejszego, az pokryja `cover`
        pikseli pierscienia. Reszta, czyli ogon rozkladu, zostaje
        przeszkoda. Przy przeszkodzie zajmujacej dwa, trzy wycinki z
        szesnastu jej barwa siedzi wlasnie w tym ogonie.
        """
        hist = [0] * (CHROMA_BINS * CHROMA_BINS)
        total = 0
        tex = []
        for i in idxs:
            f = cells[i]
            if not f.usable():
                continue
            for k, v in enumerate(f.hist):
                hist[k] += v
            total += f.n
            tex.append(f.texture())
        if total < 64 or not tex:
            return None
        order = sorted(range(len(hist)), key=lambda k: hist[k], reverse=True)
        mask = bytearray(len(hist))
        acc = 0
        for k in order:
            if hist[k] == 0:
                break
            mask[k] = 1
            acc += hist[k]
            if acc >= cover * total:
                break
        tex.sort()
        return GroundRef(bytes(mask), total, tex[len(tex) // 2])

    @staticmethod
    def from_cells(cells, idxs, floor=0.002):
        hist = [0] * (CHROMA_BINS * CHROMA_BINS)
        total = 0
        tex = []
        for i in idxs:
            f = cells[i]
            if not f.usable():
                continue
            for k, v in enumerate(f.hist):
                hist[k] += v
            total += f.n
            tex.append(f.texture())
        if total < 64 or not tex:
            return None
        # Kubelek jest "podlozem", gdy w pierscieniu odniesienia wystapil
        # istotnie, a nie raz na krzyz. Prog odsiewa szum czujnika, ktory
        # rozmazuje chromatycznosc na sasiednie kubelki.
        cut = max(2.0, floor * total)
        mask = bytes(1 if v >= cut else 0 for v in hist)
        tex.sort()
        return GroundRef(mask, total, tex[len(tex) // 2])

    def match(self, f):
        """Udzial pikseli komorki, ktorych chromatycznosc jest podlozem."""
        if f.n < 16:
            return 0.0
        hit = 0
        m, hist = self.mask, f.hist
        for k in range(len(hist)):
            if m[k]:
                hit += hist[k]
        return hit / f.n


def ring_match(ring, f, ring_hist, ring_total, ring_order, cover):
    """Udzial pikseli komorki nalezacych do podloza pierscienia.

    Blizniacza do `ring_match()` w components/flora_ground/flora_ground.c -
    kazda zmiana tutaj musi isc rownolegle tam, bo to wersja w C jedzie na
    robocie, a ten plik ma ja przewidywac.
    """
    rh = ring_hist[ring]
    loo_total = max(ring_total[ring] - f.n, 0)
    if loo_total < 64 or f.n < 16:
        return 0.0
    goal = cover * loo_total
    acc_loo = acc_cell = 0
    for k in ring_order[ring]:
        loo = rh[k] - f.hist[k]
        if loo <= 0 and f.hist[k] == 0:
            continue
        acc_loo += max(loo, 0)
        acc_cell += f.hist[k]
        if acc_loo >= goal:
            break
    return acc_cell / f.n


# --------------------------------------------------------------- opisy ---

# Font 3x5 na cyfry. Rysowanie liczb wprost na nakladce jest warte tych
# kilkunastu linii: bez numerow trzeba liczyc wycinki palcem po ekranie,
# a przy 16 wycinkach i 12 pierscieniach latwo pomylic sie o jeden.
_GLYPHS = {
    "0": ("111", "101", "101", "101", "111"),
    "1": ("010", "110", "010", "010", "111"),
    "2": ("111", "001", "111", "100", "111"),
    "3": ("111", "001", "111", "001", "111"),
    "4": ("101", "101", "111", "001", "001"),
    "5": ("111", "100", "111", "001", "111"),
    "6": ("111", "100", "111", "101", "111"),
    "7": ("111", "001", "001", "001", "001"),
    "8": ("111", "101", "111", "101", "111"),
    "9": ("111", "101", "111", "001", "111"),
}


def draw_text(rgb, w, h, x0, y0, text, color, scale=1):
    """Wypisuje liczbe na buforze RGB. Poza kadrem po cichu przycina."""
    cx = x0
    for ch in text:
        g = _GLYPHS.get(ch)
        if g:
            for row, bits in enumerate(g):
                for col, bit in enumerate(bits):
                    if bit != "1":
                        continue
                    for sy in range(scale):
                        for sx in range(scale):
                            px = cx + col * scale + sx
                            py = y0 + row * scale + sy
                            if 0 <= px < w and 0 <= py < h:
                                i = (py * w + px) * 3
                                rgb[i] = color[0]
                                rgb[i + 1] = color[1]
                                rgb[i + 2] = color[2]
        cx += 4 * scale


def draw_grid(grid, rgb, args):
    """Rysuje granice wycinkow i pierscieni plus numery wycinkow.

    Numer kazdego wycinka ladnie wypada tuz za zewnetrzna granica siatki,
    czyli dokladnie tam, gdzie patrzy sie szukajac przyczyny trafienia.
    """
    import math as _m
    w, h = grid.w, grid.h
    cx, cy = (w - 1) / 2.0, (h - 1) / 2.0
    half = args.fan / 2.0
    step = args.fan / args.sectors
    ring_step = (args.r_out - args.r_in) / args.rings

    # Granice pierscieni: co drugi, zeby nie zasypac obrazu.
    for k in range(0, args.rings + 1, 2):
        r = args.r_in + ring_step * k
        for deg in range(int(-half * 4), int(half * 4)):
            a = _m.radians(deg / 4.0 + args.forward)
            x = int(cx + r * _m.cos(a))
            y = int(cy + r * _m.sin(a))
            if 0 <= x < w and 0 <= y < h:
                i = (y * w + x) * 3
                rgb[i] = rgb[i + 1] = rgb[i + 2] = 255

    # Granice wycinkow i numery.
    for sec in range(args.sectors + 1):
        a = _m.radians(-half + step * sec + args.forward)
        for t in range(int(args.r_in), int(args.r_out)):
            x = int(cx + t * _m.cos(a))
            y = int(cy + t * _m.sin(a))
            if 0 <= x < w and 0 <= y < h:
                i = (y * w + x) * 3
                rgb[i] = rgb[i + 1] = rgb[i + 2] = 255
        if sec < args.sectors:
            am = _m.radians(-half + step * (sec + 0.5) + args.forward)
            r = args.r_out + 6
            x = int(cx + r * _m.cos(am)) - 3
            y = int(cy + r * _m.sin(am)) - 3
            draw_text(rgb, w, h, x, y, str(sec), (255, 255, 0))


# --------------------------------------------------------------- overlay ---

def write_png(path, w, h, rgb):
    """Minimalny zapis PNG: truecolor 8 bit, bez przeplotu.

    Kodowanie PNG jest proste (zlib plus CRC), w przeciwienstwie do
    dekodowania - dlatego podglad zapisujemy, a wejscie czytamy jako
    surowe RGB565. Zero zaleznosci.
    """
    raw = bytearray()
    for y in range(h):
        raw.append(0)                       # filtr "none" na wiersz
        raw += rgb[y * w * 3:(y + 1) * w * 3]

    def chunk(tag, data):
        c = struct.pack(">I", len(data)) + tag + data
        return c + struct.pack(">I", binascii.crc32(tag + data) & 0xFFFFFFFF)

    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n")
        f.write(chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0)))
        f.write(chunk(b"IDAT", zlib.compress(bytes(raw), 6)))
        f.write(chunk(b"IEND", b""))


def overlay(grid, r, g, b, verdict):
    """Obraz z nalozona decyzja: zielone podloze, czerwona przeszkoda, szare nie wiem."""
    w, h = grid.w, grid.h
    out = bytearray(w * h * 3)
    tint = {
        "ground": (0, 255, 0),
        "obstacle": (255, 40, 40),
        "unknown": (160, 160, 160),
    }
    cm = grid.cellmap
    for i in range(w * h):
        c = cm[i]
        base = i * 3
        if c < 0:
            # Poza siatka: obraz przygaszony, zeby bylo widac, co w ogole
            # bylo brane pod uwage.
            out[base] = r[i] // 3
            out[base + 1] = g[i] // 3
            out[base + 2] = b[i] // 3
            continue
        if verdict[c] == "outside":
            out[base] = r[i] // 3
            out[base + 1] = g[i] // 3
            out[base + 2] = b[i] // 3
            continue
        tr, tg, tb = tint[verdict[c]]
        out[base] = (r[i] + tr) >> 1
        out[base + 1] = (g[i] + tg) >> 1
        out[base + 2] = (b[i] + tb) >> 1
    return out


# ------------------------------------------------------------------ potok ---

def process(grid, buf, refs, args):
    n = grid.w * grid.h
    r, g, b = decode_rgb565(buf, n)
    cells = extract(grid, r, g, b, args.dark, args.bright)

    # Wzorzec z dwoch najblizszych pierscieni: to powierzchnia, po ktorej
    # robot wlasnie jedzie.
    inner = [grid.cell(s, ring) for ring in range(min(2, grid.rings))
             for s in range(grid.sectors)]
    ref = GroundRef.from_cells(cells, inner)
    if ref:
        refs.append(ref)        # pamiec kilku podlozy: deque z maxlen

    # WZORZEC OSOBNO NA KAZDY PIERSCIEN.
    #
    # Obiektyw 160 stopni mocno winietuje, a winieta nie tylko przyciemnia
    # brzeg kadru - zmienia tam takze BARWE. Zmierzone na zbiorze z
    # panelami: najdalszy pierscien byl zglaszany jako przeszkoda w polowie
    # wycinkow TAKZE na czystej podlodze, bo nie pasowal do wzorca zdjetego
    # ze srodka kadru. Jeden wzorzec na cala klatke jest wiec nie do
    # utrzymania przy takim obiektywie.
    #
    # Wzorzec pierscienia budujemy z calego pierscienia, czyli z 16
    # wycinkow naraz. Przeszkoda zajmujaca dwa czy trzy wycinki nie
    # zdominuje tej statystyki, a winieta, ktora dotyczy calego pierscienia
    # rowno, zostaje odjeta sama z siebie.
    # Histogram kazdego pierscienia. Wzorzec budujemy dopiero przy ocenie
    # konkretnej komorki, zeby mozna bylo ODJAC JEJ WLASNY WYCINEK: inaczej
    # przeszkoda wchodzi do wzorca, wedlug ktorego jest oceniana, i uznaje
    # sama siebie za podloze.
    ring_hist = []
    ring_total = []
    ring_order = []
    if args.ring_ref:
        for ring in range(grid.rings):
            h = [0] * (CHROMA_BINS * CHROMA_BINS)
            tot = 0
            for sec in range(grid.sectors):
                f = cells[grid.cell(sec, ring)]
                if not f.usable():
                    continue
                for k, v in enumerate(f.hist):
                    h[k] += v
                tot += f.n
            ring_hist.append(h)
            ring_total.append(tot)
            ring_order.append(sorted(range(len(h)), key=lambda k: h[k], reverse=True))

    verdict = ["unknown"] * grid.cells
    scores = [0.0] * grid.cells
    for c, f in enumerate(cells):
        if not f.covered():
            verdict[c] = "outside"
            continue
        if not f.usable() or not refs:
            continue
        # Komorka jest podlozem, gdy zgadza sie z KTORYMKOLWIEK z ostatnich
        # wzorcow. Bierzemy maksimum, nie sume masek: suma rozszerzalaby
        # zbior akceptowanych barw z kazdym nowym podlozem, a przegapiona
        # przeszkoda jest bledem groznym, falszywe zatrzymanie tylko
        # uciazliwym. Komorka na granicy dwoch powierzchni moze przez
        # klatke lub dwie byc zgloszona, dopoki nowe podloze nie wejdzie
        # w pierscien odniesienia - robot i tak po nim jedzie.
        if args.ring_ref:
            # Kolejnosc kubelkow liczona RAZ na pierscien, a komorka wymaga
            # jednego przebiegu - tak samo jak w wersji C. Odejmowanie
            # wlasnego wkladu komorki zostaje, zmienia sie tylko to, skad
            # bierzemy kolejnosc: z pelnego pierscienia, nie z pierscienia
            # bez tej komorki. Rozbieznosc ta dotyczyla jednej granicznej
            # komorki na 27 klatek, a oszczedza okolo stu przebiegow po
            # histogramie na komorke.
            ring = c // grid.sectors
            best = ring_match(ring, f, ring_hist, ring_total,
                              ring_order, args.ring_cover)
            if best < args.thr:
                best = max([best] + [rf.match(f) for rf in refs])
        else:
            best = max(rf.match(f) for rf in refs)
        scores[c] = best
        verdict[c] = "ground" if best >= args.thr else "obstacle"

    # Skan od srodka na zewnatrz, z dwoma regulami potwierdzenia.
    #
    # DWA KOLEJNE pierscienie niezgodne, zanim uznamy przeszkode: jeden to
    # szum, plamka albo kamyk ponizej progu rozmiaru.
    #
    # DWA KOLEJNE pierscienie nieznane KONCZA skan jako "nie wiem", a NIE
    # jako "czysto". Nie widzimy przez obszar, ktorego nie umiemy ocenic,
    # wiec nie wolno obiecywac, ze za nim jest wolne. Komentarz przy
    # flora_obstacles_t mowi to wprost: 0 i 254 to nie to samo, a mylenie
    # ich konczy sie wjechaniem w przeszkode. Pojedyncza nieznana komorka
    # jest tolerowana i nie zeruje serii przeszkody - inaczej plama
    # przeswietlenia na krawedzi obiektu kasowalaby sam obiekt.
    first_ring = []
    for s in range(grid.sectors):
        hit = None
        blind = False
        run = 0
        unk_run = 0
        for ring in range(grid.rings):
            v = verdict[grid.cell(s, ring)]
            if v == "outside":
                break           # koniec pola widzenia w tym kierunku
            if v == "unknown":
                unk_run += 1
                if unk_run >= 2:
                    blind = True
                    break
                continue
            unk_run = 0
            if v == "obstacle":
                run += 1
                if run >= 2:
                    hit = ring - 1
                    break
            else:
                run = 0
        if hit is not None:
            first_ring.append(hit)
        elif blind:
            first_ring.append(-1)           # odpowiednik FLORA_OBSTACLE_UNKNOWN
        else:
            first_ring.append(grid.rings)   # odpowiednik FLORA_OBSTACLE_CLEAR

    unusable = sum(1 for f in cells if not f.usable())
    vote = sum(f.n for f in cells)
    lost = sum(f.dark + f.bright for f in cells)
    return {
        "first_ring": first_ring,
        "lost_px_pct": round(100.0 * lost / max(vote + lost, 1), 1),
        "ring_px": [round(grid.r_in + (grid.r_out - grid.r_in) * k / grid.rings, 1)
                    for k in range(grid.rings + 1)],
        "unusable_cells": unusable,
        "valid": unusable < grid.cells // 2 and bool(refs),
        "match_min": round(min(scores), 3),
        "match_median": round(sorted(scores)[len(scores) // 2], 3),
    }, (r, g, b), verdict


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("frames", nargs="+", help="pliki .rgb565 z /raw")
    ap.add_argument("--width", type=int, default=320)
    ap.add_argument("--height", type=int, default=240)
    ap.add_argument("--sectors", type=int, default=16,
                    help="tyle, ile FLORA_OBSTACLE_SECTORS (domyslnie 16)")
    ap.add_argument("--rings", type=int, default=12)
    ap.add_argument("--r-in", type=float, default=40.0,
                    help="promien wewnetrzny w px: obrys bryly robota")
    ap.add_argument("--r-out", type=float, default=150.0,
                    help="promien zewnetrzny w px: dalej dystorsja zabija obraz")
    ap.add_argument("--fan", type=float, default=160.0,
                    help="szerokosc wachlarza w stopniach wokol kierunku jazdy")
    ap.add_argument("--forward", type=float, default=0.0,
                    help="kierunek jazdy w obrazie w stopniach; 0 to w prawo, "
                         "90 w dol. Zalezy od obrotu kamery w uchwycie")
    ap.add_argument("--thr", type=float, default=0.70,
                    help="minimalny udzial pikseli zgodnych z wzorcem podloza, "
                         "0..1. Ponizej tego komorka jest przeszkoda")
    ap.add_argument("--dark", type=int, default=DARK_DEFAULT,
                    help="suma kanalow, ponizej ktorej piksel jest 'nie wiem' "
                         f"(domyslnie {DARK_DEFAULT})")
    ap.add_argument("--bright", type=int, default=BRIGHT_DEFAULT,
                    help="kanal powyzej tego to przester, piksel jest 'nie wiem'")
    ap.add_argument("--ring-cover", type=float, default=0.85,
                    help="jaki udzial pikseli pierscienia uznajemy za podloze; "
                         "reszta rozkladu to przeszkoda (domyslnie 0.90)")
    ap.add_argument("--no-ring-ref", dest="ring_ref", action="store_false",
                    help="wzorzec jeden na cala klatke zamiast osobno na "
                         "pierscien. Do porownania, bo przy obiektywie 160 "
                         "stopni winieta falszuje brzeg kadru")
    ap.add_argument("--memory", type=int, default=3,
                    help="ile ostatnich wzorcow podloza pamietac")
    ap.add_argument("--grid", action="store_true",
                    help="dorysuj granice wycinkow, pierscieni i NUMERY wycinkow - "
                         "do odczytania, ktory wycinek to ktory fragment kadru")
    ap.add_argument("--overlay-dir", default=None,
                    help="gdzie zapisac podglady PNG (domyslnie obok klatek)")
    args = ap.parse_args()

    expect = args.width * args.height * 2
    grid = PolarGrid(args.width, args.height, args.sectors, args.rings,
                     args.r_in, args.r_out, args.fan, args.forward)
    in_grid = sum(1 for i in range(args.width * args.height) if grid.cellmap[i] >= 0)
    # Ile pierscieni kazdego wycinka faktycznie miesci sie w kadrze. Przy
    # nadirze to NIE jest stale: po dluzszej osi obrazu siega dalej.
    px = [0] * grid.cells
    for i in range(args.width * args.height):
        c = grid.cellmap[i]
        if c >= 0:
            px[c] += 1
    reach = [sum(1 for ring in range(args.rings) if px[grid.cell(sec, ring)] >= 16)
             for sec in range(args.sectors)]
    print(f"# siatka {args.sectors}x{args.rings}, {in_grid} px w siatce "
          f"({100 * in_grid / (args.width * args.height):.0f}% kadru), "
          f"promien {args.r_in:.0f}-{args.r_out:.0f} px", file=sys.stderr)
    print(f"# zasieg wycinkow w pierscieniach: {reach}", file=sys.stderr)

    refs = deque(maxlen=max(1, args.memory))
    prev_session = None
    for path in args.frames:
        # Pamiec wzorcow podloza jest zamierzona TYLKO w obrebie jednej
        # sekwencji: ma przetrwac wjazd z kostki na beton, a nie przeniesc
        # wzorzec z wczorajszej sesji w innym pomieszczeniu. Bez tego
        # zerowania wzorzec z obcej sceny rozszerza zbior akceptowanych
        # barw i detektor przestaje widziec przeszkode - sprawdzone na
        # zywych danych, klatka z noga gubila sie po doklejeniu obcej sesji.
        session = os.path.dirname(os.path.abspath(path))
        if session != prev_session:
            refs.clear()
            prev_session = session
        with open(path, "rb") as f:
            buf = f.read()
        if len(buf) != expect:
            print(f"# {path}: {len(buf)} B, oczekuje {expect} B "
                  f"({args.width}x{args.height} RGB565) - pomijam", file=sys.stderr)
            continue

        res, (r, g, b), verdict = process(grid, buf, refs, args)

        out_dir = args.overlay_dir or os.path.dirname(path) or "."
        os.makedirs(out_dir, exist_ok=True)
        # Nazwa sesji w nazwie pliku, bo kazda sesja numeruje klatki od
        # 000001 i bez tego nakladki z dwoch sesji nadpisuja sie nawzajem
        # w jednym katalogu wyjsciowym.
        sess = os.path.basename(os.path.dirname(os.path.abspath(path)))
        stem = os.path.basename(path).rsplit(".", 1)[0]
        png = os.path.join(out_dir, f"{sess}-{stem}.overlay.png"
                           if args.overlay_dir else f"{stem}.overlay.png")
        img = overlay(grid, r, g, b, verdict)
        if args.grid:
            draw_grid(grid, img, args)
        write_png(png, args.width, args.height, img)

        res["frame"] = os.path.basename(path)
        res["refs"] = len(refs)
        res["overlay"] = png
        print(json.dumps(res, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    sys.exit(main())
