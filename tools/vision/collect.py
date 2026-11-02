#!/usr/bin/env python3
"""Zbieranie zbioru do wykrywania przeszkod: klatki z glowicy + telemetria.

Klatki ciagnie z serwera podgladu w glowicy (`bringup_cam.c`, endpoint /jpg),
telemetrie z brokera MQTT przez `mosquitto_sub`. Zadnych zaleznosci z pip -
mosquitto jest w projekcie od poczatku (patrz README), a reszta to standardowa
biblioteka Pythona.

DLACZEGO TELEMETRIA LECI DO KAZDEJ KLATKI: etap 2 planu wymaga rozbicia metryk
po warunkach oswietleniowych i po podlozu, a bez zapisanego kontekstu zbior
staje sie nierozbieralny po tygodniu. Przy okazji dostajemy darmowe slabe
etykiety: sonar w mm i zliczenia enkoderow mowia, czy robot przez dany obszar
PRZEJECHAL, czyli czy byl przejezdny.

Uzycie:

    ./collect.py --host 192.168.1.42 --surface kostka --light slonce
    ./collect.py --host 192.168.1.42 --surface beton --light cien -i 0.5 -n 400

Wynik w `tools/vision/data/<data>-<podloze>-<swiatlo>/`:

    session.json    opis sesji: adres, podloze, swiatlo, notatka, tempo
    000001.jpg      klatki
    log.jsonl       po jednej linii na klatke: plik, czas, cala telemetria
"""

import argparse
import json
import os
import shutil
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request
from datetime import datetime

# Tematy, ktore faktycznie cos wnosza do etykietowania. Gwiazdka na koncu
# zbiera wszystko, ale log.jsonl robi sie wtedy trzy razy wiekszy bez zysku.
TOPICS_DEFAULT = "flora/robot1/#"


class Telemetry:
    """Ostatnia znana wartosc kazdego tematu, zbierana w tle.

    `mosquitto_sub -v` wypisuje po jednej linii na wiadomosc, w formacie
    "temat tresc". Czytamy to watkiem, bo pobieranie klatek po HTTP blokuje,
    a telemetria ma lecec niezaleznie.
    """

    def __init__(self, broker, port, topic):
        self.lock = threading.Lock()
        self.values = {}
        self.proc = None
        self.lines = 0
        cmd = ["mosquitto_sub", "-h", broker, "-p", str(port), "-v", "-t", topic]
        try:
            self.proc = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True
            )
        except FileNotFoundError:
            print("UWAGA: brak mosquitto_sub - zbieram same klatki, bez telemetrii.")
            print("       brew install mosquitto")
            return
        t = threading.Thread(target=self._reader, daemon=True)
        t.start()

    def _reader(self):
        for line in self.proc.stdout:
            parts = line.rstrip("\n").split(" ", 1)
            if len(parts) != 2:
                continue
            topic, payload = parts
            try:
                value = json.loads(payload)
            except json.JSONDecodeError:
                value = payload          # np. tele/pump albo ack/* to zwykly tekst
            with self.lock:
                self.values[topic] = (value, time.time())
                self.lines += 1

    def snapshot(self):
        """Migawka z wiekiem kazdej wartosci.

        Wiek jest wazniejszy, niz wyglada: `tele/sonar` leci co 500 ms, a
        `head/weather` co 10 s, wiec bez wieku nie odroznisz swiezego pomiaru
        od wartosci z poprzedniej sesji.
        """
        now = time.time()
        with self.lock:
            return {
                topic: {"v": value, "age_ms": int((now - ts) * 1000)}
                for topic, (value, ts) in self.values.items()
            }

    def stop(self):
        if self.proc:
            self.proc.terminate()


def fetch_frame(url, timeout):
    """Zwraca tresc i naglowki. Przy /raw w naglowkach sa wymiary klatki."""
    with urllib.request.urlopen(url, timeout=timeout) as r:
        return r.read(), dict(r.headers)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", required=True,
                    help="adres glowicy, np. 192.168.1.42 (z logu po starcie)")
    ap.add_argument("--surface", required=True,
                    help="podloze: kostka, plyty, beton, asfalt, taras, trawa")
    ap.add_argument("--light", required=True,
                    help="swiatlo: slonce, chmury, cien, zmierzch, po_deszczu")
    ap.add_argument("--note", default="",
                    help="co bylo na torze, np. 'cegla i doniczka na 60 cm'")
    ap.add_argument("-i", "--interval", type=float, default=1.0,
                    help="odstep miedzy klatkami w sekundach (domyslnie 1.0)")
    ap.add_argument("-n", "--count", type=int, default=0,
                    help="ile klatek; 0 znaczy do Ctrl+C")
    ap.add_argument("--raw", action="store_true",
                    help="pobieraj surowe RGB565 z /raw zamiast JPEG z /jpg. "
                         "Glowica musi byc w trybie raw: http://<ip>/mode?v=raw")
    ap.add_argument("--no-mqtt", action="store_true",
                    help="nie probuj telemetrii - przy kamerze na biurku, "
                         "bez bryly i bez brokera")
    ap.add_argument("--broker", default="localhost")
    ap.add_argument("--broker-port", type=int, default=1883)
    ap.add_argument("--topic", default=TOPICS_DEFAULT)
    ap.add_argument("--out", default=os.path.join(os.path.dirname(__file__), "data"))
    args = ap.parse_args()

    ext = "rgb565" if args.raw else "jpg"
    url = f"http://{args.host}/{'raw' if args.raw else 'jpg'}"
    stamp = datetime.now().strftime("%Y%m%d-%H%M")
    session_dir = os.path.join(args.out, f"{stamp}-{args.surface}-{args.light}")
    os.makedirs(session_dir, exist_ok=True)

    # Pierwsza klatka PRZED otwarciem plikow: jesli glowica nie odpowiada,
    # lepiej wyjsc z bledem niz zostawic pusty katalog sesji.
    try:
        first, hdrs = fetch_frame(url, timeout=5)
    except (urllib.error.URLError, OSError) as e:
        print(f"nie moge pobrac {url}: {e}")
        print("sprawdz adres i tryb glowicy: /jpg dziala w trybie jpeg,")
        print("/raw w trybie raw (przelaczysz przez http://<ip>/mode?v=raw)")
        return 1
    dims = (hdrs.get("X-Width"), hdrs.get("X-Height"))
    print(f"glowica odpowiada, pierwsza klatka {len(first) / 1024:.0f} kB"
          + (f", {dims[0]}x{dims[1]} {hdrs.get('X-Format', '')}" if dims[0] else ""))

    if args.no_mqtt:
        tele = None
        print("telemetria wylaczona (--no-mqtt)")
    else:
        tele = Telemetry(args.broker, args.broker_port, args.topic)
        time.sleep(1.0)      # chwila na pierwsze wiadomosci z brokera
        if tele.proc and not tele.values:
            print(f"UWAGA: broker {args.broker} nie przyslal nic - telemetria bedzie pusta")

    with open(os.path.join(session_dir, "session.json"), "w") as f:
        json.dump({
            "started": datetime.now().isoformat(timespec="seconds"),
            "host": args.host,
            "surface": args.surface,
            "light": args.light,
            "note": args.note,
            "interval_s": args.interval,
            "format": "rgb565be" if args.raw else "jpeg",
            "width": dims[0],
            "height": dims[1],
            "broker": None if args.no_mqtt else f"{args.broker}:{args.broker_port}",
            "topic": None if args.no_mqtt else args.topic,
        }, f, indent=2, ensure_ascii=False)

    log = open(os.path.join(session_dir, "log.jsonl"), "a")
    saved, errors, slow = 0, 0, 0
    t_start = time.time()

    print(f"zapisuje do {session_dir}, Ctrl+C konczy")
    try:
        while args.count == 0 or saved < args.count:
            t0 = time.time()
            try:
                data, _ = fetch_frame(url, timeout=5)
            except (urllib.error.URLError, OSError) as e:
                errors += 1
                print(f"  blad pobrania ({errors}): {e}")
                time.sleep(min(2.0, args.interval))
                continue

            saved += 1
            name = f"{saved:06d}.{ext}"
            with open(os.path.join(session_dir, name), "wb") as f:
                f.write(data)

            log.write(json.dumps({
                "frame": name,
                "t": round(t0, 3),
                "t_rel": round(t0 - t_start, 3),
                "bytes": len(data),
                "tele": tele.snapshot() if tele else {},
            }, ensure_ascii=False) + "\n")
            log.flush()      # sesja konczy sie Ctrl+C, wiec nie buforujemy

            took = time.time() - t0
            if took > args.interval:
                slow += 1    # samo pobranie dluzsze niz odstep: tempo nizsze od zadanego
            else:
                time.sleep(args.interval - took)

            if saved % 25 == 0:
                print(f"  {saved} klatek, {took * 1000:.0f} ms na ostatnia, "
                      f"tematow w telemetrii {len(tele.values) if tele else 0}")
    except KeyboardInterrupt:
        print()
    finally:
        log.close()
        if tele:
            tele.stop()

    dur = time.time() - t_start
    print(f"koniec: {saved} klatek w {dur:.0f} s ({saved / max(dur, 1):.1f} kl/s), "
          f"bledow {errors}, za wolnych {slow}")
    if not tele or not tele.values:
        print("telemetrii NIE zapisano - zbior nadaje sie do etykietowania recznego,")
        print("ale nie do slabych etykiet z sonaru i odometrii")
    du = shutil.disk_usage(session_dir)
    print(f"wolne na dysku: {du.free / 1e9:.1f} GB")
    return 0


if __name__ == "__main__":
    sys.exit(main())
