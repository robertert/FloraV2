#!/usr/bin/env python3
"""
Zapis nagrania przejazdu (cmd/rec) do pliku.

Czyta flora/<robot>/rec/data przez mosquitto_sub - bez zaleznosci z pip,
jak reszta narzedzi. Kazda wiadomosc zaczyna sie od "#<numer>"; luka
w numeracji to zgubiona paczka (liczona i wypisywana). Plik nadaje sie
wprost do components/flora_nav/host/replay.

Przyklady:
  # broker lokalny, sam wlacza i wylacza nagrywanie na robocie
  tools/rec/record.py --start

  # serwer z TLS
  tools/rec/record.py --start -h <domena> -p 8883 -u panel -P <haslo> \\
      --cafile /etc/ssl/cert.pem -o trawnik_kwadrat.log

Ctrl+C konczy (z --start wysyla tez cmd/rec off).
"""
import argparse
import os
import re
import subprocess
import sys
import time
from datetime import datetime


def broker_args(a):
    out = ["-h", a.host, "-p", str(a.port)]
    if a.user:
        out += ["-u", a.user]
    if a.password:
        out += ["-P", a.password]
    if a.cafile:
        out += ["--cafile", a.cafile]
    return out


def send_rec(a, state):
    cmd = ["mosquitto_pub", *broker_args(a), "-t", f"flora/{a.robot}/cmd/rec", "-m", state]
    try:
        subprocess.run(cmd, check=True, timeout=10)
    except (subprocess.SubprocessError, OSError) as e:
        print(f"nie udalo sie wyslac cmd/rec {state}: {e}", file=sys.stderr)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter,
                                add_help=False)
    p.add_argument("--help", action="help")
    p.add_argument("-h", "--host", default="localhost")
    p.add_argument("-p", "--port", type=int, default=1883)
    p.add_argument("-u", "--user")
    p.add_argument("-P", "--password")
    p.add_argument("--cafile", help="certyfikat CA dla TLS (port 8883)")
    p.add_argument("--robot", default="robot1")
    p.add_argument("-o", "--out", help="plik wyjsciowy (domyslnie rec_<data>.log)")
    p.add_argument("--start", action="store_true",
                   help="wyslij cmd/rec on na poczatku i off na koncu")
    a = p.parse_args()

    out = a.out or datetime.now().strftime("rec_%Y%m%d_%H%M%S.log")
    sub = subprocess.Popen(
        ["mosquitto_sub", *broker_args(a), "-t", f"flora/{a.robot}/rec/data", "-N"],
        stdout=subprocess.PIPE, text=True, bufsize=1)
    # Subskrypcja przed wlaczeniem, zeby nie zgubic naglowka (V, P, M).
    time.sleep(1.0)
    if a.start:
        send_rec(a, "on")

    seq_re = re.compile(r"^#(\d+)$")
    last = None
    msgs = gaps = lines = ticks = 0
    t_shown = time.time()
    print(f"zapis do {out} (Ctrl+C konczy)")
    try:
        with open(out, "w") as f:
            for line in sub.stdout:
                f.write(line)
                lines += 1
                m = seq_re.match(line.strip())
                if m:
                    n = int(m.group(1))
                    msgs += 1
                    if last is not None and n != last + 1:
                        gaps += n - last - 1
                        print(f"  luka: brak paczek {last + 1}..{n - 1}")
                    last = n
                elif line.startswith("E,"):
                    ticks += 1
                if time.time() - t_shown > 5:
                    t_shown = time.time()
                    print(f"  {ticks / 50:6.0f} s jazdy, {msgs} paczek, zgubionych {gaps}, "
                          f"{os.path.getsize(out) / 1024:.0f} kB")
    except KeyboardInterrupt:
        pass
    finally:
        if a.start:
            send_rec(a, "off")
        sub.terminate()
    print(f"koniec: {ticks / 50:.0f} s, {lines} linii, {msgs} paczek, zgubionych {gaps} -> {out}")
    print("odtworzenie: components/flora_nav/host/replay " + out)


if __name__ == "__main__":
    main()
