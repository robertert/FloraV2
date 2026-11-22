#!/usr/bin/env python3
"""
Eksport przejazdu z InfluxDB (przez API Grafany) do plikow CSV.

Po co: telemetria lezy tylko w jednej bazie na serwerze. Przejazdy opisane
w DZIENNIK_TESTOW.md trzymamy dodatkowo w repo, zeby dalo sie do nich wrocic
(wykresy do pracy, porownanie wariantow) bez serwera.

Uzycie:
    export GRAFANA_URL=https://<serwer>
    export GRAFANA_TOKEN_FLORA=<token>      # nigdy nie wpisuj go do repo
    tools/telemetry/export_run.py 2026-11-22T17:34:00Z 2026-11-22T17:39:00Z testy/x/przejazd3

Czasy w UTC (ISO 8601). Kazdy measurement idzie do osobnego CSV: kolumna
time (UTC, ISO), source (tele = bryla, head = glowica), potem pola.
Puste measurementy sa pomijane.
"""
import csv
import json
import os
import re
import sys
import urllib.request
from datetime import datetime, timezone

DEFAULT_MEASUREMENTS = [
    # warianty filtra pozycji (tele/pose, tele/pose_<nazwa>)
    "pose", "pose_odom", "pose_head", "pose_head_sun", "pose_odom_tags", "pose_no_odom",
    # pomiary
    "tags", "attitude", "mag", "drive", "power", "sonar",
    # kalibracje i diagnostyka
    "mag_cal", "level_cal", "track_cal", "wheel_cal", "tunnel", "boot", "log",
]


def query(url, token, flux):
    body = json.dumps({
        # maxDataPoints: bez tego Grafana obcina wynik do 1001 punktow
        # (limit dla wykresu, nie dla eksportu).
        "queries": [{"refId": "A", "datasource": {"uid": "flora-influx", "type": "influxdb"},
                     "query": flux, "maxDataPoints": 100000, "intervalMs": 1000}],
        "from": "now-30d", "to": "now",
    }).encode()
    req = urllib.request.Request(url.rstrip("/") + "/api/ds/query", data=body, headers={
        "Content-Type": "application/json", "Authorization": "Bearer " + token})
    for attempt in range(3):         # serwer bywa wolny na dluzszych oknach
        try:
            with urllib.request.urlopen(req, timeout=120) as r:
                return json.load(r)
        except OSError:
            if attempt == 2:
                raise


# Repo jest publiczne: z logow wycinamy nazwe sieci WiFi i adresy IP.
_REDACT = [
    (re.compile(r'(siec|laczenie z|z sieci|SSID)( *"+)[^"]+("+)', re.I), r'\1\2<wifi>\3'),
    (re.compile(r'\b\d{1,3}(\.\d{1,3}){3}\b'), '<ip>'),
]


def redact(v):
    if isinstance(v, str):
        for rx, rep in _REDACT:
            v = rx.sub(rep, v)
    return v


def export(url, token, start, stop, out_dir, measurements):
    os.makedirs(out_dir, exist_ok=True)
    for m in measurements:
        flux = (f'from(bucket:"flora") |> range(start:{start}, stop:{stop}) '
                f'|> filter(fn:(r)=>r._measurement=="{m}") '
                f'|> pivot(rowKey:["_time"], columnKey:["_field"], valueColumn:"_value") '
                f'|> drop(columns:["_start","_stop","_measurement","robot"]) '
                f'|> group() |> sort(columns:["_time"])')
        res = query(url, token, flux)["results"]["A"]
        frames = res.get("frames", [])
        rows, cols = [], []
        for fr in frames:
            names = [f["name"] for f in fr["schema"]["fields"]]
            for n in names:
                if n not in cols:
                    cols.append(n)
            for vals in zip(*fr["data"]["values"]):
                rows.append(dict(zip(names, vals)))
        if not rows:
            continue
        order = ["_time", "source"] + [c for c in cols if c not in ("_time", "source")]
        path = os.path.join(out_dir, f"{m}.csv")
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["time" if c == "_time" else c for c in order])
            for r in sorted(rows, key=lambda r: r.get("_time") or 0):
                out = []
                for c in order:
                    v = r.get(c)
                    if c == "_time" and isinstance(v, (int, float)):
                        v = datetime.fromtimestamp(v / 1000, timezone.utc).isoformat(timespec="milliseconds")
                    out.append("" if v is None else redact(v))
                w.writerow(out)
        print(f"  {m}: {len(rows)} wierszy -> {path}")


def main():
    if len(sys.argv) < 4:
        print(__doc__)
        sys.exit(1)
    url = os.environ.get("GRAFANA_URL")
    token = os.environ.get("GRAFANA_TOKEN_FLORA")
    if not url or not token:
        sys.exit("ustaw GRAFANA_URL i GRAFANA_TOKEN_FLORA")
    start, stop, out_dir = sys.argv[1:4]
    measurements = sys.argv[4].split(",") if len(sys.argv) > 4 else DEFAULT_MEASUREMENTS
    export(url, token, start, stop, out_dir, measurements)


if __name__ == "__main__":
    main()
