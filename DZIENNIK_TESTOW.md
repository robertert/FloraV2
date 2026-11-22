# Dziennik testow

Co bylo testowane, w jakich ustawieniach, co wyszlo i co z tego wynika.
Najnowsze wpisy na gorze. Listy kontrolne do odhaczania sa w
`TESTY_RECZNE.md` - tu jest historia.

**Dane.** Telemetria lezy w InfluxDB na serwerze (Grafana). Przejazdy
z tego dziennika sa dodatkowo zrzucone do `testy/<data>-<nazwa>/`, po
jednym CSV na temat (`pose*.csv` warianty filtra, `tags.csv`, `attitude.csv`,
`mag.csv`, `drive.csv`, `log.csv`, ...). Z logow wycinana jest nazwa sieci
WiFi i adresy IP. Zrzut nowego przejazdu (czasy w UTC):

```sh
export GRAFANA_URL=https://<serwer>
export GRAFANA_TOKEN_FLORA=<token>     # nie do repo
tools/telemetry/export_run.py 2026-11-22T17:34:00Z 2026-11-22T17:39:30Z testy/2026-11-22-plansza/3-...
```

**Uklad wspolrzednych** (tele/pose): +x do przodu robota przy zerowaniu,
+y w lewo, kurs dodatni w lewo (CCW). Punkt (0, 0) to srodek osi kol
w chwili `pose_reset`.

**Warianty filtra** (liczone rownolegle na tych samych danych, hub_nav.h):
`pose` pelny (odometria + kurs glowicy + slonce + znaczniki),
`pose_odom` sama odometria, `pose_odom_tags` odometria + znaczniki,
`pose_head` odometria + kurs glowicy, `pose_head_sun`, `pose_no_odom`.

---

## 2026-11-22 - plansza 2,5 x 2 m, znacznik, magnetometr, rozstaw kol

### Stanowisko

- Plansza 2,5 m (lewo-prawo) x 2 m (przod-tyl), kratka 0,5 m, w pomieszczeniu.
  (0, 0) 1 m od lewego i 1 m od przedniego boku. Rogi: x od -1000 do +1000,
  y od -1500 (prawy bok) do +1000 (lewy bok).
- Znacznik tag36h11, id 0, wydruk 152 mm, srodek 40 cm nad podloga,
  48,5 cm za przednim bokiem planszy: w mapie (1485, 0).
- Kamera znacznikow na glowicy (obiektyw 68 st), 83 cm nad podloga,
  ok. 10 cm przed osia kol (zmierzone na oko).
- Bryla na tymczasowym ESP bez WiFi (uszkodzone, patrz nizej), MQTT przez
  tunel UART do glowicy (`hub_tunnel.c`, galaz `tymczasowy-tunel`).
  Telemetria w tunelu ograniczona do 1 wiadomosci na temat na sekunde.

### Ustawienia w chwili przejazdow

| Klucz | Wartosc | Uwagi |
|---|---|---|
| `cam_x_mm` / `cam_y_mm` | 100 / 0 | na oko, do dokladnego pomiaru |
| `cam_yaw_deg` | -1,6 | znacznik na osi przy (0, 0) |
| `cam_h_mm` / `tag_h_mm` | 830 / 400 | odleglosc pozioma z ukosnej |
| `FLORA_HEAD_TAG_SIZE_MM` | 152 | TYMCZASOWO, docelowo 300 |
| `FLORA_HEAD_TAG_MIRROR` | 1 | kamera glowicy daje obraz lustrzany |
| znacznik w mapie | `0 1485 0` | |
| `track_mm` | 390 (przejazdy 1-2), **384,7** (przejazd 3) | z `track_cal` |
| `site_lat` / `site_lon` | miejsce testu (nie Krakow) | |
| `mag_decl` | 6,1 E | NOAA |

### Zdarzenia i poprawki po drodze

- **ESP bryly uszkodzone** (21.11, przy przykrecaniu masztu, najpewniej
  wyladowanie): 3V3 na plytce 3,15-3,2 V przy 4,92 V na wejsciu, brownout
  przy starcie WiFi, bez WiFi dziala stabilnie. Obejscie: tunel MQTT przez
  glowice. Do wymiany.
- **Lewe kolo tylko do przodu** - luzny przewod DIR, nie ESP.
- **Kamera glowicy lustrzana**: znacznik nie dekodowal sie wcale; po
  odbiciu lewo-prawo id 0, decision_margin 134. Poprawka `FLORA_HEAD_TAG_MIRROR`.
- **Pamiec glowicy**: AprilTag zjadal wewnetrzny RAM (wolne 6 kB, minimum 0),
  padalo TLS i MQTT. TLS i zwykle malloc do PSRAM (`node/sdkconfig.defaults`):
  wolne 153 kB, minimum 78 kB.
- **Skok kursu glowicy o 154 st** (przejazd 1, punkt 5): glowica bez
  zakotwiczenia przestawila kurs na zaklocony magnetometr, bryla po 10
  odrzuceniach przyjela go sila i pelny filtr odjechal o metr. Poprawki:
  skok kursu glowicy bez obrotu w zyroskopie przesuwa tylko b, blokada
  kursu przy dzialajacym zyroskopie tez (flora_nav, test hostowy na tym
  przypadku); glowica startujaca bez magnetometru zostaje przy kursie
  wzglednym do `mag_cal`. Sprawdzone na zywo: po `mag_cal` skok -154,7 st
  przeszedl do b, poza bez zmian.

### Kalibracje (katalogi `2a-...`, `3a-...`)

- **Poziom** (`level_cal auto`): przekrzywienie plytki -0,79 / 0,43 st
  (wczesniej 2,05 / -5,09 - glowica byla przekrecana).
- **Magnetometr** (`mag_cal auto 40`, na (0, 0)): 400 probek, 12/12 sektorow,
  osie 1,07, rozrzut 3,8%, hard iron (-15,6; 14,9; -40,3) uT, pole poziome
  19,9 uT, inklinacja -66,6 st.
- **Rozstaw kol** (`track_cal 3`): w lewo 384,2 mm, w prawo 385,3 mm,
  wynik **384,7 mm** (bylo domyslne 390, czyli +1,4%).

### Przejazdy

Trasa: maly kwadrat po kratce 0,5 m (przod, prawo, tyl, tyl, lewo, przod),
w przejazdach 2-3 dalej rog (1000, 1000) i koniec (-500, 1000). Postoj
ok. 15 s w punkcie, przodem do znacznika. Blad = odleglosc od naroznika
kratki [mm]; samo ustawienie robota na kratce to ok. +-2-3 cm.

**Przejazd 1** - bez magnetometru (kurs glowicy wzgledny), rozstaw 390,
przed poprawka skokow kursu. 22.11, 02:50-03:02 CET.

| Punkt | Odometria | Odom + znacznik | Pelny |
|---|---|---|---|
| (500, 0) | 34 | 35 | 35 |
| (0, -500) | 35 | 41 | 41 |
| (-500, -500) | 25 | 43 | 43 |
| (-500, 0) | 6 | 11 | **ok. 820** (skok kursu glowicy) |
| (0, 0) | 59 | 49 | **ok. 1240** |

**Przejazd 2** - magnetometr po kalibracji, rozstaw 390. 18:17-18:25 CET.

| Punkt | Odometria | Odom + znacznik | Odom + glowica | Pelny |
|---|---|---|---|---|
| (500, 0) | 35 | 46 | 35 | 47 |
| (0, -500) | 49 | 35 | 79 | 74 |
| (-500, -500) | 48 | 71 | 8 | 14 |
| (-500, 0) | 7 | 18 | 47 | 49 |
| (0, 0) | 38 | 32 | 52 | 60 |
| (1000, 1000) | 52 | 72 | 136 | 161 |
| (-500, 1000) | 146 | 105 | 55 | 46 |
| kurs na koncu | -23,5 st | -21,6 st | -0,3 st | 0,8 st |

**Przejazd 3** - magnetometr, rozstaw **384,7**. 18:34-18:39 CET.

| Punkt | Odometria | Odom + znacznik | Odom + glowica | Pelny |
|---|---|---|---|---|
| (500, 0) | 37 | 82 | 33 | 81 |
| (500, -500) | 59 | 38 | 58 | 48 |
| (0, -500) | 63 | 41 | 51 | 38 |
| (-500, -500) | 66 | 65 | 38 | 57 |
| (-500, 0) | 92 | 23 | 88 | 38 |
| (0, 0) | 73 | 49 | 56 | 30 |
| (1000, 1000) | 36 | 21 | 153 | 86 |
| (-500, 1000) | 49 | 41 | 67 | 74 |
| **srednio** | **59** | **45** | **69** | **56** |
| kurs na koncu | -2,8 st | -4,5 st | 3,9 st | 0,2 st |

### Wnioski

1. **Rozstaw kol** byl glowna przyczyna bledu kursu odometrii: -23,5 st na
   koncu trasy przy 390 mm, -2,8 st przy 384,7 mm.
2. **Odometria + znacznik** najlepsze (srednio 4,5 cm), tyle co dokladnosc
   ustawienia robota. Znacznik pomaga tylko w kadrze - w tylnej czesci
   planszy i tylem do niego wariant zachowuje sie jak sama odometria.
3. **Magnetometr w pomieszczeniu**: powstrzymuje narastajacy blad kursu
   (koniec przejazdu 2: pelny 46 mm, odometria 146 mm), ale lokalnie ciagnie
   kurs o kilka stopni - w rogu (1000, 1000) w obu przejazdach odom + glowica
   myli sie o 14-15 cm. Glowica sama oznaczyla tam pole jako zaklocone.
4. **Znaczniki przesuwaja pozycje o ok. 3-5 cm do tylu** wzgledem kratki -
   najpewniej `cam_x_mm` zmierzone na oko albo odleglosc znacznika.
5. Jeden znacznik nie wyznacza kursu: kurs i y wymieniaja sie miedzy soba,
   gdy kurs z glowicy jest slaby. Potrzebny drugi znacznik w innym kierunku.

### Do zrobienia po tych testach

- [ ] `cam_x_mm` zmierzyc z dokladnoscia do 1 cm
- [ ] drugi znacznik (lewy albo tylny bok planszy)
- [ ] sprawdzic na nagranych danych podloge niepewnosci kursu glowicy
      (`head_sigma_floor_deg`, dzis 4 st) - 8-10 st w pomieszczeniu?
- [ ] tunel: `tele/boot` bryly nie przechodzi (dluzszy niz 512 B)
- [ ] po wymianie ESP bryly: `TEST_MQTT_PRZEZ_GLOWICE 0`, lacze bryla-glowica
      z powrotem 57600 (`hub_links.c`, `head_links.c`), moc WiFi 20 dBm
      w lokalnym `hub/sdkconfig`, `FLORA_HEAD_TAG_SIZE_MM` 300 przy palikach,
      `FLORA_HEAD_CAM_NET_DEFAULT` 0
