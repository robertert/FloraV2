# TODO

## Straznicy napedu i bezpieczenstwa (odlozone - brak czesci sprzetu)

Stan obecny (`flora_motors`): utyk z bezruchu (moc >= 15%, < 5 impulsow
przez 400 ms) i z nadpradu (> 2,5 A szyny przez 300 ms), zatrzask do
`cmd/stall_clear`. Lapie tylko twarda blokade kola.

### Architektura
- **Jedno waskie gardlo do silnikow.** Dzis `flora_motors_set` wolaja
  wprost `hub_drive`, `hub_headcal` (tryb pwm) i `drive_power` z panelu -
  wszystko ma isc przez straznika.
- `components/flora_guard` - czysty C, test na hoscie. Wejscia: zadane i
  zmierzone predkosci, PWM, prad, napiecie, zyro, przechyl, przeszkody,
  wiek danych. Wyjscie: dozwolona moc/predkosc osobno przod / tyl / obrot
  + lista aktywnych powodow.
- Poziomy reakcji:
  - LIMIT - ograniczenie mocy (I2t, niskie napiecie),
  - STOP - zatrzymanie, samo puszcza (przeszkoda, przechyl, stare dane),
  - LATCH - blokada do recznego zdjecia (utyk, prad, buksowanie, przekladnia).
- `hub/main/hub_safety.c` wolany z petli 20 ms `hub_drive`.
- Telemetria `tele/safety`: stan, powod, poziom I2t, licznik zadzialan.

### Straznicy (w kolejnosci wdrazania)
1. **Weto przeszkod** - kierunkowe: przeszkoda z przodu blokuje jazde
   naprzod, pozwala cofnac i obrocic; tylny sonar pilnuje cofania; sektory
   "nie wiem" (brak cm) = zajete. Wymaga: wysokosc obiektywu glowicy nad
   ziemia (cm w `FLORA_MSG_OBSTACLES`).
2. **Przechyl** - pitch/roll z glowicy > ~20 st -> STOP (maszt podnosi
   srodek ciezkosci). Wymaga: kalibracja poziomu na docelowym robocie.
3. **Napiecie baterii** - LiFePO4 pod obciazeniem < ~11,8 V -> stop jazdy
   (pompka i lacznosc zostaja), zanim BMS odetnie wszystko.
4. **Swiezosc danych** - ATTITUDE / OBSTACLES starsze niz ~500 ms -> stop
   autonomii, reczna jazda co najwyzej pelzanie.
5. **Zyro vs odometria** - enkoder jest na wale silnika, nie na kole:
   luzny zebnik / zjechana przekladnia (R-09) i buksowanie sa niewidoczne
   dla obecnego utyku. Kola "skrecaja" np. 30 st/s, zyro widzi 0 przez
   > 1 s -> LATCH. Na start sam obrot (jednoznaczny); jazda prosto pozniej
   przez EKF (odometria rosnie, tagi/slonce mowia, ze stoi).
6. **Nasycenie PID i predkosc vs PWM** - PID ukrywa slabnacy silnik.
   Wyjscie PID ~100% > 2 s albo predkosc < 50% modelu
   `v = kff * (pwm - ks)` w stanie ustalonym -> ostrzezenie/STOP.
   Wymaga: nastrojony PID na docelowym robocie.
7. **I2t (przegrzanie)** - `E += (I^2 - I_ciagly^2) * dt`, stygniecie przy
   malym pradzie; przekroczenie -> LIMIT 50%, potem STOP. INA226 mierzy
   cala szyne: odjac pompke i maszt, rozdzial na kola wg PWM albo najgorszy
   przypadek. Wymaga: prad ciagly JGB37-520 (pomiar albo karta + zapas).
8. **Asymetria L/P** - przy jezdzie prosto jedno kolo trwale potrzebuje
   > 30% wiecej mocy -> tylko ostrzezenie (wczesny objaw zuzycia).

Deadman z panelu juz jest.

## Opcjonalne (nie krytyczne)

Polecana kolejnosc: nagrywanie/odtwarzanie -> testy w CI -> DLI.

### Pod prace dyplomowa
- **Nagrywanie i odtwarzanie przejazdow** - surowy strumien (enkodery,
  ATTITUDE, tagi, slonce) na MQTT -> plik na serwerze; ten sam EKF
  (`flora_nav`, czysty C) puszczany na hoscie na nagraniu: strojenie Q/R,
  porownanie wariantow, wykresy bez wyjazdu w teren. Najcenniejsze.
- **Punkt odniesienia do oceny EKF** - tasma + znaczniki na trasie, robot
  staje na punktach kontrolnych; ambitniej telefon na statywie z gory +
  tag na robocie.
- **Mapa trajektorii w panelu** - XY wszystkich 6 wariantow + pozycje tagow.
- **Dzienna dawka swiatla (DLI)** z VEML7700 - calka natezenia w ciagu
  dnia vs wymagania gatunku rosliny.
- **Bilans energii** - licznik Wh pobranych/oddanych z INA226 + napiecie
  baterii w cyklu dobowym (czy 20 W panelu wystarcza); pomiar tylko po
  stronie baterii, wiec bilans netto.

### Wygoda w rozwoju
- **OTA dla glowicy i tagcamu** - bryla przekazuje obraz po UART
  kawalkami albo glowica wlacza wlasne WiFi na czas aktualizacji.
- **Testy hostowe w CI** - `flora_pid`, `flora_nav`, `flora_heading`,
  `flora_ground` przy kazdym pushu.
- **Zrzut przyczyny awarii** - coredump do flash, po restarcie skrot na
  MQTT (zadanie, adres, backtrace).
- **Mapa tagow w NVS / przez MQTT** zamiast `s_tag_map` w kodzie.

### Funkcje "doniczkowe"
- **Powiadomienia** - alerty Grafany albo bot Telegram: malo wody, niska
  bateria, utyk, robot offline (tylko serwer).
- **Pogoda w decyzji o podlewaniu** - spadek cisnienia z BME280 albo
  prognoza z internetu -> wstrzymaj podlewanie.
- **Timelapse rosliny** - glowica raz dziennie robi zdjecie na serwer.
- **Ciche godziny** - bez jazdy w nocy / w zadanych porach.

### Dopieszczenie
- **Kalibracja magnetometru 3D** - zdjeta glowica obracana recznie we
  wszystkich osiach (dzis 2D, os Z nieskorygowana).
- **Samouczenie mapy tagow** - pozycje palikow z odometrii i kursu
  (mini SLAM); dobre na rozdzial "rozszerzenia".
- **Pozostale kalibracje** - rozstaw kol (UMBmark), offset i strefa
  martwa sonarow, wzmocnienia kanalow VEML, ogniskowa kamery tagow.
