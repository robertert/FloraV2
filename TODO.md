# TODO

## PCB glowicy - nastepna wersja

- **RST muxa (TCA9548A, U4 pin 5) na wolny GPIO** (np. GPIO1 albo GPIO2,
  wolne wg netlisty). Dzis RST wisi w powietrzu, wiec reset ESP nie
  resetuje muxa: jeden zwarty kanal kladzie cala magistrale I2C i pomaga
  tylko odciecie zasilania. Z RST firmware zresetuje mux przy starcie,
  pominie zepsuty kanal i uruchomi reszte. Szczegoly: `CZESTE_BLEDY.md`.
- **Zlacza z zatrzaskiem dla VEML-i i BME** (np. JST-XH zamiast goldpinow).
  SCL lezy obok GND w J2 i J5 - krzywa wtyczka = zwarcie, a drgania przy
  jezdzie to rozrusza.

## Magnetometr glowicy - wlasne pole robota (15.11.2026)

Objaw: pole calkowite 83-89 uT zamiast ~50 (Krakow), prawie cale nadwyzki
PIONOWO (~35 uT). Kalibracja 2D (obrot w miejscu) widzi tylko pole poziome,
pionowego pola robota nie zmierzy. Kompensacja przechylu traktuje cale
zmierzone pole pionowe jak ziemskie, wiec:
- przy plytce przechylonej o 5,7 st kalibracja z 15.11 zapamietala ~8 uT
  przecieku jako hard iron, a kompensacja odjela go drugi raz - po
  kalibracji pole poziome 7-9 uT zamiast 15,4, magdist trzyma "zaklocenie"
  i kurs idzie z samego zyro. Po wyprostowaniu plytki: level_cal, mag_cal;
- na pochylosci blad = 35 uT * sin(nachylenie): 5 st -> ~10 st kursu,
  10 st -> ~20 st (przy ~17 uT pola poziomego).

ZROBIONE w kodzie (15.11, flora_magcal.h, test hostowy flora_heading):
- hard iron 3D: oz z inklinacji miejsca przy zwyklym mag_cal auto,
  model pomiaru w osiach plytki (o, soft iron), poziomowanie na koncu,
  przeciek pionowej czesci pola Ziemi odejmowany przy dopasowaniu;
- do magnetometru surowy przechyl plytki (`tilt_raw`), nie po level_cal;
- MAG_CAL_RESULT z offset_z (kontrakt v5), head/mag_cal: off_z_ut;
- symulacja: plytka 5,7 st, hard iron (12, -7, -40): odzyskany co do
  0,05 uT, kurs na plasko 0,1 st, na pochylosci 10 st 0,1 st (stary
  model 21 st).
ZOSTAJE:
- [ ] **Inklinacja miejsca** `FLORA_HEAD_MAG_INCL_DEG` (flora_head_board.h,
      dzis 66,5 dla Krakowa) z kalkulatora NOAA dla ogrodu, do 0,5 st:
      1 st bledu = ~2 st kursu na 10 st pochylosci.
- [ ] Wgrac glowice i bryle RAZEM (kontrakt v5), level_cal, mag_cal na
      docelowym miejscu; oczekiwane off_z_ut ~ -40, ref_dip_deg ~ -66,
      pole poziome po kalibracji w 15% od ref_h_ut (magdist bez zaklocenia).

Wykluczone: maszt (karbon, mosiezne inserty), antena WiFi (pole 2,4 GHz
usrednia sie do zera, magnetometr mierzy pole stale). Obracanie zdjetej
plytki wokol pionu dawalo stale ~85 - to nie rozstrzyga (pionowe zrodlo
na plytce i pole pomieszczenia daja to samo).

WYNIK 15.11: plytka do gory nogami ~10 uT (normalnie ~85). Zrodlo jest
NA PLYTCE GLOWICY: ~40 uT wzdluz osi Z plytki (w strone spodu), pionowe
pole Ziemi wychodzi ~40-45 uT, czyli pokoj w normie. Czysto w osi Z =
zrodlo milimetry pod/nad ICM: stalowe sruby/dystanse modulu ICM, niklowane
gniazda goldpinow pod nim, dlawik albo metalowa obudowa pod spodem.
Sprawdzic magnesem; po usunieciu |B| przy plytce poziomo ~50 uT.
Skoro pole jest przyklejone do plytki, kalibracja 3D zdjetej glowicy ma
sens (mierzy oz wprost); punkt 1 nizej to alternatywa bez zdejmowania.
Zdjecia plytki z obu stron (15.11): brak srub, pod ICM (z drugiej strony)
modul TCA9548A, najblizsze metalowe czesci 1,5-5 cm dalej (gniazda pod
ESP32, oslona modulu, slot SD) - za daleko i nie w osi Z.
POTWIERDZONE w nocie ICM-20948 (DS-000189 v1.5, tabela 3, str. 13):
"Zero-field output, initial calibration tolerance -2000..+2000 LSB" przy
0,15 uT/LSB = do +-300 uT na os. Nasze ~40 uT w Z to wlasne przesuniecie
zera AK09916, normalne dla tego ukladu. Stale i przyklejone do plytki,
wiec usuwalne kalibracja - punkt 1 nizej jest potrzebny (kazdy egzemplarz
ICM ma inne przesuniecie, wiec po wymianie modulu tez).
Rysunek 13 (str. 83) potwierdza przemapowanie osi w flora_icm20948.c:
X magnetometru = X akcelerometru, Y i Z odwrocone.

Rozstrzygajace (zrobione):
- **plytka do gory nogami** (oba razy poziomo): spadek z ~85 do ~20 =
  zrodlo na plytce; bez zmiany = otoczenie albo skala;
- **na zewnatrz**, z dala od aut i ogrodzen: ~50 = winne mieszkanie
  (zbrojenie, metalowa rama mebla) i kalibracje trzeba powtorzyc tam,
  gdzie robot jezdzi; telefon z aplikacja pokazujaca uT jako odniesienie.

Testy lokalizacji zrodla (patrz `field_ut` w head/mag, 1 Hz):
- glowica zdjeta z masztu, z dala od wszystkiego, obracana w rece: jesli
  |B| dalej ~85 i zmienia sie z orientacja - zrodlo na plytce glowicy
  (wtedy kalibracja 3D zdjetej glowicy ma sens); jesli ~50 - zrodlo
  w maszcie / bryle;
- magnes do srub, wkretow i elementow obok ICM (stal przyciaga);
- glowica na maszcie, maszt bez zasilania vs z zasilaniem (prad).

## Funkcje autonomiczne - napisane 17.11.2026, do sprawdzenia na sprzecie

W firmware bryly, domyslnie wylaczone (przelaczniki w `hub_autocfg.h`,
tabela w README). Testy hostowe przechodza, na robocie NIC z tego jeszcze
nie jechalo. Kolejnosc prob: `TESTY_RECZNE.md`, punkt 8.

- [ ] straznik (`guard_en`): punkty 1-4 z listy nizej
- [ ] jazda do punktu (`pilot_en`)
- [ ] podlewanie (`water_en`)
- [ ] szukanie slonca (`mission_en`)
- [ ] sen (`sleep_en`) i klucz MDD10A (`mdd_key_en`, czeka na modul klucza)

Swiadomie pominiete, do dopisania pozniej:
- karty w panelu (`tools/dashboard`) - na razie `mosquitto_sub` / `mosquitto_pub`;
- stan szukania slonca (postoj, przerwa po nieudanym szukaniu) ginie przy
  glebokim snie - do przeniesienia do pamieci RTC, gdy sen w dzien bedzie uzywany;
- wylaczanie WiFi glowicy poza serwisem (bilans energii);
- dawka pompki skalowana napieciem szyny (12,0-14,6 V);
- prog kontrastu krzyza VEML dla "swiatla bezposredniego" (`contrast_min`
  0,25 w `flora_seek.h`) jest zalozony, nie zmierzony - porownac `contrast`
  w `tele/mission` w sloncu i w cieniu.

### Licznik kulombow (stan naladowania baterii)

Polityka energii (`flora_energy`) decyduje dzis po napieciu, a to przy
LiFePO4 4S malo mowi: miedzy ~30 a ~90 % napiecie spoczynkowe zmienia sie
o ok. 0,3 V (13,0-13,3 V), tyle co szum i temperatura. Do tego slabe
ladowanie z panelu (< 0,8 A, przechodzi przez filtr) zawyza napiecie
i pusta bateria wyglada w dzien na pelna. Progi 12,6 / 12,2 V to w praktyce
jazda do ok. 10 % - malo zapasu na pochmurny tydzien.

- [ ] nowy komponent `flora_soc` (czyste C + `host/test.c`): calka pradu
  z INA226 (dodatni = pobor) w Ah i Wh, SoC = kotwica - Ah / pojemnosc
- [ ] kotwice: 100 % na koncu ladowania (napiecie > ~14 V i prad ladowania
  spada), dol krzywej po odpoczynku (< ~12,8 V, tam napiecie juz cos mowi)
- [ ] stan w pamieci RTC (`RTC_DATA_ATTR`, przezywa deep sleep) i co jakis
  czas w NVS; czas snu doliczany jako staly prad snu (zmierzony raz)
- [ ] `flora_energy` po SoC w % zamiast po napieciu; napiecie zostaje tylko
  jako twarde zabezpieczenie w strazniku (11,8 V)
- [ ] `tele/energy`: SoC, Ah i Wh pobrane / oddane na dobe - to od razu
  "Bilans energii" z listy pod prace dyplomowa

### Kalibracja na zlozonym robocie (gdy wszystko bedzie spiete)

Wszystkie progi i czasy w funkcjach autonomicznych sa zalozone albo
zmierzone na stole (zasilacz laboratoryjny 12,0 / 13,07 V, glowica
prawdopodobnie z USB, bez kamer i czesci czujnikow). Do powtorzenia na
docelowym robocie, na baterii:

- [ ] pobor w spoczynku z rozbiciem na odbiorniki (`tele/power`): bryla
  sama, + maszt (glowica BEZ USB), + MDD10A bez klucza, z kluczem;
  na stole wylaczenie masztu dalo tylko ~15 mA, co nie pasuje do glowicy
  z kamera i wizja
- [ ] prad w deep sleep (do licznika kulombow) i koszt jednego wybudzenia
- [ ] bilans dobowy: Wh z panelu 20 W vs zuzycie, osobno zima i lato;
  z niego czasy snu (`e_night_s`, `e_day_s`)
- [ ] progi trybow energii (`e_v_save`, `e_v_surv`) albo progi SoC po
  wdrozeniu licznika
- [ ] UWAGA przy testach na zasilaczu: 12,0 V < `e_v_surv` 12,2 V, wiec
  polityka od razu wchodzi w SURVIVAL (bez jazdy, bez podlewania, przy
  `sleep_en` maszt gasnie) - ustawic zasilacz na ~13,2 V albo obnizyc progi
- [ ] progi straznika: `g_front_mm`, `g_rear_mm` (droga hamowania przy
  `pid_vmax`), przechyl, `g_v_stop` pod obciazeniem na baterii
- [ ] podlewanie: `w_dry` / `w_wet` dla docelowej ziemi i rosliny, wzrost
  wilgotnosci po dawce (`rise_min_pct`), czas wsiakania
- [ ] szukanie slonca: `contrast_min` (patrz wyzej), `s_ratio`
- [ ] pilot: `p_vmax`, `p_wmax`, tolerancja dojazdu po strojeniu PID

## Straznicy napedu i bezpieczenstwa

Stan obecny (`flora_motors`): utyk z bezruchu (moc >= 15%, < 5 impulsow
przez 400 ms) i z nadpradu (> 2,5 A szyny przez 300 ms), zatrzask do
`cmd/stall_clear`. Lapie tylko twarda blokade kola.

Punkty 1-4 ponizej oraz "jedno waskie gardlo" sa w kodzie (`flora_guard`,
`hub_safety`, `hub_drive_set_power`), wylaczone do czasu prob. Punkty 5-8
nadal odlozone. Roznice wobec szkicu: poziomy to zgoda osobno na przod /
tyl / obrot plus limit mocy (bez osobnego LATCH - zatrzask zostaje
w `flora_motors`), straznik liczy sie we wlasnym zadaniu co 50 ms, a petla
napedu co 20 ms tylko pyta o werdykt.

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
- ZROBIONE (0d66993): **Nagrywanie i odtwarzanie przejazdow** - surowy strumien (enkodery,
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
- **OTA dla tagcamu** (glowica ma juz OTA przez wlasne WiFi, dacaa52) - bryla przekazuje obraz po UART
  kawalkami albo glowica wlacza wlasne WiFi na czas aktualizacji.
- ZROBIONE (tools/host_tests.sh, .github/workflows/tests.yml): **Testy hostowe w CI** - `flora_pid`, `flora_nav`, `flora_heading`,
  `flora_ground` przy kazdym pushu.
- **Zrzut przyczyny awarii** - coredump do flash, po restarcie skrot na
  MQTT (zadanie, adres, backtrace).

- **Panel z telefonu** (jak starczy czasu) - pelne sterowanie i podglad
  wszystkiego z telefonu "w razie czego". Punkt wyjscia jest: obecny panel
  (`tools/dashboard/index.html`) ma juz viewport, uklad pod waskie ekrany
  (`@media max-width 760px`) i laczy sie przez `wss://<serwer>/mqtt`
  (Caddy), wiec dziala spoza domu. Do zrobienia: joystick dotykowy do
  jazdy (z deadmanem jak dzis), karty nowych tematow (`tele/safety`,
  `pilot`, `water`, `mission`, `energy`) z przelacznikami `*_en`, przyciski
  awaryjne (stop, `cmd/mission stop`, `cmd/water reset`), dodanie do
  ekranu startowego jako PWA (manifest + ikona).

### Funkcje "doniczkowe"
- **Powiadomienia** - alerty Grafany albo bot Telegram: malo wody, niska
  bateria, utyk, robot offline (tylko serwer).
- **Pogoda w decyzji o podlewaniu** - spadek cisnienia z BME280 albo
  prognoza z internetu -> wstrzymaj podlewanie.
- **Timelapse rosliny** - glowica raz dziennie robi zdjecie na serwer.
- **Ciche godziny** - bez jazdy w nocy / w zadanych porach.

### Dopieszczenie
- **Kalibracja magnetometru 3D** - zdjeta glowica obracana recznie we
  wszystkich osiach (dzis 2D, os Z nieskorygowana). Ma sens tylko wtedy,
  gdy zrodlo pola siedzi na samej glowicy - patrz "Magnetometr glowicy".
- **Samouczenie mapy tagow** - pozycje palikow z odometrii i kursu
  (mini SLAM); dobre na rozdzial "rozszerzenia".
- **Pozostale kalibracje** - offset i strefa martwa sonarow, wzmocnienia
  kanalow VEML, ogniskowa kamery tagow (rozstaw i kola: cmd/track_cal,
  cmd/wheel_cal).
