# Testy reczne i kalibracje na sprzecie

Lista rzeczy, ktorych nie da sie sprawdzic na komputerze. Kolejnosc ma
znaczenie: kazdy krok zaklada, ze poprzednie przeszly. Tematy MQTT
z prefiksem `flora/robot1/`.

Testy hostowe (przed kazdym wgraniem, wynik `WYNIK: OK`):

```sh
cd components/flora_nav/host
cc -O2 -Wall -I../include -o test test.c ../flora_ekf.c ../flora_sun.c ../flora_sungate.c -lm && ./test
cd ../../flora_heading/host
cc -O2 -Wall -I../include -I../../../node/main -o test test.c ../flora_magcal.c ../flora_hkf.c ../flora_magdist.c -lm && ./test
cd ../../flora_pid/host   # komenda na gorze test.c
```

## 0. Przygotowanie

- [ ] Cofnac zmiany testowe w roboczym drzewie:
      `cfg.enable_magnetometer = false` w `node/main/bringup_head.c`
      i `TEST_BEZ_KAMERY 1` w `node/main/main.c`.
- [x] Wgrac RAZEM bryle i glowice. Zmienil sie znak `rate_z_cdps`
      w ATTITUDE (teraz tempo kursu, dodatnie w prawo) i doszla komenda
      `FLORA_CMD_SET_STILL` - stara glowica z nowa bryla da zle wyniki.
- [ ] Tagcam tez wgrac. Od 21.11.2026 role kamer zamienione: glowica
      wykrywa znaczniki, tagcam przeszkody - wgrac OBA razem z przepieta
      kamera (68 st na glowice, 160 st na tagcam).
- [ ] Potwierdzic piny UART oznaczone `TODO potwierdzic`
      (`flora_head_board.h`, `flora_cam_board.h`) - jesli lacza dzialaja,
      zdjac komentarz.

## 1. Glowica: kurs i przechyl (etap 1 i 2)

Stara kalibracja magnetometru jest celowo niewazna (nowy klucz NVS
`mag_cal_ok3`), wiec po wgraniu w logu bedzie "magnetometr BEZ kalibracji".

- [ ] **Offset zyro**: robot nieruchomo, `cmd/head_gyro_cal`. Log:
      "offset wyznaczony", wartosci rzedu ulamka st/s.
- [x] **Poziom** (15.11: 2,05 / -5,09 st): `cmd/level_cal auto` (robot sam obraca sie o 180 st).
      Wynik na `head/level_cal`, przekrzywienie plytki kilka stopni max.
- [x] **Magnetometr** (15.11 w pokoju: off_z -45,1 uT, dip -66,5; powtorzyc na docelowym miejscu): z dala od samochodow i zbrojonego betonu,
      `cmd/mag_cal auto`. Na `head/mag_cal`: status 0, sektory >= 10/12,
      osie < 1,6, rozrzut < 5%, `off_z_ut` ok. -40 (zero AK09916 tego
      egzemplarza), `ref_dip_deg` ok. -66. Potem `head/mag`:
      `mag_disturbed false`, `field_h_ut` w 15% od `ref_h_ut`.
- [ ] **Pochylosc** (15.11: pierwsze podlozenie 8 st -> kurs < 1 st; reszta z ruchem robota, powtorzyc bez przesuwania): klocek ~10 st pod kola z jednej strony, potem z przodu.
      Kurs zmienia sie o najwyzej ~2 st (bez poprawki z 15.11 bylo ~20).
- [x] **Kierunek kursu**: obrot robota o 90 st W PRAWO -> `heading`
      rosnie o ~90. Jesli maleje - zly znak, NIE jechac dalej.
- [ ] **Zgodnosc z kompasem**: telefon na robocie, 4 kierunki (N, E, S, W),
      roznica stala w granicach kilku stopni (staly offset to montaz -
      zbierze go `b` w EKF).
- [ ] **Kompensacja przechylu**: podlozyc klocek pod jedno kolo
      (~10 st przechylu), potem z przodu/tylu (pochylenie). Kurs nie moze
      sie ruszyc o wiecej niz ~2 st.
- [ ] **Drgania**: jazda po trawie/kostce; `pitch`/`roll` w ATTITUDE
      plynne, bez skokow o kilka stopni (filtr komplementarny, tau 1 s).
- [ ] **ZUPT**: postoj > 2 s po jezdzie. W logu glowicy raz:
      "ZUPT: pierwszy pomiar dryfu na postoju". `gyro_bias_dps`
      w `head/mag` stabilne na postoju.
- [ ] **Recznie obracana glowica przy stojacych kolach** (np. kalibracja
      `manual`): ZUPT nie moze psuc kursu - obrot o 90 st recznie daje
      zmiane kursu o 90 st.
- [ ] **Zaklocenie**: przesunac stalowy przedmiot obok glowicy.
      `head/mag` pokazuje `mag_disturbed`, kurs zostaje (prowadzi zyro),
      po odsunieciu flaga gasnie po ~2 s.

## 2. Bryla: EKF pozycji

- [ ] Po kilku skretach `n_rej` w `tele/pose_head` nie rosnie
      (pojedyncze sztuki OK).
- [ ] `tele/nav_sun`: `gyro_ok: true` gdy glowica dziala;
      `gyro_vs_odo_dps` przy jezdzie prosto ~0, w skrecie w miejscu
      kilka st/s max (wieksza wartosc = poslizg albo zle `FLORA_TRACK_MM`).
- [ ] **Poslizg**: przytrzymac jedno kolo reka (albo podniesc) i krecic
      drugim tak, by odometria widziala obrot o ~90 st, robot stoi.
      `th_deg` w `tele/pose` wraca do prawdy w ciagu ~1 s (przez zyro);
      w wariancie `tele/pose_odom` zostaje bledne - tak ma byc.
- [ ] **Reset pozy**: `cmd/pose_reset` z `"0 0 90"` przy robocie stojacym
      pod innym katem niz poprzednio. Kurs dalej przyjmowany (`n_head`
      rosnie, `n_rej` nie), `b_deg` ustawia sie od nowa.
- [ ] **Nagranie**: `tools/rec/record.py --start`, przejazd, Ctrl+C.
      `tele/rec` `drops: 0`, w podsumowaniu `zgubionych 0`.
- [ ] **Zgodnosc odtworzenia**: `cmd/rec on`, OD RAZU `cmd/pose_reset`
      (replay startuje od zera, a robot z pozy z NVS - reset ustawia obie
      strony tak samo), przejazd, `cmd/rec off`. `replay` bez opcji: poza
      wariantow na koncu jak ostatnia `tele/pose*` z robota, w granicach
      kilku mm (rozne = rozjazd kodu robota i komputera, zglosic).
- [ ] **Zapamietywanie pozy**: przejazd, postoj > 3 s, restart bryly
      (`cmd/reboot`). W logu "poza z NVS: ..." i `tele/pose` od tej pozy,
      nie od (0, 0); po pierwszym kursie z glowicy `n_rej` nie rosnie.
- [ ] **Kwadrat 5 x 5 m** (tasma na rogach), NAGRYWANY: blad zamkniecia
      wariantow `odom`, `head`, `full` z `replay --end "0 0"` - liczby do
      pracy. Powtorzyc na trawie. Potem strojenie na tych nagraniach
      (`--head-floor`, `--deadband`, `--k-wheel`).

## 3. Odometria i naped (z wczesniejszych prac)

Kolejnosc: kola -> rozstaw -> kola jeszcze raz (roznica kol zalezy
od rozstawu i odwrotnie, druga runda dociaga obie). Wymaga glowicy
z kursem (etap 1). Wyniki trafiaja do NVS od razu, bez flashowania;
stan i flagi `*_set` na `config_nav`.

- [ ] **Srednica i roznica kol** na DOCELOWYM robocie, z pelnym
      zbiornikiem, na docelowym podlozu:
  - [ ] zaznaczyc punkt startu (np. srodek osi kol), `cmd/wheel_cal go`
        (3 m prosto; `go 5000` dla dluzszego),
  - [ ] zmierzyc tasma odleglosc start-koniec w linii prostej,
        `cmd/wheel_cal <mm>`,
  - [ ] na `tele/wheel_cal`: `scale_l`/`scale_r` blisko 1, `wheel_ratio`
        w granicach ~0,97-1,03. Powtorzyc 2-3 razy, wyniki maja byc zgodne.
  - [ ] Opcjonalnie z pustym zbiornikiem - pokaze, ile plywa w ciagu dnia.
- [ ] **Rozstaw kol**: `cmd/track_cal` (3 obroty w lewo, 3 w prawo).
      Na `tele/track_cal` `status: ok`, `left_turn_mm` i `right_turn_mm`
      roznia sie < 5%. Na trawie wyjdzie wiecej niz na podlodze - to
      poslizg w skrecie, dlatego kalibrowac na docelowym podlozu.
- [ ] **Recznie** (gdy trzeba): `cmd/config/track_mm`, `wheel_mm`,
      `wheel_ratio`; `-` wraca do stalych z `flora_board.h`.
- [ ] **PID predkosci** (procedura w `hub/main/hub_drive.h`), wykres na
      `tele/pid`:
  - [ ] `cmd/config/drive_mode pid`, nastawy na 0, kola w powietrzu,
  - [ ] `pid_ks` - najmniejsza moc, przy ktorej kolo rusza,
  - [ ] `pid_kff` - `cmd/pid_step "150 3000"`, sam ff daje ~zadana,
  - [ ] `pid_kp` do lekkiego przeregulowania, `pid_ki` usuwa uchyb, `pid_kd` 0,
  - [ ] powtorzyc na ziemi, potem na trawie z pelnym zbiornikiem,
  - [ ] `pid_acc` i `pid_vmax` ustawic, wpisac koncowe wartosci jako
        domyslne `FLORA_PID_*_DEFAULT`.
- [ ] **Straznik utyku**: zablokowac kolo przy jezdzie - zatrzask,
      `cmd/stall_clear` zdejmuje.
- [ ] **Deadman**: zamknac panel w trakcie jazdy - stop po `DRIVE_HOLD_MS`.

## 4. Kamera znacznikow i mapa

- [ ] **Ogniskowa** (`FLORA_HEAD_TAG_FOCAL_PX` w `flora_head_board.h`,
      dzis 297 z katalogu): znacznik 300 mm na wprost na zmierzonej
      odleglosci (np. 2000 mm), z logu glowicy (`znaczniki`) odleglosc,
      `focal = 297 * odl_zmierzona / odl_z_logu`.
- [ ] **Odleglosc z ukosa** (nowy wzor z pionowych bokow): ten sam
      znacznik obrocony o 45 st wokol pionu, ta sama odleglosc - wynik
      w granicach ~3%. Potem znacznik przy brzegu kadru.
- [ ] **Obrot kamery** (`cam_yaw_deg`): znacznik na tasmie od srodka przodu
      robota, kat z logu glowicy (`znaczniki`) ze ZMIENIONYM znakiem do
      `cmd/config/cam_yaw_deg` (np. -6,0 st w logu -> `6`). Log glowicy
      i `head/tags` dalej pokazuja kat surowy, poprawiony idzie do EKF
      i do nagran. 22.11.2026: -6 st po wstepnym ustawieniu glowicy.
- [ ] **Polozenie kamery** wzgledem srodka osi kol: zmierzyc i ustawic
      `cmd/config/cam_x_mm`, `cmd/config/cam_y_mm` (dzis 295 / 0, tymczasowo).
- [ ] **Mapa palikow**: rozstawic paliki, zmierzyc polozenia tasma w ukladzie
      mapy, kazdy `cmd/tag "<id> <x_mm> <y_mm>"` (max 16). Widac je na
      `config_tags/<id>`, liczbe na `config_nav`. Dopiero wtedy wariant
      `full` rozni sie od `head_sun`. Po restarcie mapa zostaje (NVS).
- [ ] Na postoju przed palikiem `n_tag` rosnie, `spos_mm` spada.

## 5. Kompas sloneczny

- [ ] **Kanaly krzyza**: zaslaniac po jednym VEML i sprawdzic, ktory kanal
      spada (`FLORA_HEAD_SUN_CH_*` w `flora_head_board.h`).
- [ ] **Kierunek**: patrzac z gory, E musi byc 90 st W PRAWO od N
      (kurs i azymut slonca sa teraz CW). Kanal N w strone +X glowicy
      (przod robota) - zalozenie z `hub_nav.c`.
- [ ] Sloneczny dzien: `tele/nav_sun` `gate: 0`, `sun_vs_mag_deg`
      w granicach kilku stopni. Staly offset = montaz krzyza, do odjecia.
- [ ] **Miejsce pracy**: `cmd/config/site_lat`, `site_lon`, `mag_decl`
      na wspolrzedne ogrodu (dzis Krakow; deklinacja z kalkulatora NOAA).

## 6. Przeszkody

- [ ] **Wysokosc obiektywu** kamery tagcamu nad ziemia miarka,
      `FLORA_CAM_HEIGHT_MM` w `flora_cam_board.h` i przeflashowac tagcam.
      Bez tego sektory ida jako "nie wiem".
- [ ] Przeszkoda w 4 kierunkach na znanej odleglosci - zgodnosc cm
      w `head/obstacles`.
- [ ] Kamera po 30 min pracy w sloncu - temperatura modulu (`head/cam`).

## 7. Czujniki bryly (zrobione - powtorzyc po zmianach sprzetu)

- [x] Wilgotnosc gleby (2026-10-15) - `FLORA_SOIL_RAW_DRY/WET`.
- [ ] Pompka: `cmd/pump 10000` do menzurki, `cmd/pump_cal <ml>`.
- [ ] INA226: staly prad >= 0,3 A z multimetrem, `cmd/ina_cal <A>`, restart.

## 8. Funkcje autonomiczne (wszystkie domyslnie wylaczone)

Kod nowy (17.11.2026), sprawdzony tylko testami hostowymi. Wlaczac PO
KOLEI, kazda po zaliczeniu poprzedniej. Klucze: `hub/main/hub_autocfg.h`.
Wymaga zaliczonych punktow 1-3 (kurs, EKF, PID).

- [ ] **Po wgraniu nic sie nie zmienia**: `config_auto` pokazuje wszystkie
      `*_en` = 0, reczna jazda, kalibracje i pompka dzialaja jak dotad.

Straznik (`tele/safety`, pole `reasons`: 1 kamera, 2 sonary, 4 przechyl,
8 napiecie, 16 stare dane):
- [ ] Kola w powietrzu. `cmd/config/guard_mask 30` (bez kamery), potem
      `cmd/config/guard_en 1`. Jazda z panelu dziala, `reasons` = 0.
- [ ] Dlon 20 cm przed przednim dalmierzem: jazda w przod staje, `ack/drive`
      daje `ESP_ERR_NOT_ALLOWED`, cofanie i obrot dzialaja. Po zabraniu dloni
      rusza po ok. 0,5 s.
- [ ] To samo z tylnym dalmierzem i cofaniem.
- [ ] Ktory dalmierz jest ktory: A i B to przod, C to tyl (`near_a/b/c`
      w `tele/sonar`). Zamienione = poprawic kolejnosc zlaczy albo
      `hub_telemetry.c`.
- [ ] Przechyl: podniesc robota z jednej strony ponad 20 st - wszystko
      staje (`reasons` 4), po polozeniu wraca.
- [ ] Odlaczyc glowice (`cmd/mast off`): `reasons` 16, jazda z panelu
      zwalnia do 30%.
- [ ] Na ziemi: dojazd do sciany i do kartonu - staje ok. 35 cm przed
      (`g_front_mm`). Dobrac prog do drogi hamowania.
- [ ] Z kamera w dol (po punkcie 6, wysokosc obiektywu wpisana):
      `guard_mask 31`. Bez ramek przeszkod jazda w przod stoi - tak ma byc.
      Przeszkoda w korytarzu jazdy zatrzymuje, z boku nie.

Jazda do punktu (`tele/pilot`), naped w trybie pid:
- [ ] Kola w powietrzu: `cmd/config/pilot_en 1`, `cmd/pose_reset`,
      `cmd/goto "1000 0"` - oba kola do przodu; `cmd/goto stop` zatrzymuje
      w ciagu ok. 0,3 s.
- [ ] Na ziemi: `cmd/pose_reset`, `cmd/goto "1500 0"`, potem
      `cmd/goto "1500 1000"`, `cmd/goto "0 0 0"`. Blad dojazdu z tasmy;
      `state` konczy sie na `done`.
- [ ] `cmd/turn 90`: obrot W LEWO o 90 st (kat mapy rosnie przeciwnie do
      zegara). W prawo = zly znak, nie jechac dalej.
- [ ] Reczna strzalka w panelu w trakcie `goto` przejmuje sterowanie.
- [ ] Przeszkoda na drodze przy wlaczonym strazniku: staje, po 8 s
      `failed / no_progress`.
- [ ] Dobrac `p_vmax`, `p_wmax`, `p_tol_mm` na trawie.

Podlewanie (`tele/water`), po `cmd/pump_cal`:
- [ ] `cmd/config/w_soak_s 60` na czas prob, `cmd/config/water_en 1`.
      Czujnik w powietrzu: po kilku sekundach `dosing`, pompka idzie
      porcjami, razem `w_dose_ml` (menzurka), potem `soak`.
- [ ] Czujnik do wody w trakcie: po wsiakaniu `ok`, pompka stoi.
- [ ] Czujnik caly czas w powietrzu: po 3 dawkach `fault / no_rise`,
      `cmd/water reset` zdejmuje.
- [ ] Pusty zbiornik: `no_water`; po dolaniu wznawia.
- [ ] W trakcie jazdy z panelu nie podlewa (`dry`).
- [ ] Progi `w_dry` / `w_wet` dobrac w doniczce z docelowa ziemia,
      `w_soak_s` wrocic na 600.

Szukanie slonca (`tele/mission`), wymaga straznika, pilota, glowicy i czasu
z sieci:
- [ ] Najpierw sam pomiar: `mission_en 1` BEZ ustawionego obszaru (robot
      wtedy nie ruszy, a `tele/mission` juz idzie). Zapisac `lux` i `contrast`
      w pelnym sloncu i w cieniu. Kontrast w sloncu ma byc wyraznie ponad
      0,25, w cieniu ponizej - inaczej poprawic `contrast_min`.
- [ ] `cmd/area "xmin ymin xmax ymax"` wg mapy palikow, plac bez przeszkod.
- [ ] `cmd/mission seek`: robot jedzie do punktu po stronie przeciwnej do
      slonca, stoi 20 s, jedzie do kolejnych; konczy w najjasniejszym.
      Zaden cel nie lezy blizej niz 400 mm od granicy obszaru.
- [ ] Cien od parawanu: po `s_hold_s` i `s_dwell_s` (na probe skrocic do
      60 / 120) rusza sam i konczy w sloncu.
- [ ] `cmd/mission stop` w trakcie jazdy zatrzymuje i wylacza misje.
- [ ] Reczna jazda wstrzymuje misje na minute (`hold`).

Sen i energia (`tele/energy`):
- [ ] `sleep_en 0`, wlaczony np. `water_en`: `mode` normal, `v_filt` zgodne
      z miernikiem.
- [ ] Zasilacz regulowany zamiast baterii: ponizej 12,6 V `save`, ponizej
      12,2 V `survival`, powrot dopiero 0,3 V wyzej.
- [ ] `cmd/config/e_day_s 60`, `cmd/config/sleep_en 1`: po 2 min od
      ostatniej komendy `tele/sleep`, robot znika na minute i wraca
      (`tele/boot`), poza w `tele/pose` zachowana.
- [ ] Wybudzanie na stale: `mosquitto_pub -r -t flora/robot1/cmd/config/sleep_en -m 0`,
      po powrocie robota skasowac retain (`-r -n`). `e_day_s` wrocic na 0.
- [ ] Klucz MDD10A (po zbudowaniu modulu): `mdd_key_en 1`, pierwsza
      komenda jazdy rusza po ok. 0,1 s, 10 s po zatrzymaniu pobor spada
      o ok. 50 mA.
