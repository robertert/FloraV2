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
- [ ] Wgrac RAZEM bryle i glowice. Zmienil sie znak `rate_z_cdps`
      w ATTITUDE (teraz tempo kursu, dodatnie w prawo) i doszla komenda
      `FLORA_CMD_SET_STILL` - stara glowica z nowa bryla da zle wyniki.
- [ ] Tagcam tez wgrac (nowa odleglosc do znacznika).
- [ ] Potwierdzic piny UART oznaczone `TODO potwierdzic`
      (`flora_head_board.h`, `flora_cam_board.h`) - jesli lacza dzialaja,
      zdjac komentarz.

## 1. Glowica: kurs i przechyl (etap 1 i 2)

Stara kalibracja magnetometru jest celowo niewazna (nowy klucz NVS
`mag_cal_ok2`), wiec po wgraniu w logu bedzie "magnetometr BEZ kalibracji".

- [ ] **Offset zyro**: robot nieruchomo, `cmd/head_gyro_cal`. Log:
      "offset wyznaczony", wartosci rzedu ulamka st/s.
- [ ] **Poziom**: `cmd/level_cal auto` (robot sam obraca sie o 180 st).
      Wynik na `head/level_cal`, przekrzywienie plytki kilka stopni max.
- [ ] **Magnetometr**: z dala od samochodow i zbrojonego betonu,
      `cmd/mag_cal auto`. Na `head/mag_cal`: status 0, sektory >= 10/12,
      osie < 1,6, rozrzut < 5%.
- [ ] **Kierunek kursu**: obrot robota o 90 st W PRAWO -> `heading`
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
- [ ] **Kwadrat 5 x 5 m** (tasma na rogach): blad zamkniecia wariantow
      `odom`, `head`, `full` - liczby do pracy. Powtorzyc na trawie.

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

- [ ] **Ogniskowa** (`FLORA_CAM_FOCAL_PX`, dzis 297 z katalogu): znacznik
      na wprost na zmierzonej odleglosci (np. 2000 mm), z logu tagcamu
      bok w px, `focal = bok_px * odl_mm / 300`.
- [ ] **Odleglosc z ukosa** (nowy wzor z pionowych bokow): ten sam
      znacznik obrocony o 45 st wokol pionu, ta sama odleglosc - wynik
      w granicach ~3%. Potem znacznik przy brzegu kadru.
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

- [ ] **Wysokosc obiektywu** nad ziemia miarka, `FLORA_HEAD_CAM_HEIGHT_MM`
      albo `http://<ip>/height?mm=NNN` (wymaga `FLORA_HEAD_CAM_NET_DEFAULT 1`).
      Bez tego sektory ida jako "nie wiem".
- [ ] Przeszkoda w 4 kierunkach na znanej odleglosci - zgodnosc cm
      w `head/obstacles`.
- [ ] Kamera po 30 min pracy w sloncu - temperatura modulu (`head/cam`).

## 7. Czujniki bryly (zrobione - powtorzyc po zmianach sprzetu)

- [x] Wilgotnosc gleby (2026-10-15) - `FLORA_SOIL_RAW_DRY/WET`.
- [ ] Pompka: `cmd/pump 10000` do menzurki, `cmd/pump_cal <ml>`.
- [ ] INA226: staly prad >= 0,3 A z multimetrem, `cmd/ina_cal <A>`, restart.
