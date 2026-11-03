# Doniczka-robot

Autonomiczna, solarna, mobilna doniczka: jezdzi po trawniku za sloncem, mierzy
warunki i sama podlewa rosline. Praca inzynierska AGH, termin styczen 2027.

## Uklad repozytorium

Jedno repo, dwie jednostki polaczone UART-em przez maszt, wspolne sterowniki.

```
components/        sterowniki dzielone przez obie jednostki
  flora_board/     mapa pinow bryly + przeliczniki odometrii
  flora_motors/    naped na Cytron MDD10A (PWM + DIR)
  flora_encoders/  enkodery kwadraturowe na PCNT
  flora_ina226/    pomiar pradu i napiecia szyny 12V
  flora_msgs/      kontrakt wiadomosci UART (wspolny dla obu stron)
hub/               BRYLA: ESP32-S3 DevKitC-1 N16R8
                   naped, odometria, zasilanie, podlewanie, telemetria
node/              GLOWICA MASZTU: ESP32-S3 z gniazdem kamery (do kupienia)
                   kompas sloneczny, BME280, IMU, czujnik deszczu
```

Nazwy katalogow `hub` i `node` pochodza z wczesniejszej wersji projektu.
`hub` to jednostka DOLNA, `node` to GLOWICA.

Wspolne komponenty trafiaja do obu projektow przez `EXTRA_COMPONENT_DIRS`
ustawione w `CMakeLists.txt` kazdej jednostki.

## Wymagania

ESP-IDF **v6.1**, zainstalowane przez EIM w `~/.espressif/v6.1/esp-idf`.
Obie jednostki to ESP32-S3.

## Praca z projektem

`./dev.sh` sam wciaga srodowisko ESP-IDF i wykrywa port, wiec nie trzeba
niczego sourcowac:

```sh
./dev.sh hub build      # zbuduj bryle
./dev.sh hub fm         # build + flash + monitor, najczestszy cykl
./dev.sh hub monitor    # sama konsola, wyjscie Ctrl+]
./dev.sh hub menuconfig
./dev.sh ports          # co widac na USB
./dev.sh help           # pelna lista komend
```

Nieznane komendy leca wprost do `idf.py`, np. `./dev.sh hub partition-table`.

Przy dwoch podlaczonych plytkach trzeba wskazac port:
`FLORA_PORT=/dev/cu.usbmodem101 ./dev.sh hub fm`.

## Debug przez MQTT

Bryla publikuje wszystko, co do niej dociera (wlasne czujniki i wiadomosci
glowicy), pod `flora/robot1/tele/*` i `flora/robot1/head/*`, a komendy
przyjmuje z `flora/robot1/cmd/*` (opis w `hub/main/hub_remote.h`).

1. Broker na Macu (`brew install mosquitto`):
   `mosquitto -c tools/dashboard/mosquitto.conf -v`
2. Adres brokera dla robota to IP Maca w tej samej sieci:
   `FLORA_MQTT_URI_DEFAULT` w `flora_board.h` albo klucz `mqtt_uri` w NVS.
3. Panel: otworz `tools/dashboard/index.html` w przegladarce, adres
   `ws://<ip-maca>:9001`. Strzalki na klawiaturze jada, spacja to STOP.
4. Z terminala: `mosquitto_sub -t 'flora/robot1/#' -v`,
   `mosquitto_pub -t flora/robot1/cmd/pump -m on`.

Naped z panelu staje sam po 1,5 s bez odswiezenia komendy (dead-man),
wiec zerwane WiFi nie zostawi jadacego robota.

**Do jakiego brokera laczy sie robot** - po kolei, az ktorys odpowie:

1. ustawienia z NVS (zmieniane zdalnie, patrz nizej),
2. serwer wkompilowany w firmware (`FLORA_MQTT_URI_DEFAULT` w `flora_board.h`,
   TLS, login `robot1`, haslo z NVS - w kodzie go nie ma),
3. broker na Macu po nazwie mDNS (`Roberts-MacBook-Pro.local`), bez hasel.

Haslo brokera trzeba wpisac raz (`cmd/config/mqtt_pass`, z lokalnego panelu,
gdy robot jest na brokerze Maca). Przezywa zwykle flashowanie, znika dopiero
po `erase-flash`.

**Ustawienia zdalnie** (karta "Ustawienia" w panelu albo z terminala), zapis
do NVS, siec i broker dzialaja po restarcie:

```sh
mosquitto_pub -t flora/robot1/cmd/config/mqtt_uri -m mqtt://172.20.10.14:1883
mosquitto_pub -t flora/robot1/cmd/reboot -m 1
```

Klucze: `wifi_ssid`, `wifi_pass`, `mqtt_uri`, `mqtt_user`, `mqtt_pass`,
`soil_dry`, `soil_wet`; wartosc `-` kasuje ustawienie. Zly adres albo haslo
w NVS nie wymaga podlaczania kabla - robot przejdzie do kolejnych prob.

**Diagnostyka zdalna** (`hub/main/hub_diag.c`):

- `tele/log` - ostrzezenia i bledy z logu ESP (poziomy W i E). Kolejka
  trzyma je takze bez polaczenia, wiec bledy z uruchamiania i z przerwy w
  sieci wychodza po polaczeniu. Konsola USB dostaje wszystko jak dotad.
- `tele/boot` (retain) - wersja firmware, data budowania, powod ostatniego
  restartu (`panic`, `task_wdt`, `brownout`... = awaria) i licznik startow.

Wersja firmware pochodzi z tagu `hub-vX.Y.Z` (`git tag hub-v0.1.0`); bez
niego jest skrot commita. Tagi `server-v...` sa dla serwera i do firmware
nie trafiaja.

**Historia** (Telegraf + InfluxDB + Grafana w Dockerze):

```sh
cd tools/telemetry && docker compose up -d
```

Grafana pod http://localhost:3001 (admin / flora-admin), dashboard
"Flora robot". Wersja na serwer (broker z TLS i haslami, Grafana i
panel za HTTPS): `tools/server/README.md`. Hasla domyslne sa tylko do laptopa - na serwer ustaw wlasne
w `tools/telemetry/.env` (`INFLUX_PASSWORD`, `INFLUX_TOKEN`, `GRAFANA_PASSWORD`).

## Podglad kamery i zbieranie danych

Glowica w trybie podgladu stawia serwer HTTP i nadaje strumien MJPEG. Adres
pojawia sie w logu po starcie (`./dev.sh node monitor`):

```
http://<ip-glowicy>/      zywy obraz w przegladarce
http://<ip-glowicy>/jpg   jedna klatka, np. curl -o kadr.jpg http://<ip>/jpg
```

**Domyslnie obraz NIE jest dostepny przez siec.** Glowica wstaje bez WiFi i
bez serwera HTTP: obrazu nikt nie oglada, a jedynym odbiorca jest detektor
przeszkod. Radio na solarnej glowicy to najwiekszy pojedynczy pobor, a serwer
HTTP ma wyzszy priorytet niz wizja i walczy z nia o kamere (zmierzone: przy
otwartej stronie czas na klatke skakal z 233 ms na 1728 ms). Linker wyrzuca
wtedy caly stos sieciowy, wiec obraz aplikacji schodzi z 1034 kB do 398 kB.

Podglad wlacza sie przez `FLORA_HEAD_CAM_NET_DEFAULT 1` w `flora_head_board.h`
i przeflashowanie - do wycelowania kamery, zebrania klatek albo ustawienia jej
wysokosci. Wtedy dopiero dziala ponizsze:

Kamera przyjmuje JEDEN format na uruchomienie, wiec glowica ma dwa tryby i
przelacza je przez `http://<ip>/mode?v=jpeg` albo `?v=raw` (zapis do NVS i
restart, okolo dwoch sekund):

| tryb | do czego | endpointy |
|---|---|---|
| `jpeg` | celowanie kamery, ocena ostrosci | `/stream`, `/jpg` |
| `raw` | zbieranie zbioru, 320x240 RGB565 | `/raw` |

Zbior zbieramy w trybie **raw**, nie JPEG: kompresja podprobkowuje chrominancje
4:2:0 i doklada artefakty blokow, czyli psuje dokladnie ten sygnal, na ktorym
stoi wykrywanie przeszkod. Prototyp na Macu musi widziec te same bajty, ktore
zobaczy pozniej algorytm na ESP.

## Narzedzia wizyjne (`tools/vision/`)

Zadnych zaleznosci z pip - sama biblioteka standardowa Pythona i `mosquitto_sub`
z brew.

```sh
# zbieranie: klatki + migawka telemetrii z MQTT do kazdej
collect.py --host 192.168.1.42 --surface kostka --light slonce --raw
collect.py --host 192.168.1.42 --surface beton --light cien --raw --no-mqtt

# prototyp wykrywania: nakladka PNG + linia JSON na klatke
ground.py data/20261101-kostka-slonce/*.rgb565

# syntetyczne klatki o znanej odpowiedzi, do testow bez kamery
synth.py /tmp/test.rgb565 --joints --shadow 90 --obstacle -50,80,24
```

`ground.py` pracuje na siatce POLARNEJ wokol srodka kadru, bo kamera patrzy
prosto w dol: PIERSCIEN to odleglosc (0 najblizej), WYCINEK to kierunek
(16 sztuk, od lewej do prawej, jak `sectors[]` w kontrakcie). W wyniku `12`
znaczy czysto do konca zasiegu, a `-1` nieznane. Przelacznik `--grid` rysuje
granice i numery wycinkow wprost na kadrze - bez tego trzeba liczyc wycinki
palcem po ekranie. Sesje lezą w `tools/vision/data/` i NIE sa w repo.

## Wykrywanie przeszkod

Glowica liczy 16 odleglosci i wysyla je w dol jako `FLORA_MSG_OBSTACLES`
(`components/flora_ground` plus `node/main/head_obstacles.c`, 4 Hz). Bryla
publikuje je na `head/obstacles`.

Nie uczymy sie, jak wyglada przeszkoda - w kazdej klatce od nowa uczymy sie,
jak wyglada PODLOZE tuz przy kamerze, i szukamy, gdzie przestaje tak wygladac.
Dlatego to samo dziala na kostce, betonie i trawie bez zadnego uczenia.
Porownanie idzie po BARWIE, nie po jasnosci, zeby cien nie byl przeszkoda.

Zanim sektory zaczna niesc centymetry, trzeba zmierzyc wysokosc obiektywu nad
ziemia i wpisac ja w `FLORA_HEAD_CAM_HEIGHT_MM`. Bez tego ida jako "nie wiem".

Rdzen algorytmu nie zalezy od ESP-IDF i kompiluje sie zwyklym gcc, co sluzy
do sprawdzania, czy wersja na robocie liczy to samo co prototyp:

```sh
cd components/flora_ground/host
cc -O2 -I../include -o runner runner.c ../flora_ground.c -lm
./runner --r-in 2 ../../../tools/vision/data/<sesja>/*.rgb565
```

Po KAZDEJ zmianie algorytmu porownaj wynik z `tools/vision/ground.py` na tych
samych klatkach. Uwaga: harness NIE wykryje bledow pamieci, bo na komputerze
stos ma megabajty - tak przeszlo przepelnienie stosu, ktore na ESP wywalalo
zadanie po dwoch sekundach.

## Stan

Bryla ma sterowniki napedu, enkoderow i pomiaru pradu, telemetrie MQTT i
zdalne sterowanie z panelu. Glowica ma petle czujnikow (fuzja kursu, kompas
sloneczny, BME280), oba lacza UART i kamere OV5640 z obiektywem 160 stopni
patrzaca w dol z masztu, na razie w trybie podgladu. Wykrywanie przeszkod jest
w planie: kontrakt `FLORA_MSG_OBSTACLES` i publikacja na MQTT juz istnieja,
brakuje producenta w glowicy i weta napedu w bryle.

**Uwaga przy uruchamianiu testu napedu: kola musza byc w powietrzu.**
