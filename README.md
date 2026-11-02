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

## Podglad kamery i zbieranie danych

Glowica w trybie podgladu stawia serwer HTTP i nadaje strumien MJPEG. Adres
pojawia sie w logu po starcie (`./dev.sh node monitor`):

```
http://<ip-glowicy>/      zywy obraz w przegladarce
http://<ip-glowicy>/jpg   jedna klatka, np. curl -o kadr.jpg http://<ip>/jpg
```

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
prosto w dol. Wynik jest w pierscieniach i pikselach, nie w centymetrach -
zamiana na odleglosc wymaga wysokosci kamery nad ziemia i kalibracji miarka,
czyli docelowego montazu. Sesje lezą w `tools/vision/data/` i NIE sa w repo.

## Stan

Bryla ma sterowniki napedu, enkoderow i pomiaru pradu, telemetrie MQTT i
zdalne sterowanie z panelu. Glowica ma petle czujnikow (fuzja kursu, kompas
sloneczny, BME280), oba lacza UART i kamere OV5640 z obiektywem 160 stopni
patrzaca w dol z masztu, na razie w trybie podgladu. Wykrywanie przeszkod jest
w planie: kontrakt `FLORA_MSG_OBSTACLES` i publikacja na MQTT juz istnieja,
brakuje producenta w glowicy i weta napedu w bryle.

**Uwaga przy uruchamianiu testu napedu: kola musza byc w powietrzu.**
