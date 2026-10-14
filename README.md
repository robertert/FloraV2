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

## Stan

Bryla ma sterowniki napedu, enkoderow i pomiaru pradu oraz test rozruchowy
w `hub/main/main.c`. Glowica to na razie pusty szkielet - czeka na plytke
ESP32-S3 z gniazdem kamery.

**Uwaga przy uruchamianiu testu napedu: kola musza byc w powietrzu.**
