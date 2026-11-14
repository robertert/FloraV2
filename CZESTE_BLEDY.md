# Czeste bledy i jak je rozpoznac

Zebrane przy uruchamianiu glowicy (listopad 2026). Najpierw objaw z logu,
potem co to znaczy i co sprawdzic.

## Magistrala I2C glowicy (SDA=21, SCL=47)

### Timeout na kazdym adresie w skanie (`probe device timeout`)
Ktoras linia wisi nisko - przy pustej magistrali bylby zwykly NACK, nie
timeout. Patrz na pierwsza linie po starcie:

    glowica: linie I2C w spoczynku: SDA(21)=? SCL(47)=?

- `1/1` - linie zdrowe, problem gdzie indziej.
- `SDA=0` - uklad zawieszony w polowie transakcji (ESP sie zresetowal,
  czujnik nie). Firmware sam probuje 9 impulsow SCL + STOP i pisze
  `po odblokowaniu: ...`. Zwykle pomaga.
- `SCL=0` - zwarcie do masy albo niezasilony uklad na magistrali. Zegarem
  sie tego nie odblokuje. Miernik: GPIO47 <-> GND przy wylaczonym zasilaniu,
  powinno byc kilka kOhm.

### Magistrala pada w chwili wyboru kanalu muxa
Skan glowny przechodzi, a po `wybor kanalu N` / `zapis maski` wszystko
staje. Zwarcie jest ZA muxem, na tym kanale (zlacze, przewod, modul VEML).
Mux laczy kanal z glowna magistrala, wiec jeden zly czujnik kladzie
wszystko, lacznie z BME i ICM.

### Mux pamieta kanal przez reset ESP
RST muxa jest niepodlaczony na PCB, wiec reset ESP go nie resetuje. Jesli
ostatnio wybrany kanal byl zwarty, SCL wisi nisko od samego startu.
Pomaga tylko PELNE odciecie zasilania. Uwaga: obie plytki sa na wspolnej
szynie 5V, wiec USB w tagcamie zasila tez glowice - przepinanie kabla
miedzy plytkami NIE robi resetu zasilania.

Poprawka docelowa: RST muxa na wolny GPIO (patrz TODO.md).

### Piny SCL obok GND - tam szukac zwarc
| Miejsce | SCL | sasiad |
|---|---|---|
| J2 (VEML N, kanal 0) | pin 3 | pin 2 = GND |
| ICM (U3) | pin 4 | pin 3 = GND |
| J5 (BME) | pin 3 | pin 2 = GND |

Krzywo wlozona wtyczka, wygiety pin albo zyla wystajaca z zacisku. Objaw
"raz dziala, raz nie" po kazdym przepieciu = styk, nie uszkodzony uklad.

### Niezasilony czujnik kladzie magistrale
Uklad I2C bez zasilania sciaga SDA/SCL do 0 V przez diody zabezpieczajace.
VEML-e E/S/W maja zasilanie z osobnych 2-pinowych zlaczy (J7/J9/J10),
N bierze je z J2 - po zamianie czujnikow sprawdzic oba zlacza.

### Uszkodzony modul VEML
Jeden modul byl zepsuty i zostal wymieniony. Test: zamien wtyczki dwoch
czujnikow - jesli blad idzie za modulem, winny modul; jesli zostaje na
kanale, winne zlacze/lut przy muxie.

### Dziwne adresy w pierwszym skanie (0x10, 0x0C)
To nie blad. 0x10 = VEML widoczny, bo mux zostal z otwartym kanalem z
poprzedniego uruchomienia. 0x0C = magnetometr, bo ICM zostal w trybie
przelotowym. Init muxa i ICM to porzadkuje.

### ICM pod 0x69, nie 0x68
Adafruit ICM-20948 ma domyslnie 0x69 (zworka ADR -> 0x68). Kod probuje
obu. W IDF 6.1 NACK pod zlym adresem wraca jako `ESP_ERR_INVALID_RESPONSE`,
nie `ESP_ERR_NOT_FOUND` - fallback musi reagowac na kazdy blad.

Piny AD/AC na plytce Adafruit to pomocnicza magistrala 1,8 V - nic do
nich nie podlaczac.

## Lacze glowica <-> tagcam

### Brak `modul znacznikow: polaczony` w logu glowicy
`flora_link` nie ma wlasnego heartbeatu: lacze jest "nawiazane" dopiero po
odebraniu ramki. Tagcam wysyla `CAM_STATUS` co 500 ms z osobnego zadania,
tez gdy kamera nie wstala (`camera_ok = 0`). Jesli polaczenia brak:
1. czy tagcam ma wgrany aktualny firmware,
2. czy ma zasilanie (wspolna szyna 5V),
3. dopiero potem okablowanie: glowica 38 -> tagcam 39 i odwrotnie.

Sprawdzac od strony GLOWICY - glowica w gore nic nie wysyla, wiec tagcam
u siebie "nawiazane" nie pokaze.

### `Detected camera not supported` w tagcamie
Obsluga czujnikow jest wlaczona w sdkconfig - to tasma (krzywo, stykami w
zla strone) albo brak kamery w gniezdzie.

## Siec glowicy (WiFi, MQTT, OTA)

- **`headstatus` = offline, a glowica dziala** - nie ma hasla brokera w NVS
  (log `brak hasla brokera w NVS`) albo nie widzi WiFi. Haslo wpisuje sie
  raz: glowica bez hasla trafia na broker na Macu, tam
  `headcmd/config/mqtt_pass`, potem `headcmd/reboot`.
- **Komenda do glowicy bez odpowiedzi** - poszla na `cmd/...` (to bryla)
  zamiast `headcmd/...`. Odpowiedz glowicy jest w `headack/...`.
- **Po zmianie `node/partitions.csv` OTA nie wystarczy** - tablice partycji
  wgrywa sie tylko kablem. NVS jest w tym samym miejscu, kalibracje zostaja.
- **USB w glowicy przy zasilaniu z bryly** - dwa zrodla na szynie 5V, plytki
  migaja. Odepnij zyle 5V w J1 (GND, TX, RX zostaja) albo uzyj logow przez MQTT.
- **OTA glowicy: `HTTP 404 (brak wersji)`** - na serwerze nie ma jeszcze
  zadnego `head-v...` albo `flora-deploy` jest stary (instaluje sie go recznie,
  patrz `tools/server/README.md`).

## Wgrywanie i porty

- **`port is busy`** - otwarty monitor. Zamknij `Ctrl+]` przed `flash`.
- **Mac widzi tylko jedna plytke** - kabel tylko do ladowania albo hub.
  Wspolna szyna 5V i tak zasila obie plytki z jednego USB.
- **Bledy clang w IDE** (`-mlongcalls`, `hal.h not found`) - szum, clang nie
  zna flag GCC dla Xtensa. Liczy sie wynik `./dev.sh <projekt> build`.

## Odczyty, ktore wygladaja zle, a nie sa awaria

- **Duzy offset zyroskopu w tescie rozruchowym** (dziesiatki st/s) i
  przyspieszenie w X/Y daleko od 0 - glowica sie ruszala albo byla
  przechylona w czasie kalibracji. Test robic na nieruchomej glowicy.
- **Pole magnetyczne ~90 uT** zamiast ~50 - zelazo albo przewody w poblizu.
  Kalibracja `cmd/mag_cal` na zmontowanej glowicy.
