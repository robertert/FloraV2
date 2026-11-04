# Serwer robota (VPS)

Broker MQTT z TLS i haslami, historia (Telegraf + InfluxDB + Grafana) i panel
debugowy, wszystko za HTTPS. Konfiguracja jest wbudowana w obrazy `flora-*`,
ktore buduje CI z tagu git; na serwerze leza tylko sekrety i dane.

| adres | co |
|---|---|
| `https://<domena>/` | Grafana |
| `https://<domena>/panel/` | panel debugowy (login `panel`) |
| `https://<domena>/ota/hub/` | firmware dla OTA (tylko login `robot1`) |
| `mqtts://<domena>:8883` | robot (login `robot1`) |

```
git tag server-v1.2.0 && git push origin server-v1.2.0
   -> GitHub Actions buduje flora-caddy, -mosquitto, -telegraf, -grafana
   -> wypycha do ghcr.io/robertert/flora-*:v1.2.0   (prefiks "server-" obciety)
   -> ssh deploy@VPS v1.2.0  (klucz moze uruchomic TYLKO flora-deploy)
   -> flora-deploy: FLORA_VERSION=v1.2.0 w .env, pull, up -d, test zdrowia,
      przy awarii powrot do poprzedniej wersji
```

## Pierwsze postawienie (Ubuntu 24.04)

Zaklada serwer zabezpieczony jak w rozmowie: logowanie tylko kluczem jako
`ubuntu`, `ufw` z otwartymi 22, 80, 443 i 8883, Docker z oficjalnej
instrukcji (https://docs.docker.com/engine/install/ubuntu/).

Porty wystawione przez Dockera omijaja `ufw`. Dlatego InfluxDB i zwykle MQTT
nie maja w `docker-compose.yml` zadnych `ports:`.

1. **Katalog i pliki serwera** (z laptopa, bez lokalnych sekretow):
   ```sh
   ssh flora 'sudo mkdir -p /opt/flora/tools && sudo chown ubuntu: /opt/flora/tools'
   rsync -av --exclude .env --exclude data --exclude certs \
         --exclude mosquitto/passwd --exclude images \
         tools/server flora:/opt/flora/tools/
   ```
2. **Dostep serwera do obrazow.** Rejestr obrazow jest prywatny. Na GitHubie:
   Settings > Developer settings > Personal access tokens > **Tokens (classic)**,
   uprawnienie tylko `read:packages`. Potem na serwerze:
   ```sh
   sudo docker login ghcr.io -u robertert     # haslo = ten token
   ```
3. **Pierwsze obrazy:** wypchnij pierwszy tag (`git tag server-v0.1.0 && git push
   origin server-v0.1.0`). Zadanie `images` zbuduje obrazy, a `deploy` sie nie uda,
   bo serwer nie ma jeszcze `.env` - to oczekiwane przy pierwszym razie.
4. **Setup na serwerze:**
   ```sh
   cd /opt/flora/tools/server
   sudo ./setup.sh vps-xxxxxxx.vps.ovh.net twoj@email.pl
   ```
   Generuje losowe hasla w `.env`, zaklada konta w brokerze, czeka na
   certyfikat z Let's Encrypt i stawia reszte. Na koncu wypisuje hasla i
   komendy do przelaczenia robota. `sudo`, bo pliki hasel i certyfikatu
   musza nalezec do uzytkownika mosquitto (1883).
5. **Odnawianie certyfikatu dla brokera** (Caddy odnawia swoj sam,
   mosquitto trzeba powiadomic):
   ```sh
   echo '0 4 * * 1 /opt/flora/tools/server/renew-certs.sh >> /var/log/flora-certs.log 2>&1' \
     | sudo crontab -
   sudo crontab -l                                # sprawdzenie
   sudo /opt/flora/tools/server/renew-certs.sh    # reczny test
   ```
   Uwaga: `crontab -` ZASTEPUJE caly crontab roota. Na swiezym serwerze jest
   pusty; jesli juz cos w nim jest, dopisz linie przez `sudo crontab -e`.
6. **CI** - sekcja nizej. Od tej chwili kazdy tag wdraza sie sam.

## CI: wdrozenie z tagu

Workflow: `.github/workflows/server.yml`, uruchamiany tylko tagami
`server-vX.Y.Z`. Prefiks oddziela wersje serwera od wersji firmware, ktore
dostana wlasne tagi przy OTA. Serwer przyjmuje go przez osobnego
uzytkownika `deploy`, ktory nie ma powloki - jego klucz moze tylko wywolac
`flora-deploy` z tagiem.

**Na serwerze:**
```sh
cd /opt/flora/tools/server
sudo install -o root -g root -m 755 flora-deploy /usr/local/bin/flora-deploy

sudo adduser --disabled-password --gecos "" deploy
# deploy moze przez sudo uruchomic TYLKO flora-deploy, nic wiecej
echo 'deploy ALL=(root) NOPASSWD: /usr/local/bin/flora-deploy' \
  | sudo tee /etc/sudoers.d/flora-deploy
sudo chmod 440 /etc/sudoers.d/flora-deploy && sudo visudo -c
```

**Klucz CI** (na Macu, osobny, bez hasla - uzywa go automat):
```sh
ssh-keygen -t ed25519 -f ~/.ssh/flora_ci -N "" -C flora-ci
```
Na serwerze jedna linia w `authorized_keys` uzytkownika `deploy`, z kluczem
publicznym z `~/.ssh/flora_ci.pub` w miejscu `ssh-ed25519 AAAA... flora-ci`
(`adduser` nie tworzy katalogu `.ssh`, stad `install -d`):
```sh
sudo install -d -m 700 -o deploy -g deploy /home/deploy/.ssh
sudo tee /home/deploy/.ssh/authorized_keys > /dev/null <<'EOF'
command="sudo /usr/local/bin/flora-deploy \"$SSH_ORIGINAL_COMMAND\"",restrict ssh-ed25519 AAAA... flora-ci
EOF
sudo chown deploy:deploy /home/deploy/.ssh/authorized_keys
sudo chmod 600 /home/deploy/.ssh/authorized_keys
```
`'EOF'` w apostrofach, inaczej powloka podstawi pusty `$SSH_ORIGINAL_COMMAND`
od razu przy zapisie. `command=` wymusza komende niezaleznie od tego, co
wysle klient, a `restrict` wylacza terminal, przekierowania portow i reszte.

Test z Maca - ma odpowiedziec `flora-deploy: zly tag "test"`:
```sh
ssh -i ~/.ssh/flora_ci -o IdentitiesOnly=yes deploy@<adres> test
```

**Na GitHubie:** Settings > Environments > New environment `production`
(mozna tam wlaczyc "Required reviewers", wtedy kazde wdrozenie czeka na
klikniecie). W nim sekrety:

| sekret | wartosc |
|---|---|
| `DEPLOY_HOST` | adres serwera |
| `DEPLOY_SSH_KEY` | zawartosc `~/.ssh/flora_ci` (prywatny) |
| `DEPLOY_KNOWN_HOSTS` | wynik `ssh-keyscan -t ed25519 <adres>` |

Odcisk z `ssh-keyscan` porownaj przed wklejeniem z tym na serwerze
(`ssh-keygen -lf /etc/ssh/ssh_host_ed25519_key.pub` vs
`ssh-keyscan -t ed25519 <adres> | ssh-keygen -lf -`) - po to jest w sekrecie,
zeby CI nie zaufalo podstawionemu serwerowi.

**Recznie, na serwerze:**
```sh
sudo flora-deploy v1.1.0 < /dev/null    # powrot do starszej wersji (bez "server-")
grep FLORA_VERSION /opt/flora/tools/server/.env   # co teraz dziala
```

## Robot a serwer

Adres serwera i login `robot1` sa wkompilowane w firmware
(`FLORA_MQTT_URI_DEFAULT` w `components/flora_board/include/flora_board.h`).
Haslo NIE jest - robot czyta je z NVS. Wpisuje sie je raz, z lokalnego panelu
na Macu, gdy robot nie ma jeszcze hasla i laczy sie z brokerem lokalnym:

```sh
mosquitto_pub -t flora/robot1/cmd/config/mqtt_pass -m <MQTT_PASS_ROBOT z .env>
mosquitto_pub -t flora/robot1/cmd/reboot -m 1
```

Kolejnosc polaczen przy starcie: ustawienia z NVS, potem serwer z firmware,
na koncu broker na Macu (bez hasel). Robot bez dzialajacego serwera albo
hasla wyladuje wiec na brokerze lokalnym i da sie go naprawic z panelu.
W najgorszym razie start sieci trwa wtedy do ok. 1,5 min (trzy proby).

Zmiana adresu serwera bez flashowania: `cmd/config/mqtt_uri`, potem `reboot`.
Powrot do wartosci z firmware: ten sam klucz ustawiony na `-`.

## OTA: firmware bryly

```
git tag hub-v1.2.0 && git push origin hub-v1.2.0
   -> GitHub Actions (.github/workflows/hub.yml): budowa w espressif/idf:v6.1,
      sprawdzenie, ze wersja w obrazie = tag
   -> ssh deploy@VPS "ota hub-v1.2.0" < hub.bin
   -> flora-deploy: data/ota/hub/hub-v1.2.0.bin + .json (suma liczona tutaj),
      latest.json, zostaje 5 ostatnich wersji
panel: "aktualizuj do najnowszej" -> robot pobiera, sprawdza sume, restartuje
```

CI tylko wrzuca plik - robot aktualizuje sie wylacznie na komende z panelu
(`cmd/ota`). Pliki serwuje Caddy pod `/ota/` z loginem `robot1` i jego haslem
(skrot `OTA_PASS_HASH` w `.env`, liczony z `MQTT_PASS_ROBOT`): binarka ma w
sobie domyslne haslo WiFi, wiec nie moze byc publiczna.

Co jest na serwerze: `ls /opt/flora/tools/server/data/ota/hub/`,
najnowsza: `cat .../latest.json`.

**Jednorazowo po zmianie flora-deploy** (CI celowo go nie aktualizuje, bo
dziala jako root):
```sh
rsync -av tools/server/flora-deploy flora:/tmp/
ssh flora 'sudo install -o root -g root -m 755 /tmp/flora-deploy /usr/local/bin/flora-deploy'
```

## Konta brokera (`mosquitto/acl`)

| login | uprawnienia |
|---|---|
| `robot1` | tylko `flora/robot1/#` |
| `telegraf` | tylko odczyt `flora/#` |
| `panel` | wszystko, lacznie z komendami jazdy i pompki |

Zmiana hasla: edycja `.env`, potem `sudo ./setup.sh` (bez argumentow)
przebuduje plik hasel i zrestartuje broker. Zmiana uprawnien (`acl`) to
zmiana w repo i nowy tag.

## Proba na laptopie

Obrazy budowane lokalnie zamiast z rejestru, certyfikat z wewnetrznego CA
Caddy'ego zamiast Let's Encrypt (wazny tylko 12 h - potem `./renew-certs.sh`):

```sh
export COMPOSE_FILE=docker-compose.yml:docker-compose.build.yml
docker compose build
./setup.sh localhost test@example.com
mosquitto_pub -h localhost -p 8883 \
  --cafile data/caddy/caddy/pki/authorities/local/root.crt \
  -u robot1 -P <haslo z .env> -t flora/robot1/tele/sys -m '{}'
```

Sprzatanie po probie (USUWA dane i hasla testowe):
```sh
docker compose down && rm -rf .env data certs mosquitto/passwd
```
