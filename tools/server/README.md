# Serwer robota (VPS)

Broker MQTT z TLS i haslami, historia (Telegraf + InfluxDB + Grafana) i panel
debugowy, wszystko za HTTPS. Konfiguracja jest wbudowana w obrazy `flora-*`,
ktore buduje CI z tagu git; na serwerze leza tylko sekrety i dane.

| adres | co |
|---|---|
| `https://<domena>/` | Grafana |
| `https://<domena>/panel/` | panel debugowy (login `panel`) |
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
   sudo crontab -e
   0 4 * * 1  /opt/flora/tools/server/renew-certs.sh >> /var/log/flora-certs.log 2>&1
   ```
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
Na serwerze w `/home/deploy/.ssh/authorized_keys` (katalog 700, plik 600,
wlasciciel `deploy`) jedna linia, z kluczem publicznym z `~/.ssh/flora_ci.pub`:
```
command="sudo /usr/local/bin/flora-deploy \"$SSH_ORIGINAL_COMMAND\"",restrict ssh-ed25519 AAAA... flora-ci
```
`command=` wymusza komende niezaleznie od tego, co wysle klient, a
`restrict` wylacza terminal, przekierowania portow i reszte.

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

## Przelaczenie robota na serwer

Z panelu na laptopie, dopoki robot jest jeszcze podlaczony do lokalnego
brokera (karta "Ustawienia"), albo z terminala:

```sh
B=flora/robot1/cmd
mosquitto_pub -t $B/config/mqtt_user -m robot1
mosquitto_pub -t $B/config/mqtt_pass -m <MQTT_PASS_ROBOT z .env>
mosquitto_pub -t $B/config/mqtt_uri  -m mqtts://<domena>:8883
mosquitto_pub -t $B/reboot -m 1
```

Gdy robot nie polaczy sie z serwerem, wraca do ustawien wkompilowanych,
czyli do brokera na laptopie. Powrot recznie: `mqtt_uri`, `mqtt_user` i
`mqtt_pass` ustawione na `-`, potem `reboot`.

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
