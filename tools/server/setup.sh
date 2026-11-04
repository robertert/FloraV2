#!/usr/bin/env bash
# Pierwsze uruchomienie serwera robota. Bezpieczne do ponownego wywolania:
# istniejacy .env i hasla zostaja, dochodzi tylko to, czego brakuje.
#
#   ./setup.sh <domena> <email>
#   ./setup.sh vps-1234abcd.vps.ovh.net jan@example.com
#   ./setup.sh localhost test@example.com      # proba na laptopie
set -euo pipefail
cd "$(dirname "$0")"

MOSQUITTO_IMAGE=eclipse-mosquitto:2.0.20
CADDY_IMAGE=caddy:2.8.4

if [ ! -f .env ]; then
    if [ $# -lt 2 ]; then
        echo "uzycie: $0 <domena> <email do Let's Encrypt>" >&2
        exit 1
    fi
    secret() { openssl rand -hex 16; }
    cat > .env <<EOF
# Wygenerowane przez setup.sh $(date +%F). Nie wrzucaj do repozytorium.
DOMAIN=$1
ACME_EMAIL=$2
# Wersje obrazow flora-* ustawia potem flora-deploy (CI albo recznie).
FLORA_VERSION=latest
INFLUX_PASSWORD=$(secret)
INFLUX_TOKEN=$(secret)$(secret)
GRAFANA_PASSWORD=$(secret)
MQTT_PASS_ROBOT=$(secret)
MQTT_PASS_TELEGRAF=$(secret)
MQTT_PASS_PANEL=$(secret)
EOF
    chmod 600 .env
    echo "utworzono .env z losowymi haslami"
fi
# shellcheck disable=SC1091
set -a; . ./.env; set +a

mkdir -p data/caddy data/mosquitto data/influxdb data/grafana data/ota/hub certs

# Skrot hasla robota dla Caddy'ego (logowanie do /ota/). Liczony przy kazdym
# uruchomieniu, wiec zmiana MQTT_PASS_ROBOT w .env od razu go odswieza.
# Apostrofy obowiazkowe: bcrypt zawiera "$", ktore compose i powloka
# probowalyby rozwinac jak zmienne.
hash=$(docker run --rm "$CADDY_IMAGE" caddy hash-password --plaintext "$MQTT_PASS_ROBOT")
grep -v '^OTA_PASS_HASH=' .env > .env.tmp
echo "OTA_PASS_HASH='$hash'" >> .env.tmp
chmod 600 .env.tmp && mv .env.tmp .env

# Plik hasel brokera, zawsze od nowa z .env - zmiana hasla = edycja .env
# i ponowne ./setup.sh. Hasla sa w nim jako skroty, nie jawnym tekstem.
: > mosquitto/passwd
chmod 600 mosquitto/passwd
for pair in "robot1:$MQTT_PASS_ROBOT" "telegraf:$MQTT_PASS_TELEGRAF" "panel:$MQTT_PASS_PANEL"; do
    docker run --rm -v "$PWD/mosquitto:/m" "$MOSQUITTO_IMAGE" \
        mosquitto_passwd -b /m/passwd "${pair%%:*}" "${pair#*:}"
done
# Kontenery dzialaja jako wlasni uzytkownicy, nie root: mosquitto jako 1883,
# Grafana jako 472. Na Linuksie ich pliki i katalogi danych musza do nich
# nalezec, inaczej proces nie zapisze danych i wylaczy sie zaraz po starcie
# (Grafana: "Restarting (1)"). Na macOS Docker Desktop sam mapuje uprawnienia,
# dlatego proba lokalna tego nie wylapie. InfluxDB poprawia uprawnienia sam.
if [ "$(id -u)" = 0 ]; then
    chown 1883:1883 mosquitto/passwd
    chown -R 1883:1883 data/mosquitto
    chown -R 472:0 data/grafana
fi

# Obrazy flora-* sa w prywatnym rejestrze GitHuba - serwer musi byc
# zalogowany (README, "docker login ghcr.io"), inaczej pull sie nie uda.
docker compose pull --quiet --ignore-buildable

# Najpierw sam Caddy: mosquitto nie wstanie bez certyfikatu, a certyfikat
# pobiera Caddy przy pierwszym starcie.
docker compose up -d caddy
./renew-certs.sh --wait

docker compose up -d
# Plik hasel wczytuje sie tylko przy starcie brokera - po zmianie w .env
# samo "up -d" go nie przeladuje.
docker compose restart mosquitto
echo
echo "gotowe:"
echo "  Grafana  https://$DOMAIN/          admin / $GRAFANA_PASSWORD"
echo "  panel    https://$DOMAIN/panel/    panel / $MQTT_PASS_PANEL"
echo "  robot    mqtts://$DOMAIN:8883      robot1 / $MQTT_PASS_ROBOT"
echo
echo "robot: adres i login sa w firmware (flora_board.h), haslo wpisz raz"
echo "z lokalnego panelu na Macu:"
echo "  cmd/config/mqtt_pass  $MQTT_PASS_ROBOT"
echo "  cmd/reboot"
