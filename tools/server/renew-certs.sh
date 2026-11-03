#!/usr/bin/env bash
# Kopiuje certyfikat pobrany przez Caddy'ego do mosquitto i restartuje
# broker, gdy certyfikat sie zmienil.
#
# Caddy odnawia certyfikat sam (Let's Encrypt wydaje go na 90 dni), ale
# o tym, ze mosquitto ma wczytac nowy plik, nikt mu nie powie. Dlatego
# ten skrypt idzie do crona, np. raz w tygodniu:
#   0 4 * * 1  /opt/flora/tools/server/renew-certs.sh
#
#   --wait   czekaj do 2 min na pierwszy certyfikat (uzywa setup.sh)
set -euo pipefail
cd "$(dirname "$0")"
# shellcheck disable=SC1091
set -a; . ./.env; set +a

# Caddy trzyma dane w /data/caddy, czyli na hoscie data/caddy/caddy.
# "|| true", bo przed pierwszym certyfikatem katalogu jeszcze nie ma,
# a przy pipefail nieudany find konczylby skrypt bez komunikatu.
find_cert() {
    find data/caddy/caddy/certificates -name "$DOMAIN.crt" 2>/dev/null | head -n 1 || true
}

crt=$(find_cert)
if [ -z "$crt" ] && [ "${1:-}" = "--wait" ]; then
    echo "czekam, az Caddy pobierze certyfikat dla $DOMAIN..."
    for _ in $(seq 60); do
        sleep 2
        crt=$(find_cert)
        [ -n "$crt" ] && break
    done
fi
if [ -z "$crt" ]; then
    echo "brak certyfikatu dla $DOMAIN w data/caddy - sprawdz: docker compose logs caddy" >&2
    echo "(najczestsza przyczyna: port 80 zamkniety albo domena nie wskazuje na ten serwer)" >&2
    exit 1
fi
key="${crt%.crt}.key"

mkdir -p certs
if cmp -s "$crt" certs/server.crt && cmp -s "$key" certs/server.key; then
    echo "certyfikat bez zmian"
    exit 0
fi

cp "$crt" certs/server.crt
cp "$key" certs/server.key
chmod 644 certs/server.crt
chmod 600 certs/server.key
if [ "$(id -u)" = 0 ]; then
    chown 1883:1883 certs/server.crt certs/server.key
fi
echo "skopiowano certyfikat: $crt"

# Restart, a nie przeladowanie: zerwane polaczenia robot i panel
# odnawiaja same, a restart na pewno wczytuje nowe pliki.
if docker compose ps --status running mosquitto 2>/dev/null | grep -q mosquitto; then
    docker compose restart mosquitto
fi
