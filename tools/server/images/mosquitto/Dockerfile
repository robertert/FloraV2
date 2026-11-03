# Mosquitto z konfiguracja i uprawnieniami do tematow. Kontekst: korzen repo.
# Plik hasel i certyfikat NIE sa w obrazie - montuje je serwer.
FROM eclipse-mosquitto:2.0.20
COPY --chown=mosquitto:mosquitto --chmod=600 tools/server/mosquitto/mosquitto.conf /mosquitto/config/mosquitto.conf
COPY --chown=mosquitto:mosquitto --chmod=600 tools/server/mosquitto/acl /mosquitto/config/acl
