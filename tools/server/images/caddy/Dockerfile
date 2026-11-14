# Caddy z konfiguracja i panelem debugowym. Kontekst budowania: korzen repo.
FROM caddy:2.8.4
COPY tools/server/Caddyfile /etc/caddy/Caddyfile
COPY tools/dashboard/ /srv/panel/
# Wersja serwera widoczna w naglowku panelu. CI podaje tag (vX.Y.Z),
# budowa lokalna zostawia "dev".
ARG FLORA_VERSION=dev
RUN echo "$FLORA_VERSION" > /srv/panel/version.txt
