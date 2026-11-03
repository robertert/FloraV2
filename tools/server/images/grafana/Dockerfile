# Grafana ze zrodlem danych i dashboardem. Kontekst: korzen repo.
FROM grafana/grafana:11.3.0
COPY tools/telemetry/grafana/provisioning/ /etc/grafana/provisioning/
# Dashboardy w /etc, NIE w /var/lib/grafana: tamten katalog serwer przykrywa
# wolumenem z danymi, wiec wszystko, co obraz w nim zostawi, znika.
COPY tools/telemetry/grafana/dashboards/ /etc/grafana/dashboards/
