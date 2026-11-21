/* Test hostowy flora_tunnel: sklad, podzial, zlozenie, zgubiony kawalek. */
#include <stdio.h>
#include <string.h>

#include "flora_tunnel.h"

static int fails;
#define check(c, msg) do { if (!(c)) { printf("BLAD: %s\n", msg); fails++; } } while (0)

static bool roundtrip(const char *topic, const char *payload, int qos, bool retain)
{
    uint8_t msg[FLORA_TUNNEL_MAX_MSG];
    const size_t len = flora_tunnel_pack(topic, payload, strlen(payload), msg, sizeof msg);
    if (!len) return false;
    flora_tunnel_rx_t rx;
    flora_tunnel_rx_init(&rx);
    const int n = flora_tunnel_chunk_count(len);
    bool done = false;
    for (int i = 0; i < n; i++) {
        flora_mqtt_chunk_t c;
        flora_tunnel_chunk(msg, len, 7, i, qos, retain, &c);
        done = flora_tunnel_rx_feed(&rx, &c);
        if (done != (i == n - 1)) return false;
    }
    const char *t; size_t tl; const uint8_t *d; size_t dl;
    if (!flora_tunnel_unpack(rx.buf, rx.len, &t, &tl, &d, &dl)) return false;
    return tl == strlen(topic) && memcmp(t, topic, tl) == 0 &&
           dl == strlen(payload) && memcmp(d, payload, dl) == 0 &&
           rx.qos == qos && rx.retain == retain;
}

int main(void)
{
    check(roundtrip("tele/power", "{\"v\":13.080,\"a\":0.158,\"w\":2.07}", 0, false), "krotka");
    check(roundtrip("cmd/drive", "", 1, false), "pusta tresc");
    char big[300];
    memset(big, 'x', sizeof big - 1);
    big[sizeof big - 1] = 0;
    check(roundtrip("config_auto", big, 1, true), "dluga, retain, qos 1");
    /* Dokladnie 60 B: jeden pelny kawalek. */
    char exact[60 - 1 - 4 + 1];
    memset(exact, 'y', sizeof exact - 1);
    exact[sizeof exact - 1] = 0;
    check(roundtrip("ab/c", exact, 0, false), "rowno 60 B");

    /* Zgubiony srodkowy kawalek: wiadomosc przepada, nastepna przechodzi. */
    uint8_t msg[FLORA_TUNNEL_MAX_MSG];
    const size_t len = flora_tunnel_pack("tele/x", big, strlen(big), msg, sizeof msg);
    flora_tunnel_rx_t rx;
    flora_tunnel_rx_init(&rx);
    const int n = flora_tunnel_chunk_count(len);
    bool any = false;
    for (int i = 0; i < n; i++) {
        if (i == 2) continue;
        flora_mqtt_chunk_t c;
        flora_tunnel_chunk(msg, len, 1, i, 0, false, &c);
        any |= flora_tunnel_rx_feed(&rx, &c);
    }
    check(!any, "dziura nie sklada wiadomosci");
    check(rx.dropped == 1, "dziura liczona raz");
    const size_t l2 = flora_tunnel_pack("tele/y", "ok", 2, msg, sizeof msg);
    flora_mqtt_chunk_t c;
    flora_tunnel_chunk(msg, l2, 2, 0, 0, false, &c);
    check(flora_tunnel_rx_feed(&rx, &c), "po dziurze nastepna dochodzi");

    /* Za dlugi temat albo calosc - odmowa. */
    char longt[200];
    memset(longt, 't', sizeof longt - 1);
    longt[sizeof longt - 1] = 0;
    check(flora_tunnel_pack(longt, "a", 1, msg, sizeof msg) == 0, "za dlugi temat");
    char huge[FLORA_TUNNEL_MAX_MSG];
    memset(huge, 'h', sizeof huge);
    check(flora_tunnel_pack("t", huge, sizeof huge, msg, sizeof msg) == 0, "za dluga calosc");

    printf(fails ? "WYNIK: BLAD (%d)\n" : "WYNIK: OK\n", fails);
    return fails ? 1 : 0;
}
