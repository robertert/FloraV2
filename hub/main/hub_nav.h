/**
 * @file hub_nav.h
 * @brief Pozycja robota: EKF [x, y, theta, b] z odometrii, kursu glowicy,
 *        kompasu slonecznego i znacznikow AprilTag.
 *
 * Model i konwencje w components/flora_nav/include/flora_ekf.h.
 *
 * ================= WARIANTY =================
 * Kilka filtrow liczy sie ROWNOLEGLE na tych samych danych, kazdy z innym
 * zestawem zrodel (tabela s_variants w hub_nav.c). Jeden przejazd w terenie
 * daje wszystkie kombinacje na identycznej trajektorii - roznica miedzy
 * nimi wynika tylko z fuzji, nie z innego poslizgu w kolejnym przejezdzie.
 * Wariant 0 ("full") to glowny wynik; reszta tylko do porownania.
 * Kazdy wariant ma wlasny temat: tele/pose (full), tele/pose_<nazwa>.
 *
 * ================= ZRODLA =================
 *   enkodery      predykcja co 20 ms (bez nich: model "stoi w miejscu")
 *   rate_z        zyroskop glowicy (z ATTITUDE, 20 Hz) - nie zmienia theta,
 *                 ale niezgodnosc z obrotem kol poszerza niepewnosc theta
 *                 (flora_ekf_observe_gyro). Tylko warianty z ODOM i HEAD.
 *                 Bryla wysyla tez glowicy flage postoju (FLORA_CMD_SET_STILL)
 *                 do estymacji dryfu zyroskopu na postoju.
 *   ATTITUDE      kurs z glowicy (magnetometr + zyro) co 500 ms - GLOWNE
 *                 zrodlo kursu. Rzadziej niz 20 Hz celowo: to wyjscie filtra
 *                 Kalmana glowicy (flora_hkf), kolejne probki sa skorelowane.
 *                 Kurs uparcie niezgodny z odometria (10 odrzucen z rzedu,
 *                 np. po poslizgu) poszerza niepewnosc theta i wraca do filtra
 *                 (heading_recover_n w flora_ekf.h).
 *   SUN           kompas sloneczny - POMOCNICZY, patrz bramki nizej
 *   TAGS          tylko na postoju (D-034), znaczniki z mapy palikow
 *
 * ================= KOMPAS SLONECZNY =================
 * Pomocniczy - glownym kursem jest magnetometr. Zanim pomiar slonca trafi
 * do EKF, przechodzi bramki z components/flora_nav/include/flora_sungate.h
 * (swiatlo, slonce nad horyzontem, stalosc, zgodnosc z magnetometrem),
 * ktore odsiewaja lampe we wnetrzu, odbicia i slonce nisko. Do EKF trafia
 * najwyzej jeden pomiar na 5 s.
 *
 * OPOZNIENIE: kurs z glowicy jest przesuwany o obrot od chwili pomiaru
 * (rate_z * wiek), gdy glowica ma TIME_SYNC (hub_links co 10 s). Slonce
 * i znaczniki stosowane w chwili odbioru - wolne albo na postoju.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

/** Bity zrodel w wariancie. */
#define HUB_NAV_SRC_ODOM  (1u << 0)
#define HUB_NAV_SRC_HEAD  (1u << 1)
#define HUB_NAV_SRC_SUN   (1u << 2)
#define HUB_NAV_SRC_TAGS  (1u << 3)

typedef struct {
    const char *name;
    uint8_t  src;        /**< HUB_NAV_SRC_* */
    float    x_mm, y_mm;
    float    th_deg;     /**< CCW od osi X mapy */
    float    b_deg;      /**< kurs kompasowy osi X mapy */
    float    sx_mm, sy_mm, sth_deg;
    float    spos_mm;    /**< najwieksze odchylenie pozycji - sygnal "szukaj znacznikow" */
    uint32_t n_head, n_sun, n_tag, n_rej;
    uint32_t n_rec;      /**< wyjscia z blokady kursu (po poslizgu) */
} hub_nav_pose_t;

/** Stan wspolny dla wszystkich wariantow. */
typedef struct {
    bool     time_ok;      /**< czas UTC z SNTP dostepny */
    int      sun_gate;     /**< flora_sungate_result_t ostatniego pomiaru */
    float    sun_diff_deg; /**< slonce minus magnetometr przy ostatniej ocenie */
    uint32_t n_sun_seen;   /**< pomiary slonca z valid=1 */
    uint32_t n_sun_gated;  /**< odrzucone przez bramki 1-4 */
    int32_t  att_age_ms;   /**< wiek probki kursu z glowicy; -1 bez TIME_SYNC */
    bool     gyro_ok;      /**< swieze rate_z z glowicy w ostatnim takcie */
    /** Wygladzona roznica obrotu z kol i z zyroskopu [st/s]. Trwale != 0
     *  w skrecie = poslizg; na prostej = rozne promienie kol albo zle
     *  FLORA_TRACK_MM. Baza dla straznika "zyro vs odometria" z TODO.md. */
    float    gyro_vs_odo_dps;
} hub_nav_status_t;

/** Startuje filtry. Wymaga zainicjalizowanych enkoderow i hub_links. */
esp_err_t hub_nav_start(void);

bool hub_nav_ready(void);

/** Liczba wariantow; wariant 0 to glowny wynik. */
int hub_nav_variant_count(void);

/** @return false dla zlego indeksu */
bool hub_nav_get(int variant, hub_nav_pose_t *out);

void hub_nav_status(hub_nav_status_t *out);

/** Ustawia pozycje i kat WSZYSTKICH wariantow; b liczone od nowa. */
esp_err_t hub_nav_reset_pose(float x_mm, float y_mm, float th_deg);

/** Parametry z hub_params sie zmienily (rozstaw, kola, kamera, miejsce). */
void hub_nav_params_changed(void);

/* ---- mapa palikow (NVS) ---- */

#define HUB_NAV_MAX_TAGS 16

/**
 * cmd/tag:
 *   "<id> <x_mm> <y_mm>"  dodaje albo przesuwa palik (uklad mapy)
 *   "<id> -"              usuwa
 *   "clear"               usuwa wszystkie
 * Kazdy palik publikowany (retain) na config_tags/<id>, usuniety - pusty retain.
 * @return ESP_ERR_NO_MEM gdy mapa pelna, ESP_ERR_NOT_FOUND przy usuwaniu nieznanego
 */
esp_err_t hub_nav_tag_cmd(const char *arg);

int  hub_nav_tag_count(void);

/** Publikuje cala mape (po polaczeniu z brokerem). */
void hub_nav_tags_publish(void);
