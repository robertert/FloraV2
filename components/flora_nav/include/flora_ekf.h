/**
 * @file flora_ekf.h
 * @brief Rozszerzony filtr Kalmana pozycji robota (bryla).
 *
 * Czysty C bez zaleznosci od ESP-IDF - sprawdzany na komputerze
 * (host/test.c) na symulacji kwadratu 5 x 5 m.
 *
 * ================= STAN =================
 *   x[0] = x      [mm]  pozycja w ukladzie mapy
 *   x[1] = y      [mm]
 *   x[2] = theta  [rad] kat osi +X robota, CCW od osi X mapy, (-pi, pi]
 *   x[3] = b      [rad] kurs kompasowy osi X mapy, (-pi, pi]
 *
 * b laczy wszystko, co rozni kurs z glowicy od kata w mapie: polnoc
 * magnetyczna, montaz glowicy na maszcie i obrot samej mapy (mape
 * wyznaczaja paliki ze znacznikami, nie polnoc). Bez znacznikow b stoi
 * w miejscu i niczego nie psuje; znaczniki je dostrajaja.
 *
 * ================= KONWENCJE =================
 * Robot: +X = przod (D-037), +Y w lewo, kat dodatni CCW (matematycznie).
 * Kursy z glowicy i kompasu slonecznego sa KOMPASOWE: CW od polnocy.
 * Obrot robota CCW o d zmniejsza kurs o d, wiec model pomiaru kursu:
 *
 *     psi = b - theta   (mod 2 pi)
 *
 * ================= KROKI =================
 *  predict_odom   przyrosty drog kol z enkoderow, model napedu
 *                 roznicowego. Szum na kolo, proporcjonalny do drogi:
 *                 sigma^2 = k * |ds|. Poslizg na trawie w skretach
 *                 wychodzi z tego sam, bo w skrecie kola jada w przeciwne
 *                 strony i rozjazd daje duzy przyrost niepewnosci theta.
 *  update_heading kurs z glowicy albo z kompasu slonecznego, pomiar 1D.
 *  update_tag     odleglosc i kat do znacznika o znanym polozeniu, 2D.
 *
 * Kazda aktualizacja przechodzi bramke Mahalanobisa: pomiar zbyt daleki
 * od przewidywania (wzgledem niepewnosci) jest odrzucany i liczony.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FLORA_EKF_N 4

enum { FLORA_EKF_X, FLORA_EKF_Y, FLORA_EKF_TH, FLORA_EKF_B };

typedef struct {
    float track_mm;   /**< rozstaw kol (efektywny, z UMBmark) */
    float k_l;        /**< wariancja drogi lewego kola na mm drogi [mm^2/mm] */
    float k_r;        /**< to samo dla prawego */
    float q_b;        /**< bladzenie b [rad^2/s] - zaklocenia magnetyczne */
    float gate_1d;    /**< prog chi^2 dla pomiaru 1D (6.63 = 99%) */
    float gate_2d;    /**< prog chi^2 dla pomiaru 2D (9.21 = 99%) */
} flora_ekf_cfg_t;

#define FLORA_EKF_DEFAULT_CFG()      \
    {                                \
        .track_mm = 390.0f,          \
        .k_l      = 0.5f,            \
        .k_r      = 0.5f,            \
        .q_b      = 3e-6f,           \
        .gate_1d  = 6.63f,           \
        .gate_2d  = 9.21f,           \
    }

typedef struct {
    flora_ekf_cfg_t cfg;
    float x[FLORA_EKF_N];
    float P[FLORA_EKF_N][FLORA_EKF_N];

    uint32_t n_heading;   /**< przyjete pomiary kursu */
    uint32_t n_tag;       /**< przyjete pomiary znacznikow */
    uint32_t n_rejected;  /**< odrzucone przez bramke */
} flora_ekf_t;

/**
 * Start: robot w (0, 0, 0) mapy z zerowa niepewnoscia pozycji - mapa
 * zaczyna sie tam, gdzie robot stoi. b nieznane (sigma = pi), wiec
 * pierwszy pomiar kursu ustawi je praktycznie w calosci.
 */
void flora_ekf_init(flora_ekf_t *ekf, const flora_ekf_cfg_t *cfg);

/**
 * Ustawia pozycje i kat (np. robot postawiony w znanym miejscu).
 * Nie rusza b ani jego niepewnosci; zeruje korelacje z b.
 */
void flora_ekf_set_pose(flora_ekf_t *ekf, float x_mm, float y_mm, float th_rad,
                        float sigma_xy_mm, float sigma_th_rad);

/**
 * Krok predykcji z odometrii.
 *
 * @param ds_l, ds_r  przyrost drogi kol od poprzedniego kroku [mm], ze znakiem
 * @param dt_s        czas kroku - tylko do bladzenia b
 */
void flora_ekf_predict_odom(flora_ekf_t *ekf, float ds_l, float ds_r, float dt_s);

/**
 * Krok predykcji BEZ odometrii - do porownania wariantow w terenie.
 *
 * Odometria to w tym filtrze krok predykcji, nie zwykly pomiar, wiec jej
 * "wylaczenie" wymaga innego modelu ruchu. Tu najprostszy: poza stoi
 * w miejscu, a niepewnosc rosnie z czasem (bladzenie losowe). Pozycja
 * zmienia sie wtedy tylko po znacznikach, kat po pomiarach kursu.
 *
 * @param q_xy  przyrost wariancji pozycji [mm^2/s], rzad (predkosc)^2
 * @param q_th  przyrost wariancji kata [rad^2/s]
 */
void flora_ekf_predict_static(flora_ekf_t *ekf, float dt_s, float q_xy, float q_th);

/**
 * Pomiar kursu kompasowego (glowica albo slonce).
 *
 * @param psi_rad    kurs CW od polnocy (dowolny zakres, zawijany)
 * @param sigma_rad  odchylenie standardowe pomiaru
 * @return true gdy przyjety, false gdy odrzucony przez bramke
 */
bool flora_ekf_update_heading(flora_ekf_t *ekf, float psi_rad, float sigma_rad);

/**
 * Pomiar znacznika o znanym polozeniu.
 *
 * @param tag_x_mm, tag_y_mm  polozenie palika w mapie
 * @param cam_x_mm, cam_y_mm  polozenie kamery w ukladzie robota
 * @param d_mm                zmierzona odleglosc kamera-znacznik
 * @param beta_rad            kat do znacznika od osi +X robota, CCW
 *                            (flora_tags podaje dodatni W PRAWO - zmien znak)
 */
bool flora_ekf_update_tag(flora_ekf_t *ekf, float tag_x_mm, float tag_y_mm,
                          float cam_x_mm, float cam_y_mm,
                          float d_mm, float beta_rad,
                          float sigma_d_mm, float sigma_beta_rad);

/**
 * Najwieksze odchylenie standardowe pozycji [mm] - pierwiastek z wiekszej
 * wartosci wlasnej bloku xy macierzy P. Gdy przekroczy prog, robot
 * powinien stanac i szukac znacznikow.
 */
float flora_ekf_pos_sigma_max(const flora_ekf_t *ekf);

/** Zawija kat do (-pi, pi]. */
float flora_ekf_wrap(float a);

#ifdef __cplusplus
}
#endif
