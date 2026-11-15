/**
 * @file flora_hkf.h
 * @brief Filtr Kalmana kursu w glowicy: stan [psi, b_z].
 *
 *   psi  [st]    kurs kompasowy glowicy, CW od polnocy magnetycznej, 0..360
 *   b_z  [st/s]  resztkowy dryf zyroskopu osi pionowej (po offsecie z NVS)
 *
 * Zastepuje filtr komplementarny (head_fuse_heading). Roznice:
 *  - dryf zyroskopu jest estymowany NA BIEZACO. Offset z NVS mierzony jest
 *    raz, a glowica na sloncu sie grzeje i offset plynie z temperatura,
 *  - filtr zna swoja niepewnosc sigma_psi i wysyla ja w dol - EKF pozycji
 *    nie musi jej zgadywac. Bez magnetometru sigma rosnie sama,
 *  - pomiar magnetometru przechodzi bramke: skok kursu niezgodny z zyro
 *    (przejazd obok stalowego przedmiotu) jest odrzucany i liczony.
 *
 * Konwencja: omega_dps to tempo zmiany KURSU (dodatnie w prawo, CW), nie
 * surowe omega_z IMU - head_fusion podaje -gyro_z, bo Z IMU jest w gore.
 * Czysty C, bez ESP-IDF - sprawdzane w host/test.c.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float q_psi;      /**< szum katowy zyro [st^2/s] (z drganiami, nie tylko z karty) */
    float q_bias;     /**< bladzenie dryfu [(st/s)^2/s] - zmiany z temperatura */
    float sigma_b0;   /**< poczatkowa niepewnosc dryfu [st/s] */
    float gate;       /**< prog chi^2 dla pomiaru (6.63 = 99%) */
    /**
     * Dolny prog raportowanej niepewnosci [st]. Filtr traktuje pomiary
     * magnetometru jak niezalezne, a ich bledy (resztki kalibracji,
     * lokalne anomalie pola) sa skorelowane w czasie - sama P zeszlaby
     * do ulamka stopnia i EKF w bryli ufalby kursowi bardziej niz trzeba.
     */
    float sigma_floor;
} flora_hkf_cfg_t;

#define FLORA_HKF_DEFAULT_CFG()   \
    {                             \
        .q_psi    = 0.05f,        \
        .q_bias   = 1e-4f,        \
        .sigma_b0 = 0.5f,         \
        .gate     = 6.63f,        \
        .sigma_floor = 2.0f,      \
    }

typedef struct {
    flora_hkf_cfg_t cfg;
    float psi;          /**< 0..360 */
    float bz;
    float P[2][2];
    uint32_t n_accepted;
    uint32_t n_rejected;
    uint32_t reject_run;  /**< kolejne odrzucenia - wejscie detektora zaklocen */
} flora_hkf_t;

void flora_hkf_init(flora_hkf_t *kf, const flora_hkf_cfg_t *cfg, float psi0_deg, float sigma0_deg);

/**
 * Ustawia sam kurs od nowa (np. po kalibracji magnetometru albo gdy filtr
 * dlugo nie zgadza sie z magnetometrem), zostawiajac oszacowany dryf b_z
 * i jego niepewnosc. Zeruje korelacje kursu z dryfem.
 */
void flora_hkf_reset_heading(flora_hkf_t *kf, float psi_deg, float sigma_deg);

/** Krok z zyroskopu. omega_dps - predkosc katowa wokol pionu (po offsecie z NVS). */
void flora_hkf_predict(flora_hkf_t *kf, float omega_dps, float dt_s);

/** Czy pomiar przeszedlby bramke - bez aktualizacji (do detektora w czasie zaklocenia). */
bool flora_hkf_would_accept(const flora_hkf_t *kf, float z_deg, float sigma_deg);

/** Pomiar kursu magnetycznego. false = odrzucony przez bramke. */
bool flora_hkf_update(flora_hkf_t *kf, float z_deg, float sigma_deg);

/**
 * ZUPT: pomiar dryfu na postoju. Robot stoi (enkodery bryly) i glowica
 * nie obraca sie, wiec srednie wskazanie zyroskopu to sam dryf.
 *
 * @param z_dps      srednie tempo kursu z zyroskopu w oknie postoju
 * @param sigma_dps  niepewnosc tej sredniej
 */
bool flora_hkf_update_bias(flora_hkf_t *kf, float z_dps, float sigma_dps);

/** Niepewnosc kursu do wyslania: sqrt(P_psipsi + sigma_floor^2). */
float flora_hkf_sigma(const flora_hkf_t *kf);

#ifdef __cplusplus
}
#endif
