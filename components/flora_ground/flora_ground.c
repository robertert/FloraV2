/**
 * @file flora_ground.c
 * @brief Potok: klatka RGB565 -> 16 odleglosci w centymetrach.
 *
 * Kolejnosc krokow i uzasadnienia progow pochodza z prototypu w Pythonie
 * (`tools/vision/ground.py`) sprawdzonego na zebranym zbiorze. Kazda
 * liczba, ktora tu wyglada na wyssana z palca, ma swoj pomiar - opisany
 * przy miejscu uzycia.
 */
#include "flora_ground.h"

#include <math.h>
#include <string.h>

/* Kubelkow chromatycznosci na os. 12x12 to kompromis: dosc drobno, zeby
 * odroznic cegle od betonu, dosc grubo, zeby szum czujnika nie rozmazywal
 * jednej powierzchni na kilkanascie kubelkow. */
#define CHROMA_BINS 12
#define HIST_SIZE   (CHROMA_BINS * CHROMA_BINS)

/* Komorka ponizej tylu pikseli w kadrze jest "poza kadrem", a nie
 * "nieznana". Przy nadirze zasieg w pikselach ZALEZY OD KIERUNKU: po
 * dluzszej osi obrazu siega dalej niz po krotszej, wiec skrajne wycinki
 * maja krotsze pierscienie i to jest normalne, nie slepota. */
#define MIN_CELL_PX 16

typedef struct {
    uint16_t hist[HIST_SIZE];
    uint32_t n, dark, bright;
    uint32_t y_sum, y2_sum;
} cell_t;

typedef struct {
    uint8_t mask[HIST_SIZE];    /**< ktore kubelki barwy sa podlozem */
    uint8_t valid;
} ref_t;

static struct {
    flora_ground_cfg_t cfg;
    bool     ready;
    uint16_t cells;

    int16_t *cellmap;           /**< piksel -> komorka, -1 poza siatka */
    cell_t  *cell;
    ref_t   *mem;               /**< pamiec wzorcow spod robota */
    uint8_t  mem_count, mem_next;

    /* Bufory robocze. W ARENIE, NIE NA STOSIE: same histogramy pierscieni
     * to 16 x 144 x 4 bajty, czyli 9 kB, a zadanie wizji ma stos 4 kB.
     * Pierwsza wersja trzymala je lokalnie i konczylo sie to przepelnieniem
     * stosu zaraz po starcie - na komputerze tego nie widac, bo tam watek
     * glowny ma stos liczony w megabajtach. */
    uint32_t *ring_hist;        /**< rings x HIST_SIZE */
    uint32_t *ring_total;       /**< rings */
    uint32_t *work;             /**< HIST_SIZE, do histogramu i odejmowania */
    uint8_t  *ring_order;       /**< rings x HIST_SIZE: kubelki wg malejacej liczebnosci */
    uint16_t *idx;              /**< SECTORS * MAX_RINGS */

    float ring_mm[FLORA_GROUND_MAX_RINGS + 1];
} g;

/* --------------------------------------------------------------- siatka -- */

size_t flora_ground_arena_size(const flora_ground_cfg_t *cfg)
{
    if (!cfg || !cfg->width || !cfg->height || !cfg->rings) {
        return 0;
    }
    size_t cells = (size_t)FLORA_GROUND_SECTORS * cfg->rings;
    return (size_t)cfg->width * cfg->height * sizeof(int16_t)
         + cells * sizeof(cell_t)
         + (size_t)(cfg->memory ? cfg->memory : 1) * sizeof(ref_t)
         + (size_t)cfg->rings * HIST_SIZE * sizeof(uint32_t)   /* ring_hist */
         + (size_t)cfg->rings * sizeof(uint32_t)               /* ring_total */
         + HIST_SIZE * sizeof(uint32_t)                        /* work */
         + (size_t)cfg->rings * HIST_SIZE * sizeof(uint8_t)     /* ring_order */
         + (size_t)FLORA_GROUND_SECTORS * FLORA_GROUND_MAX_RINGS * sizeof(uint16_t);
}

int flora_ground_init(const flora_ground_cfg_t *cfg, void *arena, size_t arena_len)
{
    if (!cfg || !arena || cfg->rings == 0 || cfg->rings > FLORA_GROUND_MAX_RINGS) {
        return -1;
    }
    if (cfg->r_out_px <= cfg->r_in_px || cfg->thr <= 0.0f || cfg->thr > 1.0f) {
        return -1;
    }
    if (arena_len < flora_ground_arena_size(cfg)) {
        return -1;
    }

    memset(&g, 0, sizeof g);
    g.cfg   = *cfg;
    g.cells = (uint16_t)(FLORA_GROUND_SECTORS * cfg->rings);

    uint8_t *p = (uint8_t *)arena;
    g.cellmap = (int16_t *)p; p += (size_t)cfg->width * cfg->height * sizeof(int16_t);
    g.cell    = (cell_t  *)p; p += (size_t)g.cells * sizeof(cell_t);
    g.mem     = (ref_t   *)p; p += (size_t)(cfg->memory ? cfg->memory : 1) * sizeof(ref_t);
    g.ring_hist  = (uint32_t *)p; p += (size_t)cfg->rings * HIST_SIZE * sizeof(uint32_t);
    g.ring_total = (uint32_t *)p; p += (size_t)cfg->rings * sizeof(uint32_t);
    g.work       = (uint32_t *)p; p += HIST_SIZE * sizeof(uint32_t);
    g.ring_order = (uint8_t  *)p; p += (size_t)cfg->rings * HIST_SIZE * sizeof(uint8_t);
    g.idx        = (uint16_t *)p;

    /* Przypisanie pikseli do komorek liczymy RAZ. W petli przetwarzania nie
     * ma ani jednej funkcji trygonometrycznej - tylko odczyt z tablicy. */
    const float cx = (cfg->width - 1) / 2.0f;
    const float cy = (cfg->height - 1) / 2.0f;
    const float half = cfg->fan_deg / 2.0f;
    const float step = cfg->fan_deg / FLORA_GROUND_SECTORS;
    const float ring_step = (cfg->r_out_px - cfg->r_in_px) / cfg->rings;

    for (int y = 0; y < cfg->height; y++) {
        const float dy = y - cy;
        for (int x = 0; x < cfg->width; x++) {
            const float dx = x - cx;
            const float rad = sqrtf(dx * dx + dy * dy);
            int16_t v = -1;
            if (rad >= cfg->r_in_px && rad < cfg->r_out_px) {
                float ang = atan2f(dy, dx) * 180.0f / (float)M_PI - cfg->forward_deg;
                while (ang < -180.0f) ang += 360.0f;
                while (ang >  180.0f) ang -= 360.0f;
                if (ang >= -half && ang < half) {
                    int sec  = (int)((ang + half) / step);
                    int ring = (int)((rad - cfg->r_in_px) / ring_step);
                    if (sec  >= FLORA_GROUND_SECTORS) sec  = FLORA_GROUND_SECTORS - 1;
                    if (ring >= cfg->rings)           ring = cfg->rings - 1;
                    v = (int16_t)(ring * FLORA_GROUND_SECTORS + sec);
                }
            }
            g.cellmap[y * cfg->width + x] = v;
        }
    }

    /* Granice pierscieni na ziemi: rho = h * tan(r / f). Bez wysokosci
     * zostaja zera i wynik jest tylko indeksem pierscienia. */
    for (int k = 0; k <= cfg->rings; k++) {
        const float r = cfg->r_in_px + ring_step * k;
        if (cfg->cam_height_mm > 0.0f && cfg->focal_px_per_rad > 0.0f) {
            const float theta = r / cfg->focal_px_per_rad;      /* radiany */
            g.ring_mm[k] = (theta < 1.5f)
                         ? cfg->cam_height_mm * tanf(theta)
                         : 0.0f;    /* blisko 90 stopni tangens ucieka */
        } else {
            g.ring_mm[k] = 0.0f;
        }
    }

    g.ready = true;
    return 0;
}

void flora_ground_reset(void)
{
    g.mem_count = 0;
    g.mem_next  = 0;
}

float flora_ground_ring_mm(uint8_t ring)
{
    if (!g.ready || ring > g.cfg.rings) {
        return 0.0f;
    }
    /* Srodek pierscienia, nie granica: to on reprezentuje przeszkode. */
    return 0.5f * (g.ring_mm[ring] + g.ring_mm[ring + 1]);
}

/* ---------------------------------------------------------------- cechy -- */

static bool cell_covered(const cell_t *c)
{
    return (c->n + c->dark + c->bright) >= MIN_CELL_PX;
}

static bool cell_usable(const cell_t *c)
{
    const uint32_t total = c->n + c->dark + c->bright;
    if (total < MIN_CELL_PX) {
        return false;
    }
    /* Przewaga przesteru albo czerni znaczy "nie wiem", a nie "przeszkoda".
     * To jest zrodlo zera w kontrakcie, a zero i 254 to NIE to samo. */
    return (c->dark + c->bright) * 2 < total;
}

static void extract(const uint8_t *buf)
{
    memset(g.cell, 0, (size_t)g.cells * sizeof(cell_t));

    const uint32_t px = (uint32_t)g.cfg.width * g.cfg.height;
    const uint16_t dark = g.cfg.dark_sum;
    const uint8_t  bright = g.cfg.bright;

    for (uint32_t i = 0; i < px; i++) {
        const int16_t ci = g.cellmap[i];
        if (ci < 0) {
            continue;
        }
        cell_t *c = &g.cell[ci];

        /* Rozpakowanie RGB565 big-endian, dokladnie jak dekoduje
         * esp32-camera w conversions/to_bmp.c. */
        const uint8_t hb = buf[2 * i], lb = buf[2 * i + 1];
        const uint16_t r = hb & 0xF8;
        const uint16_t gg = ((hb & 0x07) << 5) | ((lb & 0xE0) >> 3);
        const uint16_t b = (lb & 0x1F) << 3;

        if (r > bright || gg > bright || b > bright) {
            c->bright++;
            continue;
        }
        const uint16_t sum = r + gg + b;
        if (sum < dark) {
            c->dark++;
            continue;
        }

        /* Chromatycznosc znormalizowana: ten sam beton w sloncu i w cieniu
         * wpada do tego samego kubelka. */
        const uint16_t rb = (uint16_t)((r  * (CHROMA_BINS - 1)) / sum);
        const uint16_t gb = (uint16_t)((gg * (CHROMA_BINS - 1)) / sum);
        c->hist[gb * CHROMA_BINS + rb]++;

        const uint16_t y = (uint16_t)((r + 2 * gg + b) >> 2);
        c->y_sum  += y;
        c->y2_sum += (uint32_t)y * y;
        c->n++;
    }
}

/* -------------------------------------------------------- wzorzec podloza -- */

/**
 * Wzorzec z DOMINUJACYCH barw zbioru komorek.
 *
 * Do wzorca pierscienia nie wolno uzyc reguly "kazdy kubelek, ktory
 * wystapil". Pierscien zawiera przeszkode, ktorej szukamy, a przy takiej
 * regule uznalaby ona sama siebie za podloze - sprawdzone, detektor
 * przestawal widziec cokolwiek. Bierzemy wiec kubelki od najliczniejszego,
 * az pokryja `cover` pikseli, a ogon rozkladu zostaje przeszkoda.
 */
static void ref_from_hist(ref_t *out, const uint32_t *hist, uint32_t total, float cover)
{
    memset(out->mask, 0, sizeof out->mask);
    out->valid = 0;
    if (total < 64) {
        return;
    }

    /* Pokrycie <= 0 znaczy inna regule: wzorzec SPOD ROBOTA, gdzie z
     * zalozenia cala powierzchnia jest podlozem, bo robot po niej jedzie.
     * Bierzemy wtedy kazdy kubelek, ktory wystapil istotnie, a prog odsiewa
     * tylko szum rozmazujacy barwe na sasiednie kubelki. */
    if (cover <= 0.0f) {
        const uint32_t cut = (total / 500 > 2) ? total / 500 : 2;
        for (int k = 0; k < HIST_SIZE; k++) {
            out->mask[k] = (hist[k] >= cut) ? 1 : 0;
        }
        out->valid = 1;
        return;
    }

    /* Wybor najliczniejszych kubelkow bez sortowania calej tablicy: przy
     * 144 kubelkach i progu pokrycia zwykle wystarcza kilkanascie obrotow,
     * a kazdy to jeden przebieg po tablicy. Taniej niz qsort i bez
     * alokacji. */
    const uint32_t goal = (uint32_t)(cover * total);
    uint32_t acc = 0;
    while (acc < goal) {
        int best = -1;
        uint32_t best_v = 0;
        for (int k = 0; k < HIST_SIZE; k++) {
            if (!out->mask[k] && hist[k] > best_v) {
                best_v = hist[k];
                best = k;
            }
        }
        if (best < 0 || best_v == 0) {
            break;
        }
        out->mask[best] = 1;
        acc += best_v;
    }
    out->valid = 1;
}

/** Zbiera histogram ze zbioru komorek. Nieuzyteczne pomijamy. */
static uint32_t hist_collect(uint32_t *hist, const uint16_t *idx, int count)
{
    memset(hist, 0, HIST_SIZE * sizeof(uint32_t));
    uint32_t total = 0;
    for (int i = 0; i < count; i++) {
        const cell_t *c = &g.cell[idx[i]];
        if (!cell_usable(c)) {
            continue;
        }
        for (int k = 0; k < HIST_SIZE; k++) {
            hist[k] += c->hist[k];
        }
        total += c->n;
    }
    return total;
}

/**
 * Porzadkuje kubelki pierscienia wedlug malejacej liczebnosci.
 *
 * Sortowanie przez wstawianie: 144 elementow i kilkanascie pierscieni to
 * ulamek milisekundy, a qsort wymagalby wskaznika na funkcje porownujaca
 * i dostepu do histogramu przez zmienna globalna.
 */
static void order_bins(uint8_t *order, const uint32_t *hist)
{
    for (int k = 0; k < HIST_SIZE; k++) {
        order[k] = (uint8_t)k;
    }
    for (int i = 1; i < HIST_SIZE; i++) {
        const uint8_t key = order[i];
        const uint32_t kv = hist[key];
        int j = i - 1;
        while (j >= 0 && hist[order[j]] < kv) {
            order[j + 1] = order[j];
            j--;
        }
        order[j + 1] = key;
    }
}

/**
 * Udzial pikseli komorki nalezacych do podloza pierscienia, Z POMINIECIEM
 * WLASNEGO WKLADU komorki w odniesienie.
 *
 * DLACZEGO TAK, A NIE PRZEZ BUDOWANIE MASKI: pierwsza wersja skladala dla
 * KAZDEJ komorki osobny wzorzec, przegladajac 144 kubelki tyle razy, ile
 * ich wybrala. Przy 192 komorkach wychodzilo z tego ponad dwa miliony
 * operacji na klatke i zmierzone 233 ms, czyli caly budzet taktu. Tutaj
 * kolejnosc kubelkow jest policzona RAZ na pierscien, a komorka wymaga
 * jednego przebiegu po 144 kubelkach: okolo stukrotnie taniej.
 *
 * Przyblizenie jest jedno i swiadome: kolejnosc bierzemy z pelnego
 * pierscienia, a nie z pierscienia bez tej komorki. Odjecie jednego z
 * szesnastu wycinkow nie przestawia czolowych kubelkow, bo one maja
 * wielokrotnie wiecej pikseli niz ogon.
 */
static float ring_match(int ring, const cell_t *c, float cover)
{
    const uint32_t *rh = &g.ring_hist[(size_t)ring * HIST_SIZE];
    const uint8_t  *ord = &g.ring_order[(size_t)ring * HIST_SIZE];

    const uint32_t loo_total = (g.ring_total[ring] > c->n)
                             ? (g.ring_total[ring] - c->n) : 0;
    if (loo_total < 64 || c->n < MIN_CELL_PX) {
        return 0.0f;
    }

    const uint32_t goal = (uint32_t)(cover * loo_total);
    uint32_t acc_loo = 0, acc_cell = 0;
    for (int i = 0; i < HIST_SIZE; i++) {
        const uint8_t k = ord[i];
        const uint32_t loo = (rh[k] > c->hist[k]) ? (rh[k] - c->hist[k]) : 0;
        if (loo == 0 && c->hist[k] == 0) {
            continue;
        }
        acc_loo  += loo;
        acc_cell += c->hist[k];
        if (acc_loo >= goal) {
            break;
        }
    }
    return (float)acc_cell / (float)c->n;
}

/** Udzial pikseli komorki, ktorych barwa jest w masce wzorca. */
static float ref_match(const ref_t *ref, const cell_t *c)
{
    if (!ref->valid || c->n < MIN_CELL_PX) {
        return 0.0f;
    }
    uint32_t hit = 0;
    for (int k = 0; k < HIST_SIZE; k++) {
        if (ref->mask[k]) {
            hit += c->hist[k];
        }
    }
    return (float)hit / (float)c->n;
}

/* ---------------------------------------------------------------- potok -- */

enum { V_OUTSIDE, V_UNKNOWN, V_GROUND, V_OBSTACLE };

int flora_ground_process(const uint8_t *rgb565, flora_ground_result_t *out)
{
    if (!g.ready || !rgb565 || !out) {
        return -1;
    }
    memset(out, 0, sizeof *out);

    extract(rgb565);

    const uint8_t rings = g.cfg.rings;
    uint16_t *idx = g.idx;

    /* Wzorzec spod robota: dwa najblizsze pierscienie to powierzchnia, po
     * ktorej robot wlasnie jedzie. Trafia do pamieci, zeby przetrwac wjazd
     * z kostki na beton. */
    int n = 0;
    for (int ring = 0; ring < 2 && ring < rings; ring++) {
        for (int s = 0; s < FLORA_GROUND_SECTORS; s++) {
            idx[n++] = (uint16_t)(ring * FLORA_GROUND_SECTORS + s);
        }
    }
    ref_t inner;
    const uint32_t total = hist_collect(g.work, idx, n);
    ref_from_hist(&inner, g.work, total, 0.0f);   /* 0 = regula "spod robota" */
    if (inner.valid && g.cfg.memory) {
        g.mem[g.mem_next] = inner;
        g.mem_next = (uint8_t)((g.mem_next + 1) % g.cfg.memory);
        if (g.mem_count < g.cfg.memory) {
            g.mem_count++;
        }
    }

    /* Histogram KAZDEGO PIERSCIENIA. Wzorzec budujemy z niego dopiero przy
     * ocenie konkretnej komorki, bo wtedy mozna odjac wlasny wycinek. */
    for (int ring = 0; ring < rings; ring++) {
        for (int s = 0; s < FLORA_GROUND_SECTORS; s++) {
            idx[s] = (uint16_t)(ring * FLORA_GROUND_SECTORS + s);
        }
        g.ring_total[ring] = hist_collect(&g.ring_hist[(size_t)ring * HIST_SIZE],
                                          idx, FLORA_GROUND_SECTORS);
        order_bins(&g.ring_order[(size_t)ring * HIST_SIZE],
                   &g.ring_hist[(size_t)ring * HIST_SIZE]);
    }

    uint8_t *verdict = out->cell_verdict;
    uint32_t lost = 0, vote = 0;

    for (int ci = 0; ci < g.cells; ci++) {
        const cell_t *c = &g.cell[ci];
        lost += c->dark + c->bright;
        vote += c->n;

        if (!cell_covered(c)) {
            verdict[ci] = V_OUTSIDE;
            continue;
        }
        if (!cell_usable(c)) {
            verdict[ci] = V_UNKNOWN;
            out->unusable_cells++;
            continue;
        }

        /* WZORZEC PIERSCIENIA Z POMINIECIEM WLASNEGO WYCINKA.
         *
         * Pierscien jest dobrym wzorcem, bo zna winiete na swoim promieniu,
         * ale ma jedna wade: przeszkoda wchodzi do wzorca, wedlug ktorego
         * jest oceniana, i sama siebie uznaje za podloze. Zmierzone -
         * syntetyczna cegla zajmujaca 12% pierscienia znikala calkowicie,
         * bo podloga sama dawala tylko 88%, a prog pokrycia wynosil 90%.
         *
         * Odejmujemy wiec od histogramu pierscienia wlasny wycinek komorki.
         * Przeszkoda jest wtedy oceniana przez pietnascie pozostalych
         * wycinkow, czyli przez czyste podloze na tym samym promieniu.
         */
        float best = ring_match(ci / FLORA_GROUND_SECTORS, c, g.cfg.ring_cover);

        /* Pamiec wzorcow spod robota jako druga szansa: ma przetrwac wjazd
         * z kostki na beton, gdy nowa powierzchnia jeszcze nie weszla w
         * pierscien odniesienia. */
        if (best < g.cfg.thr) {
            for (int m = 0; m < g.mem_count; m++) {
                const float v = ref_match(&g.mem[m], c);
                if (v > best) {
                    best = v;
                }
            }
        }
        verdict[ci] = (best >= g.cfg.thr) ? V_GROUND : V_OBSTACLE;
    }

    /* Skan od srodka na zewnatrz, z trzema regulami zakonczenia:
     *  - DWA kolejne pierscienie niezgodne to przeszkoda; jeden to szum
     *    albo kamyk ponizej progu rozmiaru,
     *  - DWA kolejne nieznane koncza skan jako "nie wiem", bo przez obszar,
     *    ktorego nie umiemy ocenic, nie widzimy i nie wolno obiecywac, ze
     *    za nim jest wolne,
     *  - komorka poza kadrem to koniec pola widzenia w tym kierunku, czyli
     *    "czysto do granicy widzenia", a nie slepota.
     */
    for (int s = 0; s < FLORA_GROUND_SECTORS; s++) {
        int hit = -1, run = 0, unk = 0;
        bool blind = false;

        for (int ring = 0; ring < rings; ring++) {
            const uint8_t v = verdict[ring * FLORA_GROUND_SECTORS + s];
            if (v == V_OUTSIDE) {
                break;
            }
            if (v == V_UNKNOWN) {
                if (++unk >= 2) {
                    blind = true;
                    break;
                }
                continue;
            }
            unk = 0;
            if (v == V_OBSTACLE) {
                if (++run >= 2) {
                    hit = ring - 1;
                    break;
                }
            } else {
                run = 0;
            }
        }

        if (hit >= 0) {
            out->first_ring[s] = (uint8_t)hit;
            const float mm = flora_ground_ring_mm((uint8_t)hit);
            if (mm <= 0.0f) {
                out->sectors[s] = FLORA_GROUND_UNKNOWN;   /* brak kalibracji */
            } else {
                long cm = lroundf(mm / 10.0f);
                if (cm < 1)   cm = 1;
                if (cm > 253) cm = 253;
                out->sectors[s] = (uint8_t)cm;
            }
        } else {
            out->first_ring[s] = 255;
            out->sectors[s] = blind ? FLORA_GROUND_UNKNOWN : FLORA_GROUND_CLEAR;
        }
    }

    const uint32_t seen = vote + lost;
    out->lost_px_permille = seen ? (uint16_t)((1000ULL * lost) / seen) : 1000;
    out->valid = (out->unusable_cells < g.cells / 2 && g.mem_count > 0) ? 1 : 0;
    return 0;
}
