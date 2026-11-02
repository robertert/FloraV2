/*
 * Uruchamianie flora_ground na komputerze, na nagranych klatkach .rgb565.
 *
 * To nie jest zabawka - to jedyny sposob, zeby udowodnic, ze wersja w C
 * liczy TO SAMO co prototyp w Pythonie. Wypisuje po jednej linii JSON na
 * klatke, w formacie zgodnym z tools/vision/ground.py, zeby dalo sie je
 * porownac polem po polu.
 *
 *   cc -O2 -I../include -o runner runner.c ../flora_ground.c -lm
 *   ./runner --r-in 2 klatka1.rgb565 klatka2.rgb565
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "flora_ground.h"

static unsigned char *slurp(const char *path, size_t *len)
{
    FILE *f = fopen(path, "rb");
    if (!f) {
        return NULL;
    }
    fseek(f, 0, SEEK_END);
    long n = ftell(f);
    fseek(f, 0, SEEK_SET);
    unsigned char *buf = malloc((size_t)n);
    if (buf && fread(buf, 1, (size_t)n, f) != (size_t)n) {
        free(buf);
        buf = NULL;
    }
    fclose(f);
    *len = (size_t)n;
    return buf;
}

int main(int argc, char **argv)
{
    flora_ground_cfg_t cfg = FLORA_GROUND_DEFAULT_CFG();
    int first = 1;

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--r-in") && i + 1 < argc) {
            cfg.r_in_px = (float)atof(argv[++i]); first = i + 1;
        } else if (!strcmp(argv[i], "--r-out") && i + 1 < argc) {
            cfg.r_out_px = (float)atof(argv[++i]); first = i + 1;
        } else if (!strcmp(argv[i], "--thr") && i + 1 < argc) {
            cfg.thr = (float)atof(argv[++i]); first = i + 1;
        } else if (!strcmp(argv[i], "--ring-cover") && i + 1 < argc) {
            cfg.ring_cover = (float)atof(argv[++i]); first = i + 1;
        } else if (!strcmp(argv[i], "--height-mm") && i + 1 < argc) {
            cfg.cam_height_mm = (float)atof(argv[++i]); first = i + 1;
        } else if (!strcmp(argv[i], "--memory") && i + 1 < argc) {
            cfg.memory = (uint8_t)atoi(argv[++i]); first = i + 1;
        } else {
            break;
        }
    }

    size_t need = flora_ground_arena_size(&cfg);
    void *arena = malloc(need);
    if (!arena || flora_ground_init(&cfg, arena, need) != 0) {
        fprintf(stderr, "init nie wyszedl\n");
        return 1;
    }
    fprintf(stderr, "# arena %zu kB, siatka %dx%d, promien %.0f-%.0f px\n",
            need / 1024, FLORA_GROUND_SECTORS, cfg.rings, cfg.r_in_px, cfg.r_out_px);

    const size_t expect = (size_t)cfg.width * cfg.height * 2;
    char prev_dir[512] = "";

    for (int i = first; i < argc; i++) {
        /* Pamiec wzorcow podloza jest zamierzona tylko w obrebie jednej
         * sekwencji. Wzorzec z obcej sesji rozszerza zbior akceptowanych
         * barw i detektor przestaje widziec przeszkode - zmierzone. */
        char dir[512];
        snprintf(dir, sizeof dir, "%s", argv[i]);
        char *slash = strrchr(dir, '/');
        if (slash) *slash = 0; else dir[0] = 0;
        if (strcmp(dir, prev_dir) != 0) {
            flora_ground_reset();
            snprintf(prev_dir, sizeof prev_dir, "%s", dir);
        }

        size_t len = 0;
        unsigned char *buf = slurp(argv[i], &len);
        if (!buf) {
            fprintf(stderr, "# %s: nie moge otworzyc\n", argv[i]);
            continue;
        }
        if (len != expect) {
            fprintf(stderr, "# %s: %zu B, oczekuje %zu\n", argv[i], len, expect);
            free(buf);
            continue;
        }

        flora_ground_result_t res;
        if (flora_ground_process(buf, &res) != 0) {
            fprintf(stderr, "# %s: process nie wyszedl\n", argv[i]);
            free(buf);
            continue;
        }
        free(buf);

        if (getenv("DUMP")) {
            static const char sym[] = " ?.#";
            fprintf(stderr, "# %s  (spacja poza kadrem, ? nie wiem, . podloze, # przeszkoda)\n",
                    argv[i]);
            for (int s2 = 0; s2 < FLORA_GROUND_SECTORS; s2++) {
                fprintf(stderr, "#  wyc %2d: ", s2);
                for (int r2 = 0; r2 < cfg.rings; r2++) {
                    fputc(sym[res.cell_verdict[r2 * FLORA_GROUND_SECTORS + s2]], stderr);
                }
                fputc('\n', stderr);
            }
        }

        const char *name = strrchr(argv[i], '/');
        name = name ? name + 1 : argv[i];

        printf("{\"frame\": \"%s\", \"first_ring\": [", name);
        for (int s = 0; s < FLORA_GROUND_SECTORS; s++) {
            int v;
            if (res.first_ring[s] != 255)                  v = res.first_ring[s];
            else if (res.sectors[s] == FLORA_GROUND_CLEAR) v = cfg.rings;
            else                                           v = -1;
            printf("%s%d", s ? ", " : "", v);
        }
        printf("], \"sectors_cm\": [");
        for (int s = 0; s < FLORA_GROUND_SECTORS; s++) {
            printf("%s%u", s ? ", " : "", res.sectors[s]);
        }
        printf("], \"unusable_cells\": %u, \"lost_px_pct\": %.1f, \"valid\": %s}\n",
               res.unusable_cells, res.lost_px_permille / 10.0,
               res.valid ? "true" : "false");
    }
    free(arena);
    return 0;
}
