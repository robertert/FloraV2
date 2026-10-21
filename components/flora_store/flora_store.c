#include <inttypes.h>
#include <stdlib.h>
#include <string.h>

#include "flora_store.h"

#include "esp_check.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "flora_store";

struct flora_store_s {
    nvs_handle_t handle;
    char         ns[FLORA_STORE_MAX_KEY_LEN + 1];
};

static bool s_initialized;

esp_err_t flora_store_init(void)
{
    if (s_initialized) {
        return ESP_OK;   /* idempotentne - bezpiecznie wolac z kilku miejsc */
    }

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        /* Partycja pelna albo w starszym formacie. Kasujemy i zaczynamy od
         * nowa - tracimy ustawienia, ale alternatywa to brak NVS w ogole,
         * co zablokowaloby takze WiFi i BLE. */
        ESP_LOGW(TAG, "partycja NVS wymaga skasowania (%s) - ustawienia przepadna",
                 esp_err_to_name(ret));
        ESP_RETURN_ON_ERROR(nvs_flash_erase(), TAG, "nvs_flash_erase");
        ret = nvs_flash_init();
    }
    ESP_RETURN_ON_ERROR(ret, TAG, "nvs_flash_init");

    s_initialized = true;
    ESP_LOGI(TAG, "NVS gotowe");
    return ESP_OK;
}

/** Klucz dluzszy niz limit NVS konczy sie bledem dopiero w czasie pracy. */
static esp_err_t check_key(const char *key)
{
    ESP_RETURN_ON_FALSE(key && key[0], ESP_ERR_INVALID_ARG, TAG, "pusty klucz");
    ESP_RETURN_ON_FALSE(strlen(key) <= FLORA_STORE_MAX_KEY_LEN, ESP_ERR_INVALID_ARG, TAG,
                        "klucz \"%s\" ma %u znakow, limit NVS to %d",
                        key, (unsigned)strlen(key), FLORA_STORE_MAX_KEY_LEN);
    return ESP_OK;
}

esp_err_t flora_store_open(const char *ns, flora_store_t *out)
{
    ESP_RETURN_ON_FALSE(s_initialized, ESP_ERR_INVALID_STATE, TAG,
                        "najpierw flora_store_init()");
    ESP_RETURN_ON_FALSE(ns && out, ESP_ERR_INVALID_ARG, TAG, "zly argument");
    ESP_RETURN_ON_ERROR(check_key(ns), TAG, "nazwa przestrzeni");

    flora_store_t h = calloc(1, sizeof(struct flora_store_s));
    ESP_RETURN_ON_FALSE(h, ESP_ERR_NO_MEM, TAG, "brak pamieci");

    esp_err_t ret = nvs_open(ns, NVS_READWRITE, &h->handle);
    if (ret != ESP_OK) {
        free(h);
        ESP_RETURN_ON_ERROR(ret, TAG, "nvs_open(\"%s\")", ns);
    }
    strncpy(h->ns, ns, sizeof(h->ns) - 1);
    *out = h;
    return ESP_OK;
}

void flora_store_close(flora_store_t h)
{
    if (h) {
        nvs_close(h->handle);
        free(h);
    }
}

/* ================= ODCZYT ================= */
/*
 * Brak klucza to normalny przypadek, nie blad - zwracamy ESP_OK z wartoscia
 * domyslna. Bledem jest dopiero problem z samym nosnikiem, i tylko taki
 * jest propagowany do wolajacego.
 */

#define GET_SCALAR(fn, ctype)                                                  \
    ESP_RETURN_ON_FALSE(h && out, ESP_ERR_INVALID_ARG, TAG, "zly argument");   \
    ESP_RETURN_ON_ERROR(check_key(key), TAG, "klucz");                         \
    *out = def;                                                                \
    ctype tmp;                                                                 \
    esp_err_t ret = fn(h->handle, key, &tmp);                                  \
    if (ret == ESP_ERR_NVS_NOT_FOUND) { return ESP_OK; }                       \
    ESP_RETURN_ON_ERROR(ret, TAG, "odczyt \"%s\"", key);                       \
    *out = tmp;                                                                \
    return ESP_OK;

esp_err_t flora_store_get_i32(flora_store_t h, const char *key, int32_t *out, int32_t def)
{
    GET_SCALAR(nvs_get_i32, int32_t)
}

esp_err_t flora_store_get_u32(flora_store_t h, const char *key, uint32_t *out, uint32_t def)
{
    GET_SCALAR(nvs_get_u32, uint32_t)
}

esp_err_t flora_store_get_bool(flora_store_t h, const char *key, bool *out, bool def)
{
    ESP_RETURN_ON_FALSE(h && out, ESP_ERR_INVALID_ARG, TAG, "zly argument");
    ESP_RETURN_ON_ERROR(check_key(key), TAG, "klucz");

    *out = def;
    uint8_t tmp;
    esp_err_t ret = nvs_get_u8(h->handle, key, &tmp);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_OK;
    }
    ESP_RETURN_ON_ERROR(ret, TAG, "odczyt \"%s\"", key);
    *out = (tmp != 0);
    return ESP_OK;
}

esp_err_t flora_store_get_float(flora_store_t h, const char *key, float *out, float def)
{
    ESP_RETURN_ON_FALSE(h && out, ESP_ERR_INVALID_ARG, TAG, "zly argument");
    ESP_RETURN_ON_ERROR(check_key(key), TAG, "klucz");

    *out = def;
    uint32_t bits;
    esp_err_t ret = nvs_get_u32(h->handle, key, &bits);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_OK;
    }
    ESP_RETURN_ON_ERROR(ret, TAG, "odczyt \"%s\"", key);
    /* Przepisanie bitow, nie konwersja - NVS nie ma typu float. */
    memcpy(out, &bits, sizeof(float));
    return ESP_OK;
}

esp_err_t flora_store_get_str(flora_store_t h, const char *key, char *out, size_t cap,
                              const char *def)
{
    ESP_RETURN_ON_FALSE(h && out && cap > 0, ESP_ERR_INVALID_ARG, TAG, "zly argument");
    ESP_RETURN_ON_ERROR(check_key(key), TAG, "klucz");

    out[0] = '\0';
    if (def) {
        strncpy(out, def, cap - 1);
        out[cap - 1] = '\0';
    }

    size_t len = cap;
    esp_err_t ret = nvs_get_str(h->handle, key, out, &len);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_OK;   /* zostaje wartosc domyslna, juz wpisana */
    }
    if (ret == ESP_ERR_NVS_INVALID_LENGTH) {
        /* Zapisany tekst nie miesci sie w buforze. Wartosc domyslna zostala
         * nadpisana czesciowym odczytem, wiec przywracamy ja jawnie. */
        ESP_LOGW(TAG, "\"%s\" nie miesci sie w buforze %u B", key, (unsigned)cap);
        out[0] = '\0';
        if (def) {
            strncpy(out, def, cap - 1);
            out[cap - 1] = '\0';
        }
        return ESP_ERR_NVS_INVALID_LENGTH;
    }
    ESP_RETURN_ON_ERROR(ret, TAG, "odczyt \"%s\"", key);
    return ESP_OK;
}

esp_err_t flora_store_get_blob(flora_store_t h, const char *key, void *out, size_t *len)
{
    ESP_RETURN_ON_FALSE(h && out && len, ESP_ERR_INVALID_ARG, TAG, "zly argument");
    ESP_RETURN_ON_ERROR(check_key(key), TAG, "klucz");

    esp_err_t ret = nvs_get_blob(h->handle, key, out, len);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_ERR_NOT_FOUND;   /* brak domyslnej dla blobu - niech wola decyduje */
    }
    return ret;
}

/* ================= ZAPIS ================= */

#define SET_AND_COMMIT(call)                                        \
    ESP_RETURN_ON_FALSE(h, ESP_ERR_INVALID_ARG, TAG, "h == NULL");  \
    ESP_RETURN_ON_ERROR(check_key(key), TAG, "klucz");              \
    ESP_RETURN_ON_ERROR(call, TAG, "zapis \"%s\"", key);            \
    return nvs_commit(h->handle);

esp_err_t flora_store_set_i32(flora_store_t h, const char *key, int32_t v)
{
    SET_AND_COMMIT(nvs_set_i32(h->handle, key, v))
}

esp_err_t flora_store_set_u32(flora_store_t h, const char *key, uint32_t v)
{
    SET_AND_COMMIT(nvs_set_u32(h->handle, key, v))
}

esp_err_t flora_store_set_bool(flora_store_t h, const char *key, bool v)
{
    SET_AND_COMMIT(nvs_set_u8(h->handle, key, v ? 1 : 0))
}

esp_err_t flora_store_set_float(flora_store_t h, const char *key, float v)
{
    uint32_t bits;
    memcpy(&bits, &v, sizeof(bits));
    SET_AND_COMMIT(nvs_set_u32(h->handle, key, bits))
}

esp_err_t flora_store_set_str(flora_store_t h, const char *key, const char *v)
{
    ESP_RETURN_ON_FALSE(v, ESP_ERR_INVALID_ARG, TAG, "wartosc == NULL");
    SET_AND_COMMIT(nvs_set_str(h->handle, key, v))
}

esp_err_t flora_store_set_blob(flora_store_t h, const char *key, const void *v, size_t len)
{
    ESP_RETURN_ON_FALSE(v && len > 0, ESP_ERR_INVALID_ARG, TAG, "zly bufor");
    SET_AND_COMMIT(nvs_set_blob(h->handle, key, v, len))
}

/* ================= PORZADKI I DIAGNOZA ================= */

bool flora_store_has(flora_store_t h, const char *key)
{
    if (!h || check_key(key) != ESP_OK) {
        return false;
    }
    /* Pytamy o rozmiar z zerowym buforem - interesuje nas sam fakt istnienia.
     * NVS zglasza wtedy ESP_OK albo niezgodna dlugosc, oba znacza "jest". */
    size_t len = 0;
    esp_err_t ret = nvs_get_blob(h->handle, key, NULL, &len);
    if (ret == ESP_OK || ret == ESP_ERR_NVS_INVALID_LENGTH) {
        return true;
    }
    uint32_t dummy;
    return nvs_get_u32(h->handle, key, &dummy) == ESP_OK;
}

esp_err_t flora_store_erase_key(flora_store_t h, const char *key)
{
    ESP_RETURN_ON_FALSE(h, ESP_ERR_INVALID_ARG, TAG, "h == NULL");
    ESP_RETURN_ON_ERROR(check_key(key), TAG, "klucz");

    esp_err_t ret = nvs_erase_key(h->handle, key);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_OK;   /* kasowanie nieistniejacego to nie blad */
    }
    ESP_RETURN_ON_ERROR(ret, TAG, "kasowanie \"%s\"", key);
    return nvs_commit(h->handle);
}

esp_err_t flora_store_erase_namespace(flora_store_t h)
{
    ESP_RETURN_ON_FALSE(h, ESP_ERR_INVALID_ARG, TAG, "h == NULL");
    ESP_LOGW(TAG, "kasuje cala przestrzen \"%s\"", h->ns);
    ESP_RETURN_ON_ERROR(nvs_erase_all(h->handle), TAG, "nvs_erase_all");
    return nvs_commit(h->handle);
}

static const char *type_name(nvs_type_t t)
{
    switch (t) {
        case NVS_TYPE_U8:   return "u8";
        case NVS_TYPE_I8:   return "i8";
        case NVS_TYPE_U16:  return "u16";
        case NVS_TYPE_I16:  return "i16";
        case NVS_TYPE_U32:  return "u32";
        case NVS_TYPE_I32:  return "i32";
        case NVS_TYPE_U64:  return "u64";
        case NVS_TYPE_I64:  return "i64";
        case NVS_TYPE_STR:  return "tekst";
        case NVS_TYPE_BLOB: return "blob";
        default:            return "?";
    }
}

esp_err_t flora_store_dump(flora_store_t h)
{
    ESP_RETURN_ON_FALSE(h, ESP_ERR_INVALID_ARG, TAG, "h == NULL");

    ESP_LOGI(TAG, "zawartosc przestrzeni \"%s\":", h->ns);

    nvs_iterator_t it = NULL;
    esp_err_t ret = nvs_entry_find_in_handle(h->handle, NVS_TYPE_ANY, &it);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "  (pusto)");
        return ESP_OK;
    }
    ESP_RETURN_ON_ERROR(ret, TAG, "nvs_entry_find_in_handle");

    int n = 0;
    while (ret == ESP_OK) {
        nvs_entry_info_t info;
        if (nvs_entry_info(it, &info) == ESP_OK) {
            /* Typ U32 moze byc liczba albo przepisanymi bitami float -
             * pokazujemy obie interpretacje, zeby nie zgadywac. */
            if (info.type == NVS_TYPE_U32) {
                uint32_t raw = 0;
                if (nvs_get_u32(h->handle, info.key, &raw) == ESP_OK) {
                    float as_float;
                    memcpy(&as_float, &raw, sizeof(float));
                    ESP_LOGI(TAG, "  %-16s %-6s %" PRIu32 "  (jako float: %g)",
                             info.key, type_name(info.type), raw, (double)as_float);
                    n++;
                    ret = nvs_entry_next(&it);
                    continue;
                }
            }
            ESP_LOGI(TAG, "  %-16s %s", info.key, type_name(info.type));
            n++;
        }
        ret = nvs_entry_next(&it);
    }
    nvs_release_iterator(it);

    ESP_LOGI(TAG, "  razem %d wpisow", n);
    return ESP_OK;
}

esp_err_t flora_store_report_space(void)
{
    ESP_RETURN_ON_FALSE(s_initialized, ESP_ERR_INVALID_STATE, TAG, "nie zainicjalizowano");

    nvs_stats_t st;
    ESP_RETURN_ON_ERROR(nvs_get_stats(NULL, &st), TAG, "nvs_get_stats");
    ESP_LOGI(TAG, "partycja NVS: %zu z %zu wpisow uzytych, %zu dostepnych na dane, "
                  "%zu przestrzeni nazw",
             st.used_entries, st.total_entries, st.available_entries, st.namespace_count);

    /* Zapelniona partycja objawia sie tym, ze zapisy zaczynaja zwracac blad,
     * a przy nastepnym starcie moze dojsc do skasowania calej zawartosci. */
    if (st.total_entries > 0 && st.available_entries * 10 < st.total_entries) {
        ESP_LOGW(TAG, "mniej niz 10%% wolnego miejsca - rozwaz porzadki");
    }
    return ESP_OK;
}
