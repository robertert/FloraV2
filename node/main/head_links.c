/**
 * @file head_links.c
 * @brief Dwa lacza glowicy masztu i przekazywanie ramek modulu kamery.
 *
 * Glowica jest posrodku lancucha:
 *
 *   MODUL ZNACZNIKOW --UART2--> GLOWICA --UART1--> BRYLA
 *
 * W maszcie biegnie tylko jeden kabel w dol, wiec modul kamery znacznikow
 * nie ma wlasnego polaczenia z bryla. Glowica przekazuje jego ramki dalej.
 *
 * Przekazywanie dziala BEZ pola adresu w ramce, bo typy wiadomosci sa
 * globalnie unikalne - bryla rozpoznaje nadawce po samym typie. Dzieki
 * temu protokol nie wymagal przebudowy, tylko obslugi wielu lacz naraz.
 */
#include <inttypes.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "head_links.h"
#include "flora_head_board.h"
#include "flora_link.h"
#include "flora_msgs.h"

static const char *TAG = "lacza";

static flora_link_handle_t s_down;   /**< do bryly */
static flora_link_handle_t s_up;     /**< do modulu znacznikow */

static uint32_t s_tags_relayed;

flora_link_handle_t head_link_down(void) { return s_down; }
flora_link_handle_t head_link_up(void)   { return s_up; }

static void on_down_state(bool up, void *ctx)
{
    ESP_LOGI(TAG, "bryla: %s", up ? "polaczona" : "rozlaczona");
    if (!up) {
        return;
    }
    /* Bryla nie ufa danym glowicy przed odebraniem HELLO. Mape czujnikow
     * wypelni warstwa wyzej, gdy bedzie znac wynik skanu I2C. */
    flora_hello_t hello = {
        .msgs_version    = FLORA_MSGS_VERSION,
        .sensors_present = 0,
        .fw_build        = 0,
    };
    flora_link_send(s_down, FLORA_MSG_HELLO, &hello, sizeof hello);
}

static void on_up_state(bool up, void *ctx)
{
    ESP_LOGI(TAG, "modul znacznikow: %s", up ? "polaczony" : "rozlaczony");
}

/**
 * Przekazuje detekcje w dol i po drodze je liczy.
 *
 * Wlasny handler zamiast flora_link_relay, bo flora_link_on nadpisuje
 * handler tego samego typu - podgladu i przekazywania nie da sie wpiac
 * osobno. Ten robi oba naraz.
 */
static void relay_tags(uint8_t type, const void *payload, uint8_t len, void *ctx)
{
    flora_link_send(s_down, type, payload, len);
    s_tags_relayed++;

    if (len != sizeof(flora_tags_t)) {
        return;
    }
    const flora_tags_t *t = payload;
    if (t->count > 0) {
        ESP_LOGI(TAG, "przekazano %u znacznik(ow), pierwszy: id %u, %u mm, %.1f st",
                 t->count, t->tags[0].id, t->tags[0].distance_mm,
                 t->tags[0].bearing_cdeg / 100.0f);
    }
}

esp_err_t head_links_init(void)
{
    /* --- lacze w dol, do bryly --- */
    flora_link_cfg_t down = FLORA_LINK_DEFAULT_CFG();
    down.name          = "dol";
    down.port          = FLORA_HEAD_UART_PORT;
    down.tx_gpio       = FLORA_HEAD_UART_TX;
    down.rx_gpio       = FLORA_HEAD_UART_RX;
    down.proto_version = FLORA_MSGS_VERSION;
    down.size_of       = flora_msgs_payload_size;
    down.ack_type      = FLORA_MSG_ACK;
    down.on_state      = on_down_state;

    esp_err_t ret = flora_link_init(&down, &s_down);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "lacze w dol nie wystartowalo: %s", esp_err_to_name(ret));
        return ret;
    }

    /* --- lacze w gore, do modulu znacznikow ---
     * Krotki odcinek w obrebie masztu, wiec mozna szybciej niz w dol. */
    flora_link_cfg_t up = FLORA_LINK_DEFAULT_CFG();
    up.name          = "gora";
    up.port          = FLORA_HEAD_CAM_PORT;
    up.tx_gpio       = FLORA_HEAD_CAM_TX;
    up.rx_gpio       = FLORA_HEAD_CAM_RX;
    up.baud_rate     = 115200;
    up.proto_version = FLORA_MSGS_VERSION;
    up.size_of       = flora_msgs_payload_size;
    up.ack_type      = FLORA_MSG_ACK;
    up.on_state      = on_up_state;

    ret = flora_link_init(&up, &s_up);
    if (ret != ESP_OK) {
        /* Brak modulu znacznikow nie blokuje glowicy - czujniki i kamera
         * przednia dzialaja dalej, tracimy tylko korekte pozycji. */
        ESP_LOGW(TAG, "lacze w gore nie wystartowalo (%s) - modul znacznikow "
                      "bedzie niedostepny", esp_err_to_name(ret));
        s_up = NULL;
        return ESP_OK;
    }

    /* Ramki modulu kamery leca prosto w dol. HELLO modulu NIE jest
     * przekazywane - bryla ma dostac HELLO glowicy, nie dwa naraz;
     * obecnosc modulu zdradza sam fakt naplywania jego ramek. */
    flora_link_on(s_up, FLORA_MSG_TAGS, relay_tags, NULL);
    flora_link_relay(s_up, s_down, FLORA_MSG_CAM_STATUS);

    ESP_LOGI(TAG, "glowica posrodku lancucha: modul znacznikow -> glowica -> bryla");
    return ESP_OK;
}

uint32_t head_links_tags_relayed(void) { return s_tags_relayed; }
