#include <stdlib.h>
#include <string.h>

#include "flora_i2cmux.h"
#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "flora_i2cmux";

struct flora_i2cmux_s {
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t dev;
    uint8_t                 current_mask;
    bool                    mask_known;
};

esp_err_t flora_i2cmux_init(i2c_master_bus_handle_t bus, uint8_t address,
                            uint32_t speed_hz, flora_i2cmux_handle_t *out)
{
    ESP_RETURN_ON_FALSE(bus && out, ESP_ERR_INVALID_ARG, TAG, "zly argument");

    flora_i2cmux_handle_t h = calloc(1, sizeof(struct flora_i2cmux_s));
    ESP_RETURN_ON_FALSE(h, ESP_ERR_NO_MEM, TAG, "brak pamieci");
    h->bus = bus;

    esp_err_t ret = ESP_OK;
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = address,
        .scl_speed_hz    = speed_hz,
    };
    ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(bus, &dev_cfg, &h->dev),
                      fail, TAG, "dodanie urzadzenia 0x%02X", address);

    /* TCA9548A nie ma rejestru identyfikacyjnego, wiec jedyne co mozemy
     * sprawdzic to czy w ogole potwierdza transmisje. */
    uint8_t zero = 0;
    ESP_GOTO_ON_FALSE(i2c_master_transmit(h->dev, &zero, 1, 100) == ESP_OK,
                      ESP_ERR_NOT_FOUND, fail, TAG,
                      "pod adresem 0x%02X nikt nie odpowiada - sprawdz zasilanie, "
                      "podciagniecia i pin RESET multipleksera", address);
    h->current_mask = 0;
    h->mask_known   = true;

    ESP_LOGI(TAG, "TCA9548A @0x%02X gotowy", address);
    *out = h;
    return ESP_OK;

fail:
    if (h->dev) {
        i2c_master_bus_rm_device(h->dev);
    }
    free(h);
    return ret;
}

esp_err_t flora_i2cmux_deinit(flora_i2cmux_handle_t h)
{
    ESP_RETURN_ON_FALSE(h, ESP_ERR_INVALID_ARG, TAG, "h == NULL");
    uint8_t zero = 0;
    i2c_master_transmit(h->dev, &zero, 1, 100);   /* zostawiamy odlaczone */
    i2c_master_bus_rm_device(h->dev);
    free(h);
    return ESP_OK;
}

static esp_err_t write_mask(flora_i2cmux_handle_t h, uint8_t mask)
{
    /* Ten sam kanal co poprzednio - nie ma po co zajmowac magistrali. */
    if (h->mask_known && h->current_mask == mask) {
        return ESP_OK;
    }
    ESP_RETURN_ON_ERROR(i2c_master_transmit(h->dev, &mask, 1, 100),
                        TAG, "zapis maski 0x%02X", mask);
    h->current_mask = mask;
    h->mask_known   = true;
    return ESP_OK;
}

esp_err_t flora_i2cmux_select(flora_i2cmux_handle_t h, uint8_t channel)
{
    ESP_RETURN_ON_FALSE(h, ESP_ERR_INVALID_ARG, TAG, "h == NULL");
    ESP_RETURN_ON_FALSE(channel < FLORA_I2CMUX_CHANNELS, ESP_ERR_INVALID_ARG, TAG,
                        "kanal %u poza zakresem 0-%d", channel, FLORA_I2CMUX_CHANNELS - 1);
    return write_mask(h, (uint8_t)(1u << channel));
}

esp_err_t flora_i2cmux_disable_all(flora_i2cmux_handle_t h)
{
    ESP_RETURN_ON_FALSE(h, ESP_ERR_INVALID_ARG, TAG, "h == NULL");
    return write_mask(h, 0);
}

esp_err_t flora_i2cmux_get_mask(flora_i2cmux_handle_t h, uint8_t *mask)
{
    ESP_RETURN_ON_FALSE(h && mask, ESP_ERR_INVALID_ARG, TAG, "zly argument");
    return i2c_master_receive(h->dev, mask, 1, 100);
}

esp_err_t flora_i2cmux_port_select(void *port)
{
    flora_i2cmux_port_t *p = (flora_i2cmux_port_t *)port;
    ESP_RETURN_ON_FALSE(p && p->mux, ESP_ERR_INVALID_ARG, TAG, "zly port");
    return flora_i2cmux_select(p->mux, p->channel);
}

esp_err_t flora_i2cmux_scan(flora_i2cmux_handle_t h)
{
    ESP_RETURN_ON_FALSE(h, ESP_ERR_INVALID_ARG, TAG, "h == NULL");

    ESP_LOGI(TAG, "skanowanie kanalow multipleksera:");
    for (uint8_t ch = 0; ch < FLORA_I2CMUX_CHANNELS; ch++) {
        ESP_RETURN_ON_ERROR(flora_i2cmux_select(h, ch), TAG, "wybor kanalu %u", ch);

        int found = 0;
        for (uint8_t addr = 0x08; addr < 0x78; addr++) {
            /* Sam multiplekser odpowiada niezaleznie od kanalu - pominiecie
             * go oszczedza mylacego wpisu przy kazdej galezi. */
            if (i2c_master_probe(h->bus, addr, 20) == ESP_OK) {
                ESP_LOGI(TAG, "  kanal %u: 0x%02X", ch, addr);
                found++;
            }
        }
        if (found == 0) {
            ESP_LOGI(TAG, "  kanal %u: pusto", ch);
        }
    }
    return flora_i2cmux_disable_all(h);
}
