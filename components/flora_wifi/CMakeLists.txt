# esp_netif w REQUIRES, bo esp_ip4_addr_t jest w publicznym naglowku.
idf_component_register(SRCS "flora_wifi.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_netif
                       PRIV_REQUIRES esp_wifi esp_event esp_timer esp_hw_support)
