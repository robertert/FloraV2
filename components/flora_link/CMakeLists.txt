# esp_driver_uart w REQUIRES, bo uart_port_t jest w publicznym naglowku.
idf_component_register(SRCS "flora_link.c" "flora_link_frame.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_driver_uart
                       PRIV_REQUIRES esp_timer)
