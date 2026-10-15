idf_component_register(SRCS "flora_pump.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_driver_gpio esp_driver_ledc
                       PRIV_REQUIRES esp_timer)
