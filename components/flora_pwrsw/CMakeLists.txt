idf_component_register(SRCS "flora_pwrsw.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_driver_gpio
                       PRIV_REQUIRES esp_timer)
