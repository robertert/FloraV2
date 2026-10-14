idf_component_register(SRCS "flora_encoders.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_driver_gpio
                       PRIV_REQUIRES esp_driver_pcnt esp_timer)
