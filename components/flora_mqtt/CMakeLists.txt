idf_component_register(SRCS "flora_mqtt.c"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES espressif__mqtt esp_event mbedtls)
