idf_component_register(SRCS "flora_ota.c"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES flora_board flora_mqtt flora_store
                                     esp_https_ota esp_http_client app_update
                                     esp_partition esp_app_format esp_timer mbedtls)
