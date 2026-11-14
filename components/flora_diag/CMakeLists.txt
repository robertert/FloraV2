idf_component_register(SRCS "flora_diag.c"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES flora_board flora_mqtt flora_store
                                     app_update esp_app_format esp_timer)
