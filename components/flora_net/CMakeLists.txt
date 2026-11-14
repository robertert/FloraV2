idf_component_register(SRCS "flora_net.c"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES flora_board flora_mqtt flora_store flora_wifi)
