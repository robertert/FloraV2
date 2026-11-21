# Rdzen bez zaleznosci od ESP-IDF (poza naglowkiem kontraktu), test w host/test.c.
idf_component_register(SRCS "flora_tunnel.c"
                       INCLUDE_DIRS "include"
                       REQUIRES flora_msgs)
