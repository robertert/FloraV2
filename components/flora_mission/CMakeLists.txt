# Rdzen bez zaleznosci od ESP-IDF: ma sie kompilowac tez zwyklym cc na
# komputerze, patrz host/test.c.
idf_component_register(SRCS "flora_mission.c"
                       INCLUDE_DIRS "include"
                       REQUIRES flora_seek)
