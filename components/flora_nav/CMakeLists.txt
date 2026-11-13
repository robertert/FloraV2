# Rdzen bez zaleznosci: ma sie kompilowac tez zwyklym cc na komputerze,
# patrz host/test.c.
idf_component_register(SRCS "flora_ekf.c" "flora_sun.c" "flora_sungate.c" "flora_navcore.c"
                       INCLUDE_DIRS "include")
