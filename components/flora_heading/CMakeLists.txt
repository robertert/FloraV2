# Rdzen bez zaleznosci: ma sie kompilowac tez zwyklym cc na komputerze,
# patrz host/test.c.
idf_component_register(SRCS "flora_magcal.c" "flora_hkf.c" "flora_magdist.c"
                       INCLUDE_DIRS "include")
