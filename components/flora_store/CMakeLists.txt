# nvs_flash prywatnie - publiczny naglowek nie wystawia zadnego typu z NVS,
# wiec aplikacja nie musi go znac.
idf_component_register(SRCS "flora_store.c"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES nvs_flash)
