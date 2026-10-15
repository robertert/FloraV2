# Zaleznosc od ds18b20 jest PRYWATNA: nasz publiczny naglowek nie wystawia
# zadnego typu z tamtego komponentu, wiec aplikacja nie musi go znac.
idf_component_register(SRCS "flora_temp.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_driver_gpio
                       PRIV_REQUIRES espressif__ds18b20 espressif__onewire_bus)
