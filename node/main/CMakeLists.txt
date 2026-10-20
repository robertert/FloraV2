idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES flora_head_board flora_i2cmux flora_veml7700
                                flora_bme280 flora_icm20948
                                esp_driver_i2c)
