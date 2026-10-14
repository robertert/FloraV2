idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES flora_board flora_motors flora_encoders flora_ina226
                                esp_driver_i2c)
