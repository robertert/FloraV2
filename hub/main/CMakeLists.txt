idf_component_register(SRCS "main.c" "bringup_drive.c"
                       INCLUDE_DIRS "."
                       REQUIRES flora_board flora_motors flora_encoders flora_ina226
                                flora_pump flora_temp
                                esp_driver_i2c esp_driver_gpio esp_timer)
