idf_component_register(SRCS "main.c" "bringup_link.c" "head_links.c"
                       INCLUDE_DIRS "."
                       REQUIRES flora_head_board flora_i2cmux flora_veml7700
                                flora_bme280 flora_icm20948
                                flora_link flora_msgs
                                esp_driver_i2c esp_timer)
