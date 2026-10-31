idf_component_register(SRCS "main.c" "bringup_link.c" "bringup_head.c"
                            "head_links.c" "head_cmd.c" "head_fusion.c" "head_slow.c"
                       INCLUDE_DIRS "."
                       REQUIRES flora_head_board flora_i2cmux flora_veml7700
                                flora_bme280 flora_icm20948
                                flora_link flora_msgs flora_store
                                esp_driver_i2c esp_timer)
