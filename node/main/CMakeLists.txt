idf_component_register(SRCS "main.c" "bringup_link.c" "bringup_head.c" "bringup_cam.c"
                            "head_links.c" "head_cmd.c" "head_fusion.c" "head_slow.c"
                            "head_obstacles.c"
                       INCLUDE_DIRS "."
                       REQUIRES flora_head_board flora_i2cmux flora_veml7700
                                flora_bme280 flora_icm20948
                                flora_link flora_msgs flora_store flora_camera flora_ground
                                flora_board flora_wifi
                                esp_driver_i2c esp_timer esp_http_server)
