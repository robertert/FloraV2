idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES flora_cam_board flora_link flora_msgs
                                flora_camera flora_ground esp_psram
                                esp_timer)
