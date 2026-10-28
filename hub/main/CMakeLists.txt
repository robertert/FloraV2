idf_component_register(SRCS "main.c" "bringup_drive.c" "bringup_hydro.c" "bringup_sensors.c" "bringup_net.c"
                       INCLUDE_DIRS "."
                       REQUIRES flora_board flora_motors flora_encoders flora_ina226
                                flora_pump flora_temp flora_soil flora_sonar
                                flora_link flora_msgs flora_store flora_mqtt flora_wifi flora_pwrsw flora_level
                                esp_driver_i2c esp_driver_gpio esp_timer)
