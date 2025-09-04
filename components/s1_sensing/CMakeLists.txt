# Registers the component with the ESP-IDF build system.
idf_component_register(SRCS "s1_sensing.c"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_driver_gpio esp_driver_i2c esp_driver_adc esp_timer)
