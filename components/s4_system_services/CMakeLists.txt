# Registers the component with the ESP-IDF build system.
idf_component_register(SRCS "s4_system_services.c"
                    INCLUDE_DIRS "include"
                    REQUIRES nvs_flash esp_timer)
