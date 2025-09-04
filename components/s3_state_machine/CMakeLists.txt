# Registers the component with the ESP-IDF build system.
idf_component_register(SRCS "s3_state_machine.c"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_timer)
