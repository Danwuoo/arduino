# Registers the component with the ESP-IDF build system.
# SRCS: Lists the source files for this component.
# INCLUDE_DIRS: Specifies the directory containing public header files.
idf_component_register(SRCS "s0_power_protection.c"
                    INCLUDE_DIRS "include")
