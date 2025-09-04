# This registers the main source file of the application.
# The ESP-IDF build system requires this file in the main component directory.
idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS ".")
