cmake_minimum_required(VERSION 3.16)

# Define the project name, which will be used for the final binary output.
project(esp32_control_system C)

# Include the main ESP-IDF project cmake file. This line is crucial as it
# incorporates all the build logic, toolchain configurations, and component
# management provided by the framework.
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
