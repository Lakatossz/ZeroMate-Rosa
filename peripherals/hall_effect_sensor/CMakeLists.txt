include_guard()

# Specify the minimum required version of CMake
cmake_minimum_required(VERSION 3.21)

# Add the hall_effect_sensor peripheral as a shared library
add_library(hall_effect_sensor SHARED)

# Define the ZM_EXTERNAL_PERIPHERAL_EXPORT macro
target_compile_definitions(hall_effect_sensor
    PRIVATE
        ZM_EXTERNAL_PERIPHERAL_EXPORT
)

# Specify the public and private include directories
target_include_directories(hall_effect_sensor
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../../external
)

# Specify the source file making up the hall_effect_sensor peripheral
target_sources(hall_effect_sensor
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src/hall_effect_sensor.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/hall_effect_sensor.cpp
)

# Link imgui_glfw and logging_system to the hall_effect_sensor library so the peripheral can
# render itself and log messages to the logging system
target_link_libraries(hall_effect_sensor
    PRIVATE
        logging_system
)

# Copy the shared library into the output/peripherals directory
set(output_directory ${PROJECT_SOURCE_DIR}/output/peripherals)

add_custom_command(TARGET
    hall_effect_sensor POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory ${output_directory}
        COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:hall_effect_sensor> ${output_directory}
)