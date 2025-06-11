include_guard()

# Specify the minimum required version of CMake
cmake_minimum_required(VERSION 3.21)

# Add the reservoir peripheral as a shared library
add_library(reservoir SHARED)

# Define the ZM_EXTERNAL_PERIPHERAL_EXPORT macro
target_compile_definitions(reservoir
    PRIVATE
        ZM_EXTERNAL_PERIPHERAL_EXPORT
)

# Specify the public and private include directories
target_include_directories(reservoir
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../../external
        ${CMAKE_CURRENT_SOURCE_DIR}/../../external/imgui
)

# Specify the source file making up the reservoir peripheral
target_sources(reservoir
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src/reservoir.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/reservoir.cpp
)

# Link imgui_glfw and logging_system to the reservoir library so the peripheral can
# render itself and log messages to the logging system
target_link_libraries(reservoir
    PRIVATE
        imgui_glfw
        logging_system
)

# Copy the shared library into the output/peripherals directory
set(output_directory ${PROJECT_SOURCE_DIR}/output/peripherals)

add_custom_command(TARGET
    reservoir POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory ${output_directory}
        COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:reservoir> ${output_directory}
)