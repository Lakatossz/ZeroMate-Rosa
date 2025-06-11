include_guard()

# Specify the minimum required version of CMake
cmake_minimum_required(VERSION 3.21)

# Add the mock_GPIO as a shared library
add_library(mock_gpio SHARED)

# Define the ZM_EXTERNAL_PERIPHERAL_EXPORT macro
target_compile_definitions(mock_gpio
    PRIVATE
        ZM_MOCK_GPIO_EXPORT
)

# Specify the public include directories
target_include_directories(mock_gpio
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../../include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
)

# Specify the source file making up the mock_gpio library
target_sources(mock_gpio
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/mock_gpio.cpp
)

# Copy the library (.dll) to the output directory
set(output_directory ${PROJECT_SOURCE_DIR}/output)

add_custom_command(TARGET mock_gpio
    POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory ${output_directory}
        COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:mock_gpio> ${output_directory}
)