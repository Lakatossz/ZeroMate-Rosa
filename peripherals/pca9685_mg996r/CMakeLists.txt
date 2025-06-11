include_guard()

# Specify the minimum required version of CMake
cmake_minimum_required(VERSION 3.21)

# Add the pca9685_mg996r peripheral as a shared library
add_library(pca9685_mg996r SHARED
    src/pca9685_mg996r.cpp
)

# Define the ZM_EXTERNAL_PERIPHERAL_EXPORT macro
target_compile_definitions(pca9685_mg996r
    PRIVATE
        ZM_EXTERNAL_PERIPHERAL_EXPORT
)

# Specify the public and private include directories
target_include_directories(pca9685_mg996r
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../../external
        ${CMAKE_CURRENT_SOURCE_DIR}/../../external/imgui
        ${CMAKE_CURRENT_SOURCE_DIR}/../../external/json/include
)

# Specify the source file making up the pca9685_mg996r peripheral


# Link imgui_glfw and logging_system to the pca9685_mg996r library so the peripheral can
# render itself and log messages to the logging system
target_link_libraries(pca9685_mg996r
    PRIVATE
        imgui_glfw
        logging_system
)

# Copy the shared library into the output/peripherals directory
set(output_directory ${PROJECT_SOURCE_DIR}/output/peripherals)

add_custom_command(TARGET pca9685_mg996r
    POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory ${output_directory}
        COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:pca9685_mg996r> ${output_directory}
)

# ------------------ TESTING ------------------
if(BUILD_TESTING)
    include(CTest)
    enable_testing()

    include(FetchContent)
    FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/refs/tags/v1.13.0.zip
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    add_executable(pca9685_mg996r_tests
    tests/pca9685_mg996r_tests.cpp 
)

    target_include_directories(pca9685_mg996r_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../../include
        ${CMAKE_CURRENT_SOURCE_DIR}/../../external/imgui
        ${CMAKE_CURRENT_SOURCE_DIR}/../../external/json/include
        ${PROJECT_SOURCE_DIR}/src/core/peripherals

    )

    target_compile_definitions(pca9685_mg996r_tests
        PRIVATE
            TEMPLATE_LOGGER=MockLogger
    )

    target_compile_options(pca9685_mg996r_tests PRIVATE
        -Wno-comment
    )

    target_link_libraries(pca9685_mg996r_tests
        PRIVATE
            pca9685_mg996r
            GTest::gtest_main
            logging_system
            mock_bsc
            mock_gpio
    )

    add_test(
        NAME pca9685_mg996r_tests
        COMMAND $<TARGET_FILE:pca9685_mg996r_tests>
    )
endif()