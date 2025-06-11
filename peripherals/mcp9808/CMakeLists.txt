# Specify the minimum required version of CMake
cmake_minimum_required(VERSION 3.21)

include_guard()

# Øešení varování CMP0135
if(POLICY CMP0135)
    cmake_policy(SET CMP0135 NEW)
endif()

# Add the mcp9808 peripheral as a shared library
add_library(mcp9808 SHARED
    src/mcp9808.cpp
)

# Define the ZM_EXTERNAL_PERIPHERAL_EXPORT macro
target_compile_definitions(mcp9808
    PRIVATE
        ZM_EXTERNAL_PERIPHERAL_EXPORT
)

# Specify the public and private include directories
target_include_directories(mcp9808
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../../external
        ${CMAKE_CURRENT_SOURCE_DIR}/../../external/imgui
        ${CMAKE_CURRENT_SOURCE_DIR}/../../external/json/include
)

# Link imgui_glfw and logging_system to the mcp9808 library so the peripheral can
# render itself and log messages to the logging system
target_link_libraries(mcp9808
    PRIVATE
        imgui_glfw
        logging_system
)

# Copy the shared library into the output/peripherals directory
set(output_directory ${PROJECT_SOURCE_DIR}/output/peripherals)

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

    add_executable(mcp9808_tests
        tests/mcp9808_tests.cpp 
    )

    target_include_directories(mcp9808_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../../include
        ${CMAKE_SOURCE_DIR}/external/fakeit/include
        ${CMAKE_SOURCE_DIR}/external/fakeit/config/gtest
        ${CMAKE_CURRENT_SOURCE_DIR}/../../external/imgui
        ${CMAKE_CURRENT_SOURCE_DIR}/../../external/json/include
    )

    target_compile_definitions(mcp9808_tests
        PRIVATE
            TEMPLATE_LOGGER=MockLogger
    )

    target_compile_options(mcp9808_tests PRIVATE
        -Wno-comment
    )

    target_link_libraries(mcp9808_tests
        PRIVATE
            mcp9808
            GTest::gtest_main
            logging_system
            mock_bsc
            mock_gpio
    )

    add_test(
        NAME mcp9808_tests
        COMMAND $<TARGET_FILE:mcp9808_tests>
    )
endif()