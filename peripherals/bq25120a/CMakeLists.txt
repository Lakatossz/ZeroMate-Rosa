include_guard()

# Specify the minimum required version of CMake
cmake_minimum_required(VERSION 3.21)

# Add the bq25120a peripheral as a shared library
add_library(bq25120a SHARED
    src/bq25120a.cpp
)

# Define the ZM_EXTERNAL_PERIPHERAL_EXPORT macro
target_compile_definitions(bq25120a
    PRIVATE
        ZM_EXTERNAL_PERIPHERAL_EXPORT
)

# Specify the public and private include directories
target_include_directories(bq25120a
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../../external
        ${CMAKE_CURRENT_SOURCE_DIR}/../../external/imgui
        ${CMAKE_CURRENT_SOURCE_DIR}/../../external/json/include
)

# Specify the source file making up the bq25120a peripheral


# Link imgui_glfw and logging_system to the bq25120a library so the peripheral can
# render itself and log messages to the logging system
target_link_libraries(bq25120a
    PRIVATE
        imgui_glfw
        logging_system
)

# Copy the shared library into the output/peripherals directory
set(output_directory ${PROJECT_SOURCE_DIR}/output/peripherals)

add_custom_command(TARGET bq25120a
    POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory ${output_directory}
        COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:bq25120a> ${output_directory}
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

    add_executable(bq25120a_tests
    tests/bq25120a_tests.cpp 
)

    target_include_directories(bq25120a_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../../include
        ${CMAKE_CURRENT_SOURCE_DIR}/../../external/imgui
        ${CMAKE_CURRENT_SOURCE_DIR}/../../external/json/include
        ${PROJECT_SOURCE_DIR}/src/core/peripherals

    )

    target_compile_definitions(bq25120a_tests
        PRIVATE
            TEMPLATE_LOGGER=MockLogger
    )

    target_compile_options(bq25120a_tests PRIVATE
        -Wno-comment
    )

    target_link_libraries(bq25120a_tests
        PRIVATE
            bq25120a
            GTest::gtest_main
            logging_system
            mock_bsc
            mock_gpio
    )

    add_test(
        NAME bq25120a_tests
        COMMAND $<TARGET_FILE:bq25120a_tests>
    )
endif()