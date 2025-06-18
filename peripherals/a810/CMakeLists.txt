include_guard()

# Specify the minimum required version of CMake
cmake_minimum_required(VERSION 3.21)

# Add the a810 peripheral as a shared library
add_library(a810 SHARED
    src/a810.cpp
)

# Define the ZM_EXTERNAL_PERIPHERAL_EXPORT macro
target_compile_definitions(a810
    PRIVATE
        ZM_EXTERNAL_PERIPHERAL_EXPORT
)

# Specify the public and private include directories
target_include_directories(a810
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../../external
        ${CMAKE_CURRENT_SOURCE_DIR}/../../external/imgui
        ${CMAKE_CURRENT_SOURCE_DIR}/../../external/json/include
)

# Specify the source file making up the a810 peripheral


# Link imgui_glfw and logging_system to the a810 library so the peripheral can
# render itself and log messages to the logging system
target_link_libraries(a810
    PRIVATE
        imgui_glfw
        logging_system
)

# Copy the shared library into the output/peripherals directory
set(output_directory ${PROJECT_SOURCE_DIR}/output/peripherals)

add_custom_command(TARGET a810
    POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory ${output_directory}
        COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:a810> ${output_directory}
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

    target_link_libraries(a810_tests PRIVATE legacy_stdio_definitions.lib)

    add_executable(a810_tests
    tests/a810_tests.cpp 
)

    target_include_directories(a810_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../../include
        ${CMAKE_CURRENT_SOURCE_DIR}/../../external/imgui
        ${CMAKE_CURRENT_SOURCE_DIR}/../../external/json/include
        ${PROJECT_SOURCE_DIR}/src/core/peripherals

    )

    target_compile_definitions(a810_tests
        PRIVATE
            TEMPLATE_LOGGER=MockLogger
    )

    target_compile_options(a810_tests PRIVATE
        -Wno-comment
    )

    target_link_libraries(a810_tests
        PRIVATE
            a810
            GTest::gtest_main
            logging_system
            mock_bsc
            mock_gpio
    )

    add_test(
        NAME a810_tests
        COMMAND $<TARGET_FILE:a810_tests>
    )
endif()