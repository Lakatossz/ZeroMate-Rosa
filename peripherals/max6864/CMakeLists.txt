cmake_minimum_required(VERSION 3.21)
include_guard()

# Øešení varování CMP0135
if(POLICY CMP0135)
    cmake_policy(SET CMP0135 NEW)
endif()

# ------------------ MAX6864 PERIPHERAL ------------------
add_library(max6864 SHARED 
    src/max6864.cpp
)

target_compile_definitions(max6864 
    PRIVATE 
        ZM_EXTERNAL_PERIPHERAL_EXPORT
)

target_include_directories(max6864
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../../external
        ${CMAKE_CURRENT_SOURCE_DIR}/../../external/imgui
        ${CMAKE_CURRENT_SOURCE_DIR}/../../external/json/include
)

target_link_libraries(max6864
    PRIVATE
        imgui_glfw
        logging_system  # Pøedpokládá, že logging_system je definován jinde
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

    add_executable(max6864_tests
        tests/max6864_tests.cpp  # ODSTRANÌN src/logger_stdo.cpp!
    )

    target_include_directories(max6864_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../../include
        ${CMAKE_SOURCE_DIR}/external/fakeit/include
        ${CMAKE_SOURCE_DIR}/external/fakeit/config/gtest
        ${CMAKE_CURRENT_SOURCE_DIR}/../../external/imgui
        ${CMAKE_CURRENT_SOURCE_DIR}/../../external/json/include
    )

    target_compile_definitions(max6864_tests
        PRIVATE
            TEMPLATE_LOGGER=MockLogger
    )

    target_compile_options(max6864_tests PRIVATE
        -Wno-comment
    )

    target_link_libraries(max6864_tests
        PRIVATE
            max6864
            GTest::gtest_main
            logging_system
    )

    add_test(
        NAME max6864_tests
        COMMAND $<TARGET_FILE:max6864_tests>
    )
endif()
