include_guard()

# Specify the minimum required version of CMake
cmake_minimum_required(VERSION 3.21)

# Add the mock_bsc as a shared library
add_library(mock_bsc SHARED)

# Definuj exportní makro pro hlavní projekt (pokud ještě není v hlavním CMakeLists.txt)
target_compile_definitions(mock_bsc
    PRIVATE
        ZM_MOCK_BSC_EXPORT
)

# Specify the public include directories
target_include_directories(mock_bsc
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../../include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
)

# Specify the source file making up the mock_bsc library
target_sources(mock_bsc
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/mock_bsc.cpp
)

# Copy the library (.dll) to the output directory
set(output_directory ${PROJECT_SOURCE_DIR}/output)

add_custom_command(TARGET mock_bsc
    POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory ${output_directory}
        COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:mock_bsc> ${output_directory}
)