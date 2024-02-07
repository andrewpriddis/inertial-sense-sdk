cmake_minimum_required(VERSION 3.0)
project(LED_Control)

# Find pigpio library
find_library(PIGPIO_LIBRARY NAMES pigpio)

if(NOT PIGPIO_LIBRARY)
    message(FATAL_ERROR "pigpio library not found")
endif()

# Include pigpio header files
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Add your source files
add_executable(led_control main.cpp)

# Link pigpio library
target_link_libraries(led_control ${PIGPIO_LIBRARY})