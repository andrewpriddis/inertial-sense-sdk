CMAKE_MINIMUM_REQUIRED(VERSION 2.8.13)

# Required: set the project
project(ISNtripRoverExample)

# Required: set SOURCES_PROJECT to the code in this project, the files must go on their own lines
set(SOURCES_PROJECT	"${CMAKE_CURRENT_LIST_DIR}/ISNtripRoverExample.cpp")

# Optional: add any libraries this project needs
set(PROJECT_LIBS "pigpio")

find_library(PIGPIO_LIBRARY NAMES pigpio PATHS /usr/lib /usr/local/lib)
if (NOT PIGPIO_LIBRARY)
    message(FATAL_ERROR "pigpio library not found")
endif()


# Everything is setup, use common template to build
include(../../CMakeCommon.txt)
