# Psvrd cmake build script.
cmake_minimum_required(VERSION 2.8)
project(PSVRD C)

# Check the build type
if (CMAKE_BUILD_TYPE STREQUAL "")
	# CMake defaults to leaving CMAKE_BUILD_TYPE empty. This screws up
	# differentiation between debug and release builds.
	set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Choose the type of build, options are: None (CMAKE_C_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif ()

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# Special checks for Emscripten
if(${CMAKE_SYSTEM_NAME} STREQUAL Emscripten)
    set(ON_EMSCRIPTEN TRUE)
endif()

# Output paths.

set(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_BINARY_DIR}/dist")
set(PSVRD_DATA_PREFIX "${CMAKE_CURRENT_BINARY_DIR}/dist/data")
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/;${CMAKE_MODULE_PATH}")

# Turn on warnings.
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -pthread")

# Perform platform checks
include(${CMAKE_ROOT}/Modules/CheckIncludeFile.cmake)
include(${CMAKE_ROOT}/Modules/CheckFunctionExists.cmake)
include(${CMAKE_ROOT}/Modules/CheckLibraryExists.cmake)

# Set output dir.
set(EXECUTABLE_OUTPUT_PATH "${PSVRD_BINARY_DIR}/dist")
set(LIBRARY_OUTPUT_PATH "${PSVRD_BINARY_DIR}/dist")

# Find libusb
find_library(LIBUSB_LIB usb-1.0)
find_path(LIBUSB_INCLUDE_PATH libusb.h PATH_SUFFIXES libusb-1.0)
if(NOT LIBUSB_LIB OR NOT LIBUSB_INCLUDE_PATH)
    message(FATAL_ERROR "Failed to find libusb.")
endif()

# Add the current directory.
include_directories(
.
"${PROJECT_SOURCE_DIR}/include"
"${LIBUSB_INCLUDE_PATH}"
)

# Build the program
add_subdirectory(psvrd)
