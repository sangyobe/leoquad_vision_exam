cmake_minimum_required(VERSION 3.13)

project(quaduped_vision_rpc_example)

cmake_policy(SET CMP0077 NEW)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# check if ARTF_INSTALL_DIR environment variable is set.
if ("$ENV{ARTF_INSTALL_DIR}" STREQUAL "")
    message(FATAL_ERROR "ARTF_INSTALL_DIR is not set.")
else()
    message(STATUS "ARTF_INSTALL_DIR: $ENV{ARTF_INSTALL_DIR}")
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

include_directories(${CMAKE_SOURCE_DIR}/proto)
include_directories($ENV{ARTF_INSTALL_DIR}/include)
link_directories($ENV{ARTF_INSTALL_DIR}/lib)

# --------------------------------------------------------
# Build examples
# --------------------------------------------------------
add_subdirectory(examples)