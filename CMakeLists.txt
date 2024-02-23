cmake_minimum_required(VERSION 3.13)

project(quaduped_vision_rpc_example)

cmake_policy(SET CMP0077 NEW)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/proto)
link_directories(${CMAKE_SOURCE_DIR}/lib)
# include_directories(/usr/local/include)
# link_directories(/usr/local/lib)
include_directories($ENV{HOME}/.local/include)
link_directories($ENV{HOME}/.local/lib)


# --------------------------------------------------------
# Build examples
# --------------------------------------------------------
add_subdirectory(examples)