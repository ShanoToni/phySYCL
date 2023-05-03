cmake_minimum_required(VERSION 3.10)
if(MSVC)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /std:c++14")
endif(MSVC)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(VulkanRenderer)

SET(OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/")
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${OUTPUT_DIRECTORY})

# Executable
include_directories(include)
file(GLOB_RECURSE SOURCES src/*.cpp)
add_library(physycl STATIC ${SOURCES})
target_include_directories(physycl PUBLIC include)

