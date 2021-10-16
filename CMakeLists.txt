cmake_minimum_required(VERSION 3.15.0)
project(nbodytool LANGUAGES CXX)

option(BUILD_TYPE "Build Type" Release)
option(BUILD_TESTS "Build tests" OFF)
option(USE_CUDA "Use CUDA Acceleration" OFF)

if(USECUDA)
    enable_language(CUDA)
endif()

set(CMAKE_BUILD_TYPE ${BUILD_TYPE})
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compiler flags
if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    # using GCC
    add_compile_options(-O3)
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    # using Visual Studio C++
    # Remove RTC1 flag and add O2
    string(REGEX REPLACE "/RTC(su|[1su])" "" CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}")
    string(REGEX REPLACE "/RTC(su|[1su])" "" CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
    add_compile_options(/O2)
endif()

include_directories(
    include/
    deps/
    deps/eigen-3.4.0/Eigen/
)

add_subdirectory(src/)

if(BUILD_TESTS)
    add_subdirectory(deps/googletest/)
    add_subdirectory(test/)
endif()
