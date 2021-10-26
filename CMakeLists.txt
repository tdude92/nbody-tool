cmake_minimum_required(VERSION 3.15.0)
project(nbodytool LANGUAGES CXX C)

option(NBT_BUILD_DEMOS "Build Visual Demos" OFF)
option(NBT_BUILD_TESTS "Build Tests" OFF)
option(NBT_USE_CUDA "Use CUDA Acceleration" OFF)

if(USE_CUDA)
    enable_language(CUDA)
    include(FindCUDA/select_compute_arch)
    CUDA_DETECT_INSTALLED_GPUS(INSTALLED_GPU_CCS_1)
    string(STRIP "${INSTALLED_GPU_CCS_1}" INSTALLED_GPU_CCS_2)
    string(REPLACE " " ";" INSTALLED_GPU_CCS_3 "${INSTALLED_GPU_CCS_2}")
    string(REPLACE "." "" CUDA_ARCH_LIST "${INSTALLED_GPU_CCS_3}")
    SET(CMAKE_CUDA_ARCHITECTURES ${CUDA_ARCH_LIST})
endif()

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# GLFW flags
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)

# Compiler flags
if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    # using GCC
    add_compile_options(-O3)
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    # using Visual Studio C++
    # TODO: Remove RTC1 flag and add Ox
    string(REGEX REPLACE "/RTC(su|[1su])" "" CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}")
    string(REGEX REPLACE "/RTC(su|[1su])" "" CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
    string(REGEX REPLACE "/RTC(su|[1su])" "" CMAKE_C_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}")
    string(REGEX REPLACE "/RTC(su|[1su])" "" CMAKE_C_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
    add_compile_options(/Ox)
endif()

include_directories(
    include/
    deps/
    deps/googletest-1.11.0/googletest/include/
    deps/eigen-3.4.0/Eigen/
    deps/glfw-3.3.4/include/
)

add_subdirectory(src/)

if(NBT_BUILD_TESTS)
    add_subdirectory(deps/googletest-1.11.0/)
    add_subdirectory(test/)
endif()

if(NBT_BUILD_DEMOS)
    add_subdirectory(deps/glfw-3.3.4/)
    add_subdirectory(demo/)
endif()
