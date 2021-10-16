cmake_minimum_required(VERSION 3.15.0)
project(nbodytool LANGUAGES CXX)

option(BUILDTESTS "Build tests" OFF)
option(USECUDA "Use CUDA Acceleration" OFF)

if(USECUDA)
    enable_language(CUDA)
endif()

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
)


add_subdirectory(src/)

if(BUILDTESTS)
    add_subdirectory(deps/googletest/)
    add_subdirectory(test/)
endif()
