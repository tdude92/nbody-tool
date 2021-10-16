cmake_minimum_required(VERSION 3.15.0)
project(nbodytool)

option(BUILDTESTS "Build tests" OFF)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


# g++ flags
set(GCC_COMPILE_FLAGS "-O3")
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_COMPILE_FLAGS}")

include_directories(
    include/
    deps/
)

add_subdirectory(src/)

if(BUILDTESTS)
    add_subdirectory(deps/googletest/)
    add_subdirectory(test/)
endif()
