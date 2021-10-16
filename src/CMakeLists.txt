set(BINARY ${CMAKE_PROJECT_NAME})
set(
    SOURCES
    Integrator.cpp
    Octree.cpp
    Simulator.cpp
)

add_library(${BINARY} STATIC ${SOURCES})