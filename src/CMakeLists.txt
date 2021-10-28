set(BINARY ${CMAKE_PROJECT_NAME})

if(USE_CUDA)
    set(
        SOURCES
        cuda/simulator.cpp
    )
else()
    set(
        SOURCES
        cpu/integrator.cpp
        cpu/force_computer.cpp
        cpu/simulator.cpp
    )
endif()

add_library(${BINARY} STATIC ${SOURCES})