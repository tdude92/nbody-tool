set(BINARY ${CMAKE_PROJECT_NAME})

if(NBT_USE_CUDA)
    set(
        SOURCES
        cuda/simulator.cpp
    )
else()
    set(
        SOURCES
        cpu/force_computer.cpp
        cpu/integrator.cpp
        cpu/octree.cpp
        cpu/simulator.cpp
    )
endif()

add_library(${BINARY} STATIC ${SOURCES})

if (UNIX)
    target_link_libraries(${BINARY} PUBLIC pthread)
endif()
