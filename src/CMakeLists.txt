set(BINARY ${CMAKE_PROJECT_NAME})

if(USE_CUDA)
    set(
        SOURCES
        cuda/simulator2d.cpp
    )
else()
    set(
        SOURCES
        cpu/struct/rigidbody.cpp
        cpu/simulator2d.cpp
    )
endif()

add_library(${BINARY} STATIC ${SOURCES})