set(DATA_FILE_LIST_2D
    solarsystem_uniform_10
    solarsystem_uniform_100
    stars_uniform_1k
    stars_uniform_10k
    stars_uniform_100
)
set(INTEGRATOR_LIST     EULER)
set(FORCE_COMPUTER_LIST GRAVITATIONAL_DIRECT)

find_package(OpenGL REQUIRED)
foreach(DATA_FILE IN LISTS DATA_FILE_LIST_2D)
    foreach(INTEGRATOR IN LISTS INTEGRATOR_LIST)
        foreach(FORCE_COMPUTER IN LISTS FORCE_COMPUTER_LIST)
            string(TOLOWER ${INTEGRATOR} INTEGRATOR_STRING)
            string(TOLOWER ${FORCE_COMPUTER} FORCE_COMPUTER_STRING)
            set(DEMO_NAME "2d_${INTEGRATOR_STRING}_${FORCE_COMPUTER_STRING}_${DATA_FILE}")
            
            add_executable(${DEMO_NAME} nbt_demo_2d.cpp)
            target_link_libraries(${DEMO_NAME} PUBLIC ${CMAKE_PROJECT_NAME} OpenGL::GL glfw)
            target_compile_definitions(${DEMO_NAME} PRIVATE DATA_FILE=${DATA_FILE} PRIVATE ${INTEGRATOR} PRIVATE ${FORCE_COMPUTER})
        endforeach()
    endforeach()
endforeach()