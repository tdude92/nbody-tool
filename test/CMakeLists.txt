set(BINARY ${CMAKE_PROJECT_NAME}_test)
set(
    SOURCES
    main.cpp
    simulator.cpp
)

add_executable(${BINARY} ${SOURCES})
add_test(NAME ${BINARY} COMMAND ${BINARY})
target_link_libraries(${BINARY} PUBLIC ${CMAKE_PROJECT_NAME} gtest)

add_executable(benchmark benchmark.cpp)
target_link_libraries(benchmark PUBLIC ${CMAKE_PROJECT_NAME})