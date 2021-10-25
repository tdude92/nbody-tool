# Executables
add_executable(nbt_demo_2d nbt_demo_2d.cpp)

# Libs
target_link_libraries(nbt_demo_2d PUBLIC ${CMAKE_PROJECT_NAME} sfml-graphics sfml-window sfml-system)