# Find Eigen
find_package (Eigen3 3.3 REQUIRED)

add_library(backend_network_input_filter STATIC
    ball_filter.cpp
    robot_filter.cpp
    robot_team_filter.cpp
    )
target_include_directories(backend_network_input_filter
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
    )
target_link_libraries(backend_network_input_filter
    particle_filter
    Eigen3::Eigen
    )

add_subdirectory(particle_filter)
