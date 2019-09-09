add_library(path_planning_navigator_obstacle STATIC
    obstacle.cpp 
    robot_obstacle.cpp 
    )
target_include_directories(path_planning_navigator_obstacle 
    PUBLIC 
        ${CMAKE_CURRENT_SOURCE_DIR}
    )
target_link_libraries(path_planning_navigator_obstacle
    world
    )
