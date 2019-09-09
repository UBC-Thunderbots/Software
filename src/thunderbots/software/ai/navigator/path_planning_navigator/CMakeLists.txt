add_library(path_planning_navigator STATIC
    path_planning_navigator.cpp 
    trespass.cpp
    util.cpp
    )
target_include_directories(path_planning_navigator
    PUBLIC 
        ${CMAKE_CURRENT_SOURCE_DIR}
    )

add_subdirectory(obstacle)
add_subdirectory(path_planner)
