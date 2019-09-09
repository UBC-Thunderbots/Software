add_library(navigator INTERFACE)
target_include_directories(navigator
    INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}
    )

add_subdirectory(path_planning_navigator)
add_subdirectory(placeholder_navigator)
