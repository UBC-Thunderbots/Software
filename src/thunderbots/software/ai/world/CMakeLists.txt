add_library(world STATIC
    ball.cpp
    field.cpp
    game_state.cpp
    refbox_constants.cpp
    robot.cpp
    team.cpp
    world.cpp
    )
target_include_directories(world
    STATIC
        ${CMAKE_CURRENT_SOURCE_DIR}
    )
