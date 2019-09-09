add_library(stp_evaluation STATIC
    ball.cpp
    calc_best_shot.cpp
    deflect_off_enemy_target.cpp
    detect_threat.cpp
    enemy_threat.cpp
    find_open_areas.cpp
    indirect_chip.cpp
    intercept.cpp
    possession.cpp
    robot.cpp
    team.cpp
    )
target_include_directories(stp_evaluation
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
    )
target_link_libraries(stp_evaluation
    world
    )

