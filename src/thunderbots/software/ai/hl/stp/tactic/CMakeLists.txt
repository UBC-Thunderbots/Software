add_library(tactic STATIC
    block_shot_path_tactic.cpp
    cherry_pick_tactic.cpp
    chip_tactic.cpp
    crease_defender_tactic.cpp
    free_kick_tactic.cpp
    goalie_tactic.cpp
    move_tactic.cpp
    passer_tactic.cpp
    patrol_tactic.cpp
    penalty_kick_tactic.cpp
    receiver_tactic.cpp
    shadow_enemy_tactic.cpp
        shadow_freekicker_tactic.cpp
    shoot_goal_tactic.cpp
    stop_tactic.cpp
    tactic.cpp
    )
target_include_directories(tactic
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
    )
target_link_libraries(tactic
    world
    intent
    )

