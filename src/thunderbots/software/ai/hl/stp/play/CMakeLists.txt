add_library(play STATIC
    corner_kick_play.cpp
    defense_play.cpp
    enemy_freekick_play.cpp
    example_play.cpp
    free_kick_play.cpp
    halt_play.cpp
    kickoff_enemy_play.cpp
    kickoff_friendly_play.cpp
    penalty_kick_enemy_play.cpp
    penalty_kick_play.cpp
    play.cpp
    play_factory.cpp
    shoot_or_chip_play.cpp
    stop_play.cpp
    )
target_include_directories(play
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
    )
target_link_libraries(play
    world
    tactic
    )

