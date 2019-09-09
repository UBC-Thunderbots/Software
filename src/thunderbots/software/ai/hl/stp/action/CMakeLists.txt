add_library(action STATIC
    action.cpp
    chip_action.cpp
    dribble_action.cpp
    kick_action.cpp
    move_action.cpp
    movespin_action.cpp
    pivot_action.cpp
    stop_action.cpp
    )
target_include_directories(action
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
    )
