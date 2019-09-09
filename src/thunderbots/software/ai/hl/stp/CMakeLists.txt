add_library(stp STATIC
    stp.cp
    )
target_include_directories(stp
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
    )
target_link_libraries(stp
    world
    action
    stp_evaluation
    play
    tactic
    )

add_subdirectory(action)
add_subdirectory(evaluation)
add_subdirectory(play)
add_subdirectory(tactic)
