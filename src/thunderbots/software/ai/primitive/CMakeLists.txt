add_library(primitive STATIC
    catch_primitive.cpp
    chip_primitive.cpp
    direct_velocity_primitive.cpp
    direct_wheels_primitive.cpp
    dribble_primitive.cpp
    kick_primitive.cpp
    move_primitive.cpp
    movespin_primitive.cpp
    pivot_primitive.cpp
    stop_primitive.cpp
    )
target_include_directories(primitive
    STATIC
        ${CMAKE_CURRENT_SOURCE_DIR}
    )
