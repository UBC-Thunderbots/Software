add_library(evaluation STATIC
    pass.cpp
    )
target_include_directories(evaluation
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
    )
target_link_libraries(evaluation
    world
    )
