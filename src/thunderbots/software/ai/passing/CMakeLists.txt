add_library(passing STATIC
    evaluation.cpp
    pass.cpp
    pass_generator.cpp
    )
target_include_directories(passing
    STATIC
        ${CMAKE_CURRENT_SOURCE_DIR}
    )
