add_library(ai STATIC
    ai.cpp
    ai_wrapper.cpp
    )
target_include_directories(ai
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
    )
target_link_libraries(ai
    multithreading
    primitive
    hl
    navigator
    )
