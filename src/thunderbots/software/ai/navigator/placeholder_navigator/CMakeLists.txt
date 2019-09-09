add_library(placeholder_navigator STATIC
    placeholder_navigator.cpp
    )
target_include_directories(placeholder_navigator
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
    )
target_link_libraries(placeholder_navigator
    navigator
    )
