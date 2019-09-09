add_library(hl INTERFACE)
target_include_directories(hl
    INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}
    )
target_link_libraries(hl
    world
    stp
    )

add_subdirectory(stp)
