add_library(constants INTERFACE)
target_include_directories(constants
    INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}
    )

add_subdirectory(canvas_messenger)
add_subdirectory(logger)
add_subdirectory(math)
add_subdirectory(optimization)
add_subdirectory(parameter)
add_subdirectory(time)

