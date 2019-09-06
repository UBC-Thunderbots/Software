add_library(canvas_messenger STATIC
    canvas_messenger.cpp
    )
target_include_directories(canvas_messenger
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
    )
target_link_libraries(canvas_messenger
    ${catkin_LIBRARIES}
    ${G3LOG}
    )
add_dependencies(canvas_messenger
    ${catkin_EXPORTED_TARGETS}
    )

