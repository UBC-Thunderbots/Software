add_library(backend_grsim_output STATIC
        command_primitive_visitor/grsim_command_primitive_visitor.cpp
    grsim_output.cpp
        command_primitive_visitor/motion_controller.cpp
    )
target_include_directories(backend_grsim_output
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
    )
target_link_libraries(backend_grsim_output
    tbots_proto
    )
