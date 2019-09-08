add_library(backend STATIC
    backend_factory.cpp
    grsim_backend.cpp
    radio_backend.cpp
    )
target_include_directories(backend
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
    )
target_link_libraries(backend
    backend_network_input
    backend_radio_output
    backend_grsim_output
    world
    )

add_subdirectory(input)
add_subdirectory(output)
