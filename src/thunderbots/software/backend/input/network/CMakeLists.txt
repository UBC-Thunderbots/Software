add_library(backend_network_input INTERFACE)
target_link_libraries(backend_network_input
    INTERFACE
        backend_network_input_filter
        backend_network_input_networking
    )

add_subdirectory(filter)
add_subdirectory(networking)
