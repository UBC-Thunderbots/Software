add_library(backend_network_input_networking STATIC
    network_client.cpp
    network_filter.cpp
    ssl_gamecontroller_client.cpp
    ssl_vision_client.cpp
    )
target_include_directories(backend_network_input_networking 
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
    )
target_link_libraries(backend_network_input_networking
    backend_network_input_filter
    )
