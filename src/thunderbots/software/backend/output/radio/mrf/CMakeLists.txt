add_library(backend_radio_output_mrf STATIC
    annunciator.cpp
    dongle.cpp
    mrf_primitive_visitor.cpp
    send_reliable_message_operation.cpp
    )
target_include_directories(backend_radio_output_mrf
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
    )
target_link_libraries(backend_radio_output_mrf
    tbots_primitive
    backend_radio_output_mrf_usb
    backend_radio_output_mrf_util
    )

add_subdirectory(usb)
add_subdirectory(util)
