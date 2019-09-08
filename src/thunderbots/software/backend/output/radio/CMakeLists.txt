add_library(backend_radio_output STATIC
    radio_output.cpp
    )
target_include_directories(backend_radio_output
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
    )
target_link_libraries(backend_radio_output
    backend_radio_output_mrf
    )

add_subdirectory(mrf)
