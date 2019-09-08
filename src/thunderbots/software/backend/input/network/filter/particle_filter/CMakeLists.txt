add_library(particle_filter STATIC
    particle_filter.cpp
    )
target_include_directories(particle_filter
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
    )
