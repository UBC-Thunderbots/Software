add_library(time STATIC
    duration.cpp
        time.cpp
    timestamp.cpp
    )
target_include_directories(time
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
    )
target_link_libraries(time
    ${G3LOG}
    )

tbots_add_gtest(time_test
    duration_test.cpp
    timestamp_test.cpp
    )
target_link_libraries(time_test
    time
    )
