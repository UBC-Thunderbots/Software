include(${PROJECT_SOURCE_DIR}/test/test_utils.cmake)

add_library(multithreading INTERFACE)
target_include_directories(multithreading
    INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}
    )
target_link_libraries(multithreading 
    INTERFACE
        ${Boost_LIBRARIES}
        time
    )

tbots_add_gtest(multithreading_test
    observer_test.cpp
    subject_test.cpp
    thread_safe_buffer_test.cpp
    threaded_observer_test.cpp
    )
target_link_libraries(multithreading_test
    multithreading
    )
