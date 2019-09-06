include(${PROJECT_SOURCE_DIR}/test/test_utils.cmake)

add_library(optimization INTERFACE)
target_include_directories(optimization 
    INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}
    )

tbots_add_gtest(optimization_test
    gradient_descent_test.cpp
    )
target_link_libraries(optimization_test
    optimization
    )


