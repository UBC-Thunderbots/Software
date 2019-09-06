include(${PROJECT_SOURCE_DIR}/test/test_utils.cmake)

add_library(math STATIC
    math_functions.cpp
    )
target_include_directories(math
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
    )

tbots_add_gtest(math_test
    math_functions_test.cpp
    )
target_link_libraries(math_test
    math
    )
