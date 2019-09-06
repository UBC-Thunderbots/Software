# Creates a gtest executable
# Example:
# ```
#   tbots_add_gtest(geom_test
#       angle_test.cpp
#       point_test.cpp
#       polygon_test.cpp
#       util_test.cpp
#       )
#   target_link_libraries(geom_test
#       geom
#       )
# ```
# Note: this will provide a `main` function
function(tbots_add_gtest [test_name])
    message(ARGV0 ${ARGV0})
    message(ARGVN ${ARGN})
    add_executable(${ARGV0} ${ARGN})
    target_link_libraries(${ARGV0}
        gtest
        gtest_main
        pthread
        )
    gtest_discover_tests(${ARGV0})
endfunction(tbots_add_gtest)


