add_library(parameter STATIC
    dynamic_parameter_utils.cpp
    dynamic_parameters.cpp
    )
target_include_directories(parameter
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
    )
target_link_libraries(parameter
    ${catkin_LIBRARIES}
    )

find_package(rostest REQUIRED)

# The test node that verifies the dynamic_parameters are working
add_rostest_gtest(dynamic_parameters_test 
    rostest_dynamic_parameters.test
    dynamic_parameters_test.cpp
    )
target_link_libraries(dynamic_parameters_test 
    ${catkin_LIBRARIES}
    parameter
    )

# The test node that verifies the dynamic_parameters exist in the parameter server
add_rostest_gtest(check_parameter_existance_test 
    rostest_parameter_exists.test
    parameter_exists_test.cpp
    )
target_link_libraries(check_parameter_existance_test 
    ${catkin_LIBRARIES}
    parameter
    )

