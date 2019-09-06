include(${PROJECT_SOURCE_DIR}/test/test_utils.cmake)

add_library(geom STATIC
    polygon.cpp
    rectangle.cpp
    util.cpp
    )
target_include_directories(geom
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
    )
target_link_libraries(geom
    ${G3LOG}
    )

tbots_add_test(geom_test
    angle_test.cpp
    point_test.cpp
    polygon_test.cpp
    util_test.cpp
    )
target_link_libraries(geom_test
    geom
    )
