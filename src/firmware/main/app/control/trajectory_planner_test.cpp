extern "C"
{
#include "firmware/main/app/control/trajectory_planner.h"
}

#include <gtest/gtest.h>

#include <iostream>

#include "firmware/main/math/polynomial_1d.h"
#include "firmware/main/math/polynomial_2d.h"
#include "firmware/main/math/vector_2d.h"
#include "trajectory_planner.h"
#include "math.h"

TEST(TrajectoryPlannerTest, path_length_and_start_point_end_point_test)
{
    Polynomial2dOrder3_t path = {
        .x = {.coefficients = {0, 0, 1, 0}},
        .y = {.coefficients = {0, 0, 1, 0}},

    };

    const double t_start = 0;
    const double t_end = 1;
    const unsigned int num_segments = 5999;
    const double max_acc = 3;
    const double max_speed = 3;
    const double initial_speed = 0;
    const double final_speed = 0;

    TrajectoryElement_t* trajectory =
        generate_constant_arc_length_segmentation(path, t_start, t_end, num_segments, max_acc, max_speed, initial_speed, final_speed);
    
    // Create the parmeterization to contain the desired number of segments
    CREATE_STATIC_ARC_LENGTH_PARAMETRIZATION(arc_length_param, __TRAJECTORY_PLANNER_MAX_NUM_SEGMENTS__);

    // Get all of the points for the arc length parameterization (Not constant arc length
    // segments)
    shared_polynomial_getArcLengthParametrizationOrder3(path, t_start, t_end,
                                                        arc_length_param);

    const double arc_segment_length =
        arc_length_param.arc_length_values[arc_length_param.num_values-1];

    double segment_length_sum = 0;

    // Test by checking the length between points
    // and the length of the total path
    for (uint i = 0; i < num_segments-1; i++)
    {
        double length = sqrt( pow(trajectory[i].position.x - trajectory[i+1].position.x,2) + pow(trajectory[i].position.x - trajectory[i+1].position.x,2));
        segment_length_sum += length;
    }
    
    EXPECT_NEAR(segment_length_sum, arc_segment_length, 0.01);
    EXPECT_NEAR(trajectory[num_segments-1].position.x, shared_polynomial2d_getValueOrder3(path, t_end).x, 0.01 );
    EXPECT_NEAR(trajectory[num_segments-1].position.x, shared_polynomial2d_getValueOrder3(path, t_end).y, 0.01 );

    EXPECT_NEAR(trajectory[0].position.x, shared_polynomial2d_getValueOrder3(path, t_start).x, 0.01 );
    EXPECT_NEAR(trajectory[0].position.x, shared_polynomial2d_getValueOrder3(path, t_start).y, 0.01 );
}

TEST(TrajectoryPlannerTest, path_length_and_start_point_end_point_test_reverse_direction)
{
    Polynomial2dOrder3_t path = {
        .x = {.coefficients = {0, 0, 1, 0}},
        .y = {.coefficients = {0, 0, 1, 0}},

    };

    const double t_start = 1;
    const double t_end = 0;
    const unsigned int num_segments = 5999;
    const double max_acc = 3;
    const double max_speed = 3;
    const double initial_speed = 0;
    const double final_speed = 0;

    TrajectoryElement_t* trajectory =
        generate_constant_arc_length_segmentation(path, t_start, t_end, num_segments, max_acc, max_speed, initial_speed, final_speed);
    
    // Create the parmeterization to contain the desired number of segments
    CREATE_STATIC_ARC_LENGTH_PARAMETRIZATION(arc_length_param, __TRAJECTORY_PLANNER_MAX_NUM_SEGMENTS__);

    // Get all of the points for the arc length parameterization (Not constant arc length
    // segments)
    shared_polynomial_getArcLengthParametrizationOrder3(path, t_start, t_end,
                                                        arc_length_param);

    const double arc_segment_length =
        arc_length_param.arc_length_values[arc_length_param.num_values-1];

    double segment_length_sum = 0;

    // Test by checking the length between points
    // and the length of the total path
    for (uint i = 0; i < num_segments-1; i++)
    {
        double length = sqrt( pow(trajectory[i].position.x - trajectory[i+1].position.x,2) + pow(trajectory[i].position.x - trajectory[i+1].position.x,2));
        segment_length_sum += length;
    }
    
    EXPECT_NEAR(segment_length_sum, arc_segment_length, 0.01);
    EXPECT_NEAR(trajectory[num_segments-1].position.x, shared_polynomial2d_getValueOrder3(path, t_start).x, 0.01 );
    EXPECT_NEAR(trajectory[num_segments-1].position.x, shared_polynomial2d_getValueOrder3(path, t_start).y, 0.01 );

    EXPECT_NEAR(trajectory[0].position.x, shared_polynomial2d_getValueOrder3(path, t_end).x, 0.01 );
    EXPECT_NEAR(trajectory[0].position.x, shared_polynomial2d_getValueOrder3(path, t_end).y, 0.01 );
}