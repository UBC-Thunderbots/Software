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

TEST(TrajectoryPlannerTest, generate_constant_arc_length_segments)
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


    // Test by checking the length between points
    // and the length of the total path
    
    //double segment_length_sum = 0;
    
    // Create the parmeterization to contain the desired number of segments
    CREATE_STATIC_ARC_LENGTH_PARAMETRIZATION(arc_length_param, __TRAJECTORY_PLANNER_MAX_NUM_SEGMENTS__);

    // Get all of the points for the arc length parameterization (Not constant arc length
    // segments)
    shared_polynomial_getArcLengthParametrizationOrder3(path, t_start, t_end,
                                                        arc_length_param);

    const double arc_segment_length =
        arc_length_param.arc_length_values[arc_length_param.num_values-1];

    std::cout << "ARC LENGTH ->>>: " << arc_segment_length << std::endl;


    double segment_length_sum = 0;

    for (uint i = 0; i < num_segments-1; i++)
    {
        double length = sqrt( pow(trajectory[i].position.x - trajectory[i+1].position.x,2) + pow(trajectory[i].position.x - trajectory[i+1].position.x,2));
        //if( fabs(length -)
        segment_length_sum += length;
    }
    std::cout << "Numerical segment length: " << segment_length_sum << std::endl;
}