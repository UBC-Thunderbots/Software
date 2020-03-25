extern "C"
{
#include "firmware/main/app/control/trajectory_planner.h"
}

#include <gtest/gtest.h>

#include <vector>
#include <iostream>

#include "firmware/main/math/polynomial_1d.h"
#include "firmware/main/math/polynomial_2d.h"
#include "firmware/main/math/vector_2d.h"
#include "trajectory_planner.h"
#include "math.h"

class TrajectoryPlannerTest : public testing::Test
{
   protected:
    virtual void SetUp() {}

    virtual void TearDown() {}

    static std::vector<double> getVelocityFromTrajectory(TrajectoryElement_t trajectory[], int num_elements, double inital_speed)
    {
        std::vector<double> velocity;

        // Calculate the delta-position and the delta-time
        velocity.push_back(inital_speed);

        for(unsigned i = 1; i < num_elements-1; i++){
            const double dx = trajectory[i+1].position.x - trajectory[i].position.x;
            const double dy = trajectory[i+1].position.y - trajectory[i].position.y;
        
            const double dt = trajectory[i+1].time - trajectory[i].time;

            velocity.push_back( sqrt((pow(dx,2) + pow(dy,2)))/dt);
        }
        return velocity;
    }

    static std::vector<double> getAccelerationFromVelocity(std::vector<double> velocity, TrajectoryElement_t trajectory[], int num_elements)
    {
        std::vector<double> acceleration;

        // Calculate the delta-position and the delta-time
        acceleration.push_back(0.0);

        for(unsigned i = 1; i < num_elements-2; i++){
            const double dv = velocity[i+1] - velocity[i];
            const double dt = trajectory[i+1].time - trajectory[i].time;

            acceleration.push_back(dv/dt);
        }

        for( int i = 0; i < acceleration.size(); i++){
            std::cout << "ACC: " << acceleration[i] << " VEL: " << velocity[i] << std::endl;
        }
       return acceleration;
    }
};

TEST_F(TrajectoryPlannerTest, path_length_and_start_point_end_point_test)
{
    Polynomial2dOrder3_t path = {
        .x = {.coefficients = {2, 0, 1, 0}},
        .y = {.coefficients = {1, 0, 1, 0}},

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
        double length = sqrt( pow(trajectory[i].position.x - trajectory[i+1].position.x,2) + pow(trajectory[i].position.y - trajectory[i+1].position.y,2));
        segment_length_sum += length;
    }
    
    EXPECT_NEAR(segment_length_sum, arc_segment_length, 0.01);
    EXPECT_NEAR(trajectory[num_segments-1].position.x, shared_polynomial2d_getValueOrder3(path, t_end).x, 0.01 );
    EXPECT_NEAR(trajectory[num_segments-1].position.y, shared_polynomial2d_getValueOrder3(path, t_end).y, 0.01 );

    EXPECT_NEAR(trajectory[0].position.x, shared_polynomial2d_getValueOrder3(path, t_start).x, 0.01 );
    EXPECT_NEAR(trajectory[0].position.x, shared_polynomial2d_getValueOrder3(path, t_start).y, 0.01 );
}

TEST_F(TrajectoryPlannerTest, dynamics_dont_exceed_maximums_straight_line)
{
    Polynomial2dOrder3_t path = {
        .x = {.coefficients = {0, 0, 1, 0}},
        .y = {.coefficients = {0, 0, 1, 0}},

    };

    const double t_start = 0;
    const double t_end = 3;
    const unsigned int num_segments = 500;
    const double max_acc = 3;
    const double max_speed = 9;
    const double initial_speed = 2;
    const double final_speed = 5;

    TrajectoryElement_t* trajectory =
        generate_constant_arc_length_segmentation(path, t_start, t_end, num_segments, max_acc, max_speed, initial_speed, final_speed);
    
    // Create the parmeterization to contain the desired number of segments
    CREATE_STATIC_ARC_LENGTH_PARAMETRIZATION(arc_length_param, __TRAJECTORY_PLANNER_MAX_NUM_SEGMENTS__);

    // Get all of the points for the arc length parameterization (Not constant arc length
    // segments)
    shared_polynomial_getArcLengthParametrizationOrder3(path, t_start, t_end,
                                                        arc_length_param);

    double velocities[num_segments];
    double accelerations[num_segments];

    std::vector<double> velocity = getVelocityFromTrajectory(trajectory, num_segments, initial_speed);
    std::vector<double> acceleration = getAccelerationFromVelocity(velocity, trajectory, num_segments);

    for( double vel : velocity) {
        EXPECT_TRUE( vel <= max_speed*1.1 && vel >= initial_speed);
    }

    EXPECT_NEAR( velocity.back(), final_speed, 0.1);
    EXPECT_DOUBLE_EQ( velocity.front(), initial_speed);

    for( double acc : acceleration){
        EXPECT_TRUE( fabs(acc) <= max_acc*1.2 );
    }

    EXPECT_NEAR(trajectory[num_segments-1].position.x, shared_polynomial2d_getValueOrder3(path, t_end).x, 0.01 );
    EXPECT_NEAR(trajectory[num_segments-1].position.y, shared_polynomial2d_getValueOrder3(path, t_end).y, 0.01 );

    EXPECT_NEAR(trajectory[0].position.x, shared_polynomial2d_getValueOrder3(path, t_start).x, 0.01 );
    EXPECT_NEAR(trajectory[0].position.x, shared_polynomial2d_getValueOrder3(path, t_start).y, 0.01 );
}

TEST_F(TrajectoryPlannerTest, path_length_and_start_point_end_point_test_reverse_direction)
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
    shared_polynomial_getArcLengthParametrizationOrder3(path, t_end, t_start,
                                                        arc_length_param);

    const double arc_segment_length =
        arc_length_param.arc_length_values[arc_length_param.num_values-1];

    double segment_length_sum = 0;

    // Test by checking the length between points
    // and the length of the total path
    for (uint i = 0; i < num_segments-1; i++)
    {
        double length = sqrt( pow(trajectory[i+1].position.x - trajectory[i].position.x, 2) + pow(trajectory[i+1].position.y - trajectory[i].position.y,2));
        segment_length_sum += length;
    }
    EXPECT_NEAR(segment_length_sum, arc_segment_length, 0.01);
    EXPECT_NEAR(trajectory[num_segments-1].position.x, shared_polynomial2d_getValueOrder3(path, t_end).x, 0.01 );
    EXPECT_NEAR(trajectory[num_segments-1].position.y, shared_polynomial2d_getValueOrder3(path, t_end).y, 0.01 );

    EXPECT_NEAR(trajectory[0].position.x, shared_polynomial2d_getValueOrder3(path, t_start).x, 0.01 );
    EXPECT_NEAR(trajectory[0].position.y, shared_polynomial2d_getValueOrder3(path, t_start).y, 0.01 );
}

TEST_F(TrajectoryPlannerTest, dynamics_dont_exceed_maximums_curved_path)
{
    Polynomial2dOrder3_t path = {
        .x = {.coefficients = {0, 2, 1, 0}},
        .y = {.coefficients = {3, 0, 1, 0}},

    };

    const double t_start = 0;
    const double t_end = 1;
    const unsigned int num_segments = 500;
    const double max_acc = 3;
    const double max_speed = 9;
    const double initial_speed = 1;
    const double final_speed = 5;

    TrajectoryElement_t* trajectory =
        generate_constant_arc_length_segmentation(path, t_start, t_end, num_segments, max_acc, max_speed, initial_speed, final_speed);
    
    // Create the parmeterization to contain the desired number of segments
    CREATE_STATIC_ARC_LENGTH_PARAMETRIZATION(arc_length_param, __TRAJECTORY_PLANNER_MAX_NUM_SEGMENTS__);

    // Get all of the points for the arc length parameterization (Not constant arc length
    // segments)
    shared_polynomial_getArcLengthParametrizationOrder3(path, t_start, t_end,
                                                        arc_length_param);

    double velocities[num_segments];
    double accelerations[num_segments];

    std::vector<double> velocity = getVelocityFromTrajectory(trajectory, num_segments, initial_speed);
    std::vector<double> acceleration = getAccelerationFromVelocity(velocity, trajectory, num_segments);

    for( double vel : velocity) {
        EXPECT_TRUE( vel <= max_speed*1.2 && vel >= initial_speed);
    }

    EXPECT_NEAR( velocity.back(), final_speed, 0.1);
    
    EXPECT_DOUBLE_EQ( velocity.front(), initial_speed);

    for( double acc : acceleration){
        EXPECT_TRUE( fabs(acc) <= max_acc*1.2 );
    }

    EXPECT_NEAR(trajectory[num_segments-1].position.x, shared_polynomial2d_getValueOrder3(path, t_end).x, 0.025 );
    EXPECT_NEAR(trajectory[num_segments-1].position.y, shared_polynomial2d_getValueOrder3(path, t_end).y, 0.025 );

    EXPECT_NEAR(trajectory[0].position.x, shared_polynomial2d_getValueOrder3(path, t_start).x, 0.025 );
    EXPECT_NEAR(trajectory[0].position.x, shared_polynomial2d_getValueOrder3(path, t_start).y, 0.025 );
}