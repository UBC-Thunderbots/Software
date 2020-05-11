extern "C"
{
#include "firmware/app/control/trajectory_planner.h"
}

#include <gtest/gtest.h>

#include <iostream>
#include <vector>

#include "firmware/shared/math/polynomial_1d.h"
#include "firmware/shared/math/polynomial_2d.h"
#include "firmware/shared/math/vector_2d.h"
#include "math.h"
#include "trajectory_planner.h"

class TrajectoryPlannerTest : public testing::Test
{
   protected:
    virtual void SetUp() {}

    virtual void TearDown() {}

    static std::vector<float> getSpeedsFromTrajectory(Trajectory_t* trajectory,
                                                      float inital_speed)
    {
        std::vector<float> velocity;

        // Calculate the delta-position and the delta-time
        velocity.push_back(inital_speed);

        for (unsigned i = 1; i < trajectory->num_elements - 1; i++)
        {
            const float dx = trajectory->trajectory_elements[i + 1].position.x -
                             trajectory->trajectory_elements[i].position.x;
            const float dy = trajectory->trajectory_elements[i + 1].position.y -
                             trajectory->trajectory_elements[i].position.y;

            const float dt = trajectory->trajectory_elements[i + 1].time -
                             trajectory->trajectory_elements[i].time;

            velocity.push_back(sqrt((pow(dx, 2) + pow(dy, 2))) / dt);
        }
        return velocity;
    }

    static std::vector<float> getAccelerationsFromSpeed(std::vector<float> speeds,
                                                        Trajectory_t* trajectory)
    {
        std::vector<float> acceleration;

        // Calculate the delta-position and the delta-time
        acceleration.push_back(0.0);

        for (unsigned i = 1; i < trajectory->num_elements - 2; i++)
        {
            const float dv = speeds[i + 1] - speeds[i];
            const float dt = trajectory->trajectory_elements[i + 1].time -
                             trajectory->trajectory_elements[i].time;

            acceleration.push_back(dv / dt);
        }
        return acceleration;
    }
};

TEST_F(TrajectoryPlannerTest, check_trajectory_length_and_end_points_match_path)
{
    Polynomial2dOrder3_t path = {
        .x = {.coefficients = {2, 0, 1, 0}},
        .y = {.coefficients = {1, 0, 1, 0}},

    };

    FirmwareRobotPathParameters_t path_parameters;
    path_parameters.path                       = path;
    path_parameters.t_start                    = 0;
    path_parameters.t_end                      = 1;
    path_parameters.num_segments               = 5999;
    path_parameters.max_allowable_acceleration = 3;
    path_parameters.max_allowable_speed        = 3;
    path_parameters.initial_speed              = 0;
    path_parameters.final_speed                = 0;

    TrajectoryElement_t const_arc_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    Trajectory_t trajectory = {.trajectory_elements = const_arc_elements};

    app_trajectory_planner_generate_constant_arc_length_segmentation(path_parameters,
                                                                     &trajectory);

    // Create the parmeterization to contain the desired number of segments
    CREATE_STATIC_ARC_LENGTH_PARAMETRIZATION(arc_length_param,
                                             TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS);
    // Get all of the points for the arc length parameterization (Not constant arc length
    // segments)
    shared_polynomial_getArcLengthParametrizationOrder3(
        path_parameters.path, path_parameters.t_start, path_parameters.t_end,
        arc_length_param);

    const float arc_segment_length =
        arc_length_param.arc_length_values[arc_length_param.num_values - 1];

    float segment_length_sum = 0;

    // Test by checking the length between points
    // and the length of the total path
    for (uint i = 0; i < trajectory.num_elements - 1; i++)
    {
        float length = static_cast<float>(
            sqrt(pow(trajectory.trajectory_elements[i].position.x -
                         trajectory.trajectory_elements[i + 1].position.x,
                     2) +
                 pow(trajectory.trajectory_elements[i].position.y -
                         trajectory.trajectory_elements[i + 1].position.y,
                     2)));
        segment_length_sum += length;
    }
    EXPECT_NEAR(segment_length_sum, arc_segment_length, 0.01);
    EXPECT_NEAR(
        trajectory.trajectory_elements[path_parameters.num_segments - 1].position.x,
        shared_polynomial2d_getValueOrder3(path_parameters.path, path_parameters.t_end).x,
        0.01);
    EXPECT_NEAR(
        trajectory.trajectory_elements[path_parameters.num_segments - 1].position.y,
        shared_polynomial2d_getValueOrder3(path_parameters.path, path_parameters.t_end).y,
        0.01);

    EXPECT_NEAR(
        trajectory.trajectory_elements[0].position.x,
        shared_polynomial2d_getValueOrder3(path_parameters.path, path_parameters.t_start)
            .x,
        0.01);
    EXPECT_NEAR(
        trajectory.trajectory_elements[0].position.x,
        shared_polynomial2d_getValueOrder3(path_parameters.path, path_parameters.t_start)
            .y,
        0.01);
}

TEST_F(TrajectoryPlannerTest, dynamics_dont_exceed_maximums_straight_line)
{
    Polynomial2dOrder3_t path = {
        .x = {.coefficients = {0, 0, 1, 0}},
        .y = {.coefficients = {0, 0, 1, 0}},

    };

    FirmwareRobotPathParameters_t path_parameters;
    path_parameters.path                       = path;
    path_parameters.t_start                    = 0;
    path_parameters.t_end                      = 3;
    path_parameters.num_segments               = 500;
    path_parameters.max_allowable_acceleration = 3;
    path_parameters.max_allowable_speed        = 9;
    path_parameters.initial_speed              = 2;
    path_parameters.final_speed                = 5;

    TrajectoryElement_t const_arc_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    Trajectory_t trajectory = {.trajectory_elements = const_arc_elements};


    app_trajectory_planner_generate_constant_arc_length_segmentation(path_parameters,
                                                                     &trajectory);

    // Create the parmeterization to contain the desired number of segments
    CREATE_STATIC_ARC_LENGTH_PARAMETRIZATION(arc_length_param,
                                             TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS);

    // Get all of the points for the arc length parameterization (Not constant arc length
    // segments)
    shared_polynomial_getArcLengthParametrizationOrder3(
        path, path_parameters.t_start, path_parameters.t_end, arc_length_param);

    float velocities[path_parameters.num_segments];
    std::vector<float> velocity =
        getSpeedsFromTrajectory(&trajectory, path_parameters.initial_speed);
    std::vector<float> acceleration = getAccelerationsFromSpeed(velocity, &trajectory);

    for (float vel : velocity)
    {
        EXPECT_TRUE(vel <= path_parameters.max_allowable_speed * 1.1 &&
                    vel >= path_parameters.initial_speed);
    }

    EXPECT_NEAR(velocity.back(), path_parameters.final_speed, 0.1);
    EXPECT_FLOAT_EQ(velocity.front(), path_parameters.initial_speed);

    for (float acc : acceleration)
    {
        EXPECT_TRUE(fabs(acc) <= path_parameters.max_allowable_acceleration * 1.2);
    }

    EXPECT_NEAR(
        trajectory.trajectory_elements[path_parameters.num_segments - 1].position.x,
        shared_polynomial2d_getValueOrder3(path_parameters.path, path_parameters.t_end).x,
        0.01);
    EXPECT_NEAR(
        trajectory.trajectory_elements[path_parameters.num_segments - 1].position.y,
        shared_polynomial2d_getValueOrder3(path_parameters.path, path_parameters.t_end).y,
        0.01);

    EXPECT_NEAR(trajectory.trajectory_elements[0].position.x,
                shared_polynomial2d_getValueOrder3(path, path_parameters.t_start).x,
                0.01);
    EXPECT_NEAR(trajectory.trajectory_elements[0].position.y,
                shared_polynomial2d_getValueOrder3(path, path_parameters.t_start).y,
                0.01);
}

TEST_F(TrajectoryPlannerTest,
       check_trajectory_length_and_end_points_match_path_reverse_parameterization)
{
    Polynomial2dOrder3_t path = {
        .x = {.coefficients = {0, 0, 1, 0}},
        .y = {.coefficients = {0, 0, 1, 0}},

    };

    FirmwareRobotPathParameters_t path_parameters;
    path_parameters.path                       = path;
    path_parameters.t_start                    = 1;
    path_parameters.t_end                      = 0;
    path_parameters.num_segments               = 5999;
    path_parameters.max_allowable_acceleration = 3;
    path_parameters.max_allowable_speed        = 3;
    path_parameters.initial_speed              = 0;
    path_parameters.final_speed                = 0;

    TrajectoryElement_t const_arc_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    Trajectory_t trajectory = {.trajectory_elements = const_arc_elements};


    app_trajectory_planner_generate_constant_arc_length_segmentation(path_parameters,
                                                                     &trajectory);

    // Create the parmeterization to contain the desired number of segments
    CREATE_STATIC_ARC_LENGTH_PARAMETRIZATION(arc_length_param,
                                             TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS);

    // Get all of the points for the arc length parameterization (Not constant arc length
    // segments)
    shared_polynomial_getArcLengthParametrizationOrder3(
        path, path_parameters.t_end, path_parameters.t_start, arc_length_param);

    const float arc_segment_length =
        arc_length_param.arc_length_values[arc_length_param.num_values - 1];

    float segment_length_sum = 0;

    // Test by checking the length between points
    // and the length of the total path
    for (uint i = 0; i < trajectory.num_elements - 1; i++)
    {
        float length =
            static_cast<float>(sqrt(pow(trajectory.trajectory_elements[i + 1].position.x -
                                            trajectory.trajectory_elements[i].position.x,
                                        2) +
                                    pow(trajectory.trajectory_elements[i + 1].position.y -
                                            trajectory.trajectory_elements[i].position.y,
                                        2)));
        segment_length_sum += length;
    }
    EXPECT_NEAR(segment_length_sum, arc_segment_length, 0.01);
    EXPECT_NEAR(trajectory.trajectory_elements[trajectory.num_elements - 1].position.x,
                shared_polynomial2d_getValueOrder3(path, path_parameters.t_end).x, 0.01);
    EXPECT_NEAR(trajectory.trajectory_elements[trajectory.num_elements - 1].position.y,
                shared_polynomial2d_getValueOrder3(path, path_parameters.t_end).y, 0.01);

    EXPECT_NEAR(trajectory.trajectory_elements[0].position.x,
                shared_polynomial2d_getValueOrder3(path, path_parameters.t_start).x,
                0.01);
    EXPECT_NEAR(trajectory.trajectory_elements[0].position.y,
                shared_polynomial2d_getValueOrder3(path, path_parameters.t_start).y,
                0.01);
}

TEST_F(TrajectoryPlannerTest, dynamics_dont_exceed_maximums_curved_path)
{
    Polynomial2dOrder3_t path = {
        .x = {.coefficients = {0, 2, 1, 0}},
        .y = {.coefficients = {3, 0, 1, 0}},

    };
    FirmwareRobotPathParameters_t path_parameters;
    path_parameters.path                       = path;
    path_parameters.t_start                    = 0;
    path_parameters.t_end                      = 1;
    path_parameters.num_segments               = 500;
    path_parameters.max_allowable_acceleration = 3;
    path_parameters.max_allowable_speed        = 9;
    path_parameters.initial_speed              = 1;
    path_parameters.final_speed                = 5;

    TrajectoryElement_t const_arc_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    Trajectory_t trajectory = {.trajectory_elements = const_arc_elements};

    app_trajectory_planner_generate_constant_arc_length_segmentation(path_parameters,
                                                                     &trajectory);

    // Create the parmeterization to contain the desired number of segments
    CREATE_STATIC_ARC_LENGTH_PARAMETRIZATION(arc_length_param,
                                             TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS);

    // Get all of the points for the arc length parameterization (Not constant arc length
    // segments)
    shared_polynomial_getArcLengthParametrizationOrder3(
        path, path_parameters.t_start, path_parameters.t_end, arc_length_param);

    float velocities[trajectory.num_elements];

    std::vector<float> velocity =
        getSpeedsFromTrajectory(&trajectory, path_parameters.initial_speed);
    std::vector<float> acceleration = getAccelerationsFromSpeed(velocity, &trajectory);

    for (float vel : velocity)
    {
        EXPECT_TRUE(vel <= path_parameters.max_allowable_speed * 1.2 &&
                    vel >= path_parameters.initial_speed);
    }

    EXPECT_NEAR(velocity.back(), path_parameters.final_speed, 0.1);

    EXPECT_FLOAT_EQ(velocity.front(), path_parameters.initial_speed);

    for (float acc : acceleration)
    {
        EXPECT_TRUE(fabs(acc) <= path_parameters.max_allowable_acceleration * 1.2);
    }

    EXPECT_NEAR(trajectory.trajectory_elements[trajectory.num_elements - 1].position.x,
                shared_polynomial2d_getValueOrder3(path, path_parameters.t_end).x, 0.025);
    EXPECT_NEAR(trajectory.trajectory_elements[trajectory.num_elements - 1].position.y,
                shared_polynomial2d_getValueOrder3(path, path_parameters.t_end).y, 0.025);

    EXPECT_NEAR(trajectory.trajectory_elements[0].position.x,
                shared_polynomial2d_getValueOrder3(path, path_parameters.t_start).x,
                0.025);
    EXPECT_NEAR(trajectory.trajectory_elements[0].position.x,
                shared_polynomial2d_getValueOrder3(path, path_parameters.t_start).y,
                0.025);
}

TEST_F(TrajectoryPlannerTest, test_get_constant_time_interpolation_straight_line)
{
    Polynomial2dOrder3_t path = {
        .x = {.coefficients = {0, 0, 1, 0}},
        .y = {.coefficients = {0, 0, 1, 0}},

    };

    FirmwareRobotPathParameters_t path_parameters;
    path_parameters.path                       = path;
    path_parameters.t_start                    = 0;
    path_parameters.t_end                      = 1;
    path_parameters.num_segments               = 5999;
    path_parameters.max_allowable_acceleration = 3;
    path_parameters.max_allowable_speed        = 3;
    path_parameters.initial_speed              = 0;
    path_parameters.final_speed                = 0;

    TrajectoryElement_t const_arc_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    Trajectory_t trajectory = {.trajectory_elements = const_arc_elements};

    app_trajectory_planner_generate_constant_arc_length_segmentation(path_parameters,
                                                                     &trajectory);

    // Create the parameterization to contain the desired number of segments
    CREATE_STATIC_ARC_LENGTH_PARAMETRIZATION(arc_length_param,
                                             TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS);

    // Get all of the points for the arc length parameterization (Not constant arc length
    // segments)
    shared_polynomial_getArcLengthParametrizationOrder3(
        path, path_parameters.t_start, path_parameters.t_end, arc_length_param);

    float segment_length_sum = 0;

    Trajectory_t const_interp_trajectory;
    TrajectoryElement_t const_interp_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    const_interp_trajectory.trajectory_elements = const_interp_elements;

    // Calculate the constant-interpolation period equivalent of the trajectory
    app_trajectory_planner_interpolate_constant_time_trajectory_segmentation(
        &const_interp_trajectory, &trajectory, 0.001);

    EXPECT_NEAR(trajectory.trajectory_elements[trajectory.num_elements - 1].position.x,
                const_interp_trajectory
                    .trajectory_elements[const_interp_trajectory.num_elements - 1]
                    .position.x,
                0.001);
    EXPECT_NEAR(trajectory.trajectory_elements[trajectory.num_elements - 1].position.y,
                const_interp_trajectory
                    .trajectory_elements[const_interp_trajectory.num_elements - 1]
                    .position.y,
                0.001);
    EXPECT_NEAR(trajectory.trajectory_elements[trajectory.num_elements - 1].time,
                const_interp_trajectory
                    .trajectory_elements[const_interp_trajectory.num_elements - 1]
                    .time,
                0.001);

    EXPECT_NEAR(trajectory.trajectory_elements[0].position.x,
                const_interp_trajectory.trajectory_elements[0].position.x, 0.001);
    EXPECT_NEAR(trajectory.trajectory_elements[0].position.y,
                const_interp_trajectory.trajectory_elements[0].position.x, 0.001);
    EXPECT_NEAR(trajectory.trajectory_elements[0].time,
                const_interp_trajectory.trajectory_elements[0].time, 0.001);
}

TEST_F(TrajectoryPlannerTest, test_get_constant_time_interpolation_curved_line)
{
    Polynomial2dOrder3_t path = {
        .x = {.coefficients = {2, 0, 1, 0}},
        .y = {.coefficients = {1, 0, 1, 0}},

    };

    FirmwareRobotPathParameters_t path_parameters;
    path_parameters.path                       = path;
    path_parameters.t_start                    = 0;
    path_parameters.t_end                      = 1;
    path_parameters.num_segments               = 5999;
    path_parameters.max_allowable_acceleration = 3;
    path_parameters.max_allowable_speed        = 3;
    path_parameters.initial_speed              = 0;
    path_parameters.final_speed                = 0;

    TrajectoryElement_t const_arc_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    Trajectory_t trajectory = {.trajectory_elements = const_arc_elements};


    app_trajectory_planner_generate_constant_arc_length_segmentation(path_parameters,
                                                                     &trajectory);

    // Create the parameterization to contain the desired number of segments
    CREATE_STATIC_ARC_LENGTH_PARAMETRIZATION(arc_length_param,
                                             TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS);

    // Get all of the points for the arc length parameterization (Not constant arc length
    // segments)
    shared_polynomial_getArcLengthParametrizationOrder3(
        path, path_parameters.t_start, path_parameters.t_end, arc_length_param);

    Trajectory_t const_interp_trajectory;
    TrajectoryElement_t const_interp_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    const_interp_trajectory.trajectory_elements = const_interp_elements;

    // Calculate the constant-interpolation period equivalent of the trajectory
    app_trajectory_planner_interpolate_constant_time_trajectory_segmentation(
        &const_interp_trajectory, &trajectory, 0.001);

    EXPECT_NEAR(trajectory.trajectory_elements[trajectory.num_elements - 1].position.x,
                const_interp_trajectory
                    .trajectory_elements[const_interp_trajectory.num_elements - 1]
                    .position.x,
                0.001);
    EXPECT_NEAR(trajectory.trajectory_elements[trajectory.num_elements - 1].position.y,
                const_interp_trajectory
                    .trajectory_elements[const_interp_trajectory.num_elements - 1]
                    .position.y,
                0.001);
    EXPECT_NEAR(trajectory.trajectory_elements[trajectory.num_elements - 1].time,
                const_interp_trajectory
                    .trajectory_elements[const_interp_trajectory.num_elements - 1]
                    .time,
                0.001);

    EXPECT_NEAR(trajectory.trajectory_elements[0].position.x,
                const_interp_trajectory.trajectory_elements[0].position.x, 0.001);
    EXPECT_NEAR(trajectory.trajectory_elements[0].position.y,
                const_interp_trajectory.trajectory_elements[0].position.x, 0.001);
    EXPECT_NEAR(trajectory.trajectory_elements[0].time,
                const_interp_trajectory.trajectory_elements[0].time, 0.001);
}
