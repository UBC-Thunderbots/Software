extern "C"
{
#include "firmware/app/control/trajectory_planner_private.h"

#include "firmware/app/control/trajectory_planner.h"
}
#include <gtest/gtest.h>
#include <math.h>

#include "firmware/shared/math/polynomial_1d.h"
#include "firmware/shared/math/polynomial_2d.h"
#include "firmware/shared/math/vector_2d.h"

TEST(TrajectoryPlannerImplTest, test_robot_state_generation_at_constant_t_intervals)
{
    Polynomial2dOrder3_t path = {
        .x = {.coefficients = {0, 0, 1, 0}},
        .y = {.coefficients = {0, 0, 1, 0}},

    };

    Polynomial1dOrder3_t orientation_profile = {.coefficients = {0, 0, 1, 0}};
    // Generate parameters for a trajectory
    FirmwareRobotPathParameters_t path_parameters{
        .path                               = path,
        .orientation_profile                = orientation_profile,
        .t_start                            = 0,
        .t_end                              = 1,
        .num_elements                       = TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS,
        .max_allowable_linear_acceleration  = 3,
        .max_allowable_linear_speed         = 3,
        .max_allowable_angular_acceleration = 9,
        .max_allowable_angular_speed        = 6,
        .initial_linear_speed               = 0,
        .final_linear_speed                 = 0,

    };

    float segment_lengths_linear[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    float segment_lengths_angular[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    float x_values[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    float y_values[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    float theta_values[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    app_trajectory_planner_impl_generate2dSegmentNodesAndLengths(
        path_parameters.t_start, path_parameters.t_end, path_parameters.path,
        path_parameters.num_elements, x_values, y_values, segment_lengths_linear);
    app_trajectory_planner_impl_generate1dSegmentNodesAndLengths(
        path_parameters.t_start, path_parameters.t_end,
        path_parameters.orientation_profile, path_parameters.num_elements, theta_values,
        segment_lengths_angular);

    const float delta_t = (path_parameters.t_end - path_parameters.t_start) /
                          static_cast<float>(path_parameters.num_elements - 1);

    // Check that all of the state variables are correct
    for (unsigned int i = 0; i < path_parameters.num_elements; i++)
    {
        const Vector2d_t expected_position =
            shared_polynomial2d_getValueOrder3(path, static_cast<float>(i) * delta_t);
        const float expected_orientation = shared_polynomial1d_getValueOrder3(
            orientation_profile, static_cast<float>(i) * delta_t);

        EXPECT_FLOAT_EQ(expected_orientation, theta_values[i]);
        EXPECT_FLOAT_EQ(expected_position.x, x_values[i]);
        EXPECT_FLOAT_EQ(expected_position.y, y_values[i]);
    }
    EXPECT_FLOAT_EQ(y_values[path_parameters.num_elements - 1], 1);
    EXPECT_FLOAT_EQ(x_values[path_parameters.num_elements - 1], 1);
    EXPECT_FLOAT_EQ(theta_values[path_parameters.num_elements - 1], 1);
}

TEST(TrajectoryPlannerImplTest,
     test_robot_segment_size_generation_at_constant_t_intervals)
{
    const float total_path_length_linear  = sqrtf(2.0f);
    const float total_path_length_angular = 1;
    Polynomial2dOrder3_t path             = {
        .x = {.coefficients = {0, 0, 1, 0}},
        .y = {.coefficients = {0, 0, 1, 0}},
    };

    Polynomial1dOrder3_t orientation_profile = {.coefficients = {0, 0, 1, 0}};
    // Generate parameters for a trajectory
    FirmwareRobotPathParameters_t path_parameters{
        .path                               = path,
        .orientation_profile                = orientation_profile,
        .t_start                            = 0,
        .t_end                              = 1,
        .num_elements                       = TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS,
        .max_allowable_linear_acceleration  = 3,
        .max_allowable_linear_speed         = 3,
        .max_allowable_angular_acceleration = 9,
        .max_allowable_angular_speed        = 6,
        .initial_linear_speed               = 0,
        .final_linear_speed                 = 0,

    };

    float segment_lengths_linear[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    float segment_lengths_angular[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    float x_values[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    float y_values[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    float theta_values[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    app_trajectory_planner_impl_generate2dSegmentNodesAndLengths(
        path_parameters.t_start, path_parameters.t_end, path_parameters.path,
        path_parameters.num_elements, x_values, y_values, segment_lengths_linear);
    app_trajectory_planner_impl_generate1dSegmentNodesAndLengths(
        path_parameters.t_start, path_parameters.t_end,
        path_parameters.orientation_profile, path_parameters.num_elements, theta_values,
        segment_lengths_angular);

    const float delta_length =
        total_path_length_linear / static_cast<float>(path_parameters.num_elements);
    const float delta_length_orientation =
        total_path_length_angular / static_cast<float>(path_parameters.num_elements);
    // Check that all of the segment lengths are correct
    for (unsigned int i = 0; i < path_parameters.num_elements - 1; i++)
    {
        EXPECT_NEAR(segment_lengths_linear[i], delta_length, 0.00001);
        EXPECT_NEAR(segment_lengths_angular[i], delta_length_orientation, 0.00001);
    }
}

TEST(TrajectoryPlannerImplTest,
     test_robot_segment_size_generation_at_constant_t_intervals_low_segment_count)
{
    const float total_path_length_linear  = sqrtf(2.0f);
    const float total_path_length_angular = 1;
    Polynomial2dOrder3_t path             = {
        .x = {.coefficients = {0, 0, 1, 0}},
        .y = {.coefficients = {0, 0, 1, 0}},

    };

    Polynomial1dOrder3_t orientation_profile = {.coefficients = {0, 0, 1, 0}};
    // Generate parameters for a trajectory
    FirmwareRobotPathParameters_t path_parameters{
        .path                               = path,
        .orientation_profile                = orientation_profile,
        .t_start                            = 0,
        .t_end                              = 1,
        .num_elements                       = 5,
        .max_allowable_linear_acceleration  = 3,
        .max_allowable_linear_speed         = 3,
        .max_allowable_angular_acceleration = 9,
        .max_allowable_angular_speed        = 6,
        .initial_linear_speed               = 0,
        .final_linear_speed                 = 0,

    };

    float segment_lengths_linear[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    float segment_lengths_angular[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    float x_values[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    float y_values[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    float theta_values[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    app_trajectory_planner_impl_generate2dSegmentNodesAndLengths(
        path_parameters.t_start, path_parameters.t_end, path_parameters.path,
        path_parameters.num_elements, x_values, y_values, segment_lengths_linear);
    app_trajectory_planner_impl_generate1dSegmentNodesAndLengths(
        path_parameters.t_start, path_parameters.t_end,
        path_parameters.orientation_profile, path_parameters.num_elements, theta_values,
        segment_lengths_angular);

    const float delta_length =
        total_path_length_linear / static_cast<float>(path_parameters.num_elements - 1);
    const float delta_length_orientation =
        total_path_length_angular / static_cast<float>(path_parameters.num_elements - 1);
    // Check that all of the segment lengths are correct
    for (unsigned int i = 0; i < path_parameters.num_elements - 1; i++)
    {
        EXPECT_NEAR(segment_lengths_linear[i], delta_length, 0.00001);
        EXPECT_NEAR(segment_lengths_angular[i], delta_length_orientation, 0.00001);
    }
}

TEST(TrajectoryPlannerImplTest,
     test_forward_continuity_path_curvature_max_speed_limited_constant_segment_length)
{
    const unsigned int num_segments = 10;

    float linear_segments[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    float max_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    max_speed_profile[0]  = 1;
    max_speed_profile[1]  = 1;
    max_speed_profile[2]  = 1;
    max_speed_profile[3]  = 1;
    max_speed_profile[4]  = 1;
    max_speed_profile[5]  = 1;
    max_speed_profile[6]  = 1;
    max_speed_profile[7]  = 1;
    max_speed_profile[8]  = 1;
    max_speed_profile[9]  = 1;
    max_speed_profile[10] = 1;

    linear_segments[0] = 1.0;
    linear_segments[1] = 1.0;
    linear_segments[2] = 1.0;
    linear_segments[3] = 1.0;
    linear_segments[4] = 1.0;
    linear_segments[5] = 1.0;
    linear_segments[6] = 1.0;
    linear_segments[7] = 1.0;
    linear_segments[8] = 1.0;

    Polynomial2dOrder3_t path                = {.x = {0, 0, 0, 0}, .y = {0, 0, 0, 0}};
    Polynomial1dOrder3_t orientation_profile = {.coefficients = {0, 0, 1, 0}};
    // Generate parameters for a trajectory
    FirmwareRobotPathParameters_t path_parameters{
        .path                               = path,
        .orientation_profile                = orientation_profile,
        .t_start                            = 0,
        .t_end                              = 1,
        .num_elements                       = num_segments,
        .max_allowable_linear_acceleration  = 3,
        .max_allowable_linear_speed         = 3,
        .max_allowable_angular_acceleration = 8,
        .max_allowable_angular_speed        = 3,
        .initial_linear_speed               = 1,
        .final_linear_speed                 = 1,

    };

    float speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    app_trajectory_planner_impl_createForwardsContinuousSpeedProfile(
        path_parameters.final_linear_speed, linear_segments, max_speed_profile,
        path_parameters.max_allowable_linear_acceleration,
        path_parameters.initial_linear_speed, path_parameters.num_elements,
        speed_profile);

    for (unsigned int i = 0; i < num_segments; i++)
    {
        EXPECT_TRUE(speed_profile[i] <= 1);
    }
}

TEST(TrajectoryPlannerImplTest, test_forwards_continuity_varying_segment_length)
{
    float max_allowable_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    const unsigned int num_segments = 5;

    for (unsigned int i = 0; i < num_segments; i++)
    {
        max_allowable_speed_profile[i] = 10;
    }

    FirmwareRobotPathParameters_t path_parameters;
    path_parameters.max_allowable_angular_acceleration = 8;
    path_parameters.max_allowable_angular_speed        = 3;
    path_parameters.max_allowable_linear_acceleration  = 1;
    path_parameters.max_allowable_linear_speed         = 10;
    path_parameters.num_elements                       = num_segments;
    path_parameters.initial_linear_speed               = 1;
    path_parameters.final_linear_speed                 = 2;


    float speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    float segment_lengths[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    segment_lengths[0] = 1.0;
    segment_lengths[1] = 2.0;
    segment_lengths[2] = 3.0;
    segment_lengths[3] = 4.0;

    TrajectoryPlannerGenerationStatus_t status =
        app_trajectory_planner_impl_createForwardsContinuousSpeedProfile(
            path_parameters.final_linear_speed, segment_lengths,
            max_allowable_speed_profile,
            path_parameters.max_allowable_linear_acceleration,
            path_parameters.initial_linear_speed, path_parameters.num_elements,
            speed_profile);
    EXPECT_EQ(status, OK);

    EXPECT_NEAR(speed_profile[0], path_parameters.initial_linear_speed, 0.0001);
    EXPECT_NEAR(speed_profile[1], sqrt(3), 0.00001);
    EXPECT_NEAR(speed_profile[2], sqrt(7), 0.00001);
    EXPECT_NEAR(speed_profile[3], sqrt(13), 0.00001);
    EXPECT_NEAR(speed_profile[4], 2, 0.00001);
}

TEST(TrajectoryPlannerImplTest, test_forwards_continuity_final_velocity_too_high)
{
    // Create a trajectory where the final velocity is much to high to reach given the
    // path length and acceleration
    float max_allowable_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    const unsigned int num_segments = 5;

    for (unsigned int i = 0; i < num_segments; i++)
    {
        max_allowable_speed_profile[i] = 10;
    }

    float segment_lengths[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    segment_lengths[0] = 1.0;
    segment_lengths[1] = 2.0;
    segment_lengths[2] = 3.0;
    segment_lengths[3] = 4.0;

    FirmwareRobotPathParameters_t path_parameters;
    path_parameters.max_allowable_angular_acceleration = 8;
    path_parameters.max_allowable_angular_speed        = 3;
    path_parameters.max_allowable_linear_acceleration  = 1;
    path_parameters.max_allowable_linear_speed         = 10;
    path_parameters.num_elements                       = num_segments;
    path_parameters.initial_linear_speed               = 0;
    path_parameters.final_linear_speed                 = 200;


    float speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    TrajectoryPlannerGenerationStatus_t status =
        app_trajectory_planner_impl_createForwardsContinuousSpeedProfile(
            path_parameters.final_linear_speed, segment_lengths,
            max_allowable_speed_profile,
            path_parameters.max_allowable_linear_acceleration,
            path_parameters.initial_linear_speed, path_parameters.num_elements,
            speed_profile);
    EXPECT_EQ(status, FINAL_VELOCITY_TOO_HIGH);
}

TEST(TrajectoryPlannerImplTest, test_forwards_continuity_varying_segment_length_angular)
{
    float max_allowable_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    const unsigned int num_segments = 5;

    for (unsigned int i = 0; i < num_segments; i++)
    {
        max_allowable_speed_profile[i] = 10;
    }

    float segment_lengths[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    segment_lengths[0] = 1.0;
    segment_lengths[1] = 2.0;
    segment_lengths[2] = 3.0;
    segment_lengths[3] = 4.0;

    FirmwareRobotPathParameters_t path_parameters;
    path_parameters.max_allowable_angular_acceleration = 1;
    path_parameters.max_allowable_angular_speed        = 10;
    path_parameters.num_elements                       = num_segments;

    float speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    TrajectoryPlannerGenerationStatus_t status =
        app_trajectory_planner_impl_createForwardsContinuousSpeedProfile(
            0.0f, segment_lengths, max_allowable_speed_profile,
            path_parameters.max_allowable_angular_acceleration, 0.0f,
            path_parameters.num_elements, speed_profile);
    EXPECT_EQ(status, OK);
    EXPECT_NEAR(speed_profile[0], 0, 0.0001);
    EXPECT_NEAR(speed_profile[1], sqrt(2), 0.00001);
    EXPECT_NEAR(speed_profile[2], sqrt(6), 0.00001);
    EXPECT_NEAR(speed_profile[3], sqrt(12), 0.00001);
    EXPECT_NEAR(speed_profile[4], 0, 0.00001);
}

TEST(TrajectoryPlannerImplTest,
     test_forwards_continuity_constant_segment_speed_limited_angular)
{
    float max_allowable_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    const unsigned int num_segments = 5;

    for (unsigned int i = 0; i < num_segments; i++)
    {
        max_allowable_speed_profile[i] = 1;
    }

    float segment_lengths_angular[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    segment_lengths_angular[0] = 1.0;
    segment_lengths_angular[1] = 1.0;
    segment_lengths_angular[2] = 1.0;
    segment_lengths_angular[3] = 1.0;

    FirmwareRobotPathParameters_t path_parameters;
    path_parameters.max_allowable_angular_acceleration = 10;
    path_parameters.num_elements                       = num_segments;

    float speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    TrajectoryPlannerGenerationStatus_t status =
        app_trajectory_planner_impl_createForwardsContinuousSpeedProfile(
            0.0f, segment_lengths_angular, max_allowable_speed_profile,
            path_parameters.max_allowable_angular_acceleration, 0.0f,
            path_parameters.num_elements, speed_profile);
    EXPECT_EQ(status, OK);

    EXPECT_NEAR(speed_profile[0], 0, 0.0001);
    EXPECT_NEAR(speed_profile[1], 1, 0.00001);
    EXPECT_NEAR(speed_profile[2], 1, 0.00001);
    EXPECT_NEAR(speed_profile[3], 1, 0.00001);
    EXPECT_NEAR(speed_profile[4], 0, 0.00001);
}

TEST(TrajectoryPlannerImplTest,
     test_backwards_continuity_constant_segment_length_continuous)
{
    const unsigned int num_segments = 4;


    float segment_lengths_linear[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    segment_lengths_linear[0] = 1.0;
    segment_lengths_linear[1] = 1.0;
    segment_lengths_linear[2] = 1.0;


    FirmwareRobotPathParameters_t path_parameters;
    path_parameters.max_allowable_linear_acceleration = 1;
    path_parameters.max_allowable_linear_speed        = 1;
    path_parameters.num_elements                      = num_segments;
    path_parameters.initial_linear_speed              = sqrtf(7);
    path_parameters.final_linear_speed                = 1;

    float speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    speed_profile[0] = 4;
    speed_profile[1] = 3;
    speed_profile[2] = 2;
    speed_profile[3] = 1;


    TrajectoryPlannerGenerationStatus_t status =
        app_trajectory_planner_impl_modifySpeedsToBeBackwardsContinuous(
            path_parameters.initial_linear_speed, segment_lengths_linear,
            path_parameters.max_allowable_linear_acceleration,
            path_parameters.num_elements, speed_profile);
    EXPECT_EQ(status, OK);

    EXPECT_NEAR(speed_profile[0], sqrt(7), 0.0001);
    EXPECT_NEAR(speed_profile[1], sqrt(5), 0.0001);
    EXPECT_NEAR(speed_profile[2], sqrt(3), 0.0001);
    EXPECT_NEAR(speed_profile[3], 1, 0.0001);
}

TEST(TrajectoryPlannerImplTest, test_backwards_continuity_variable_segment_length_input)
{
    const unsigned int num_segments = 4;


    float segment_lengths_linear[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    segment_lengths_linear[0] = 1.0;
    segment_lengths_linear[1] = 2.0;
    segment_lengths_linear[2] = 3.0;


    FirmwareRobotPathParameters_t path_parameters;
    path_parameters.max_allowable_linear_acceleration = 1;
    path_parameters.max_allowable_linear_speed        = 1;
    path_parameters.num_elements                      = num_segments;
    path_parameters.initial_linear_speed              = sqrtf(7);
    path_parameters.final_linear_speed                = 1;

    float speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    speed_profile[0] = 4;
    speed_profile[1] = 3;
    speed_profile[2] = 10;
    speed_profile[3] = 1;

    TrajectoryPlannerGenerationStatus_t status =
        app_trajectory_planner_impl_modifySpeedsToBeBackwardsContinuous(
            path_parameters.initial_linear_speed, segment_lengths_linear,
            path_parameters.max_allowable_linear_acceleration,
            path_parameters.num_elements, speed_profile);
    EXPECT_EQ(status, OK);

    EXPECT_NEAR(speed_profile[0], sqrt(11), 0.0001);
    EXPECT_NEAR(speed_profile[1], 3, 0.0001);
    EXPECT_NEAR(speed_profile[2], sqrt(7), 0.0001);
    EXPECT_NEAR(speed_profile[3], 1, 0.0001);
}

TEST(
    TrajectoryPlannerImplTest,
    test_backwards_continuity_variable_segment_length_input_and_initial_velocity_too_high)
{
    // Create an 'initial velocity too high' state by requesting an initial velocity that
    // is much greater than physically possible given the path length and acceleration
    const unsigned int num_segments = 4;


    float segment_lengths_linear[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    segment_lengths_linear[0] = 1.0;
    segment_lengths_linear[1] = 2.0;
    segment_lengths_linear[2] = 3.0;

    FirmwareRobotPathParameters_t path_parameters;
    path_parameters.max_allowable_linear_acceleration = 1;
    path_parameters.max_allowable_linear_speed        = 1;
    path_parameters.num_elements                      = num_segments;
    path_parameters.initial_linear_speed              = 20;
    path_parameters.final_linear_speed                = 1;

    float speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    speed_profile[0] = 20;
    speed_profile[1] = 3;
    speed_profile[2] = 10;
    speed_profile[3] = 1;

    TrajectoryPlannerGenerationStatus_t status =
        app_trajectory_planner_impl_modifySpeedsToBeBackwardsContinuous(
            path_parameters.initial_linear_speed, segment_lengths_linear,
            path_parameters.max_allowable_linear_acceleration,
            path_parameters.num_elements, speed_profile);

    EXPECT_EQ(status, INITIAL_VELOCITY_TOO_HIGH);
}

TEST(TrajectoryPlannerImplTest,
     test_backwards_continuity_constant_segment_length_input_to_angular_profile)
{
    const unsigned int num_segments = 4;

    float segment_lengths_angular[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    segment_lengths_angular[0] = 1.0;
    segment_lengths_angular[1] = 1.0;
    segment_lengths_angular[2] = 1.0;


    FirmwareRobotPathParameters_t path_parameters;
    path_parameters.max_allowable_angular_acceleration = 1;
    path_parameters.max_allowable_angular_speed        = 1;
    path_parameters.num_elements                       = num_segments;

    float speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    speed_profile[0] = 0;
    speed_profile[1] = 3;
    speed_profile[2] = 2;
    speed_profile[3] = 0;


    TrajectoryPlannerGenerationStatus_t status =
        app_trajectory_planner_impl_modifySpeedsToBeBackwardsContinuous(
            0.0f, segment_lengths_angular,
            path_parameters.max_allowable_angular_acceleration,
            path_parameters.num_elements, speed_profile);
    EXPECT_EQ(status, OK);

    EXPECT_NEAR(speed_profile[0], 0, 0.0001);
    EXPECT_NEAR(speed_profile[1], sqrt(4), 0.0001);
    EXPECT_NEAR(speed_profile[2], sqrt(2), 0.0001);
    EXPECT_NEAR(speed_profile[3], 0, 0.0001);
}

TEST(TrajectoryPlannerImplTest,
     test_backwards_continuity_variable_segment_length_input_angular_profile)
{
    const unsigned int num_segments = 4;

    float segment_lengths[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    segment_lengths[0] = 1.0;
    segment_lengths[1] = 2.0;
    segment_lengths[2] = 3.0;

    FirmwareRobotPathParameters_t path_parameters;
    path_parameters.max_allowable_angular_acceleration = 1;
    path_parameters.max_allowable_angular_speed        = 1;
    path_parameters.num_elements                       = num_segments;

    float speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    speed_profile[0] = 0;
    speed_profile[1] = 3;
    speed_profile[2] = 10;
    speed_profile[3] = 0;

    TrajectoryPlannerGenerationStatus_t status =
        app_trajectory_planner_impl_modifySpeedsToBeBackwardsContinuous(
            0.0f, segment_lengths, path_parameters.max_allowable_angular_acceleration,
            path_parameters.num_elements, speed_profile);
    EXPECT_EQ(status, OK);

    EXPECT_NEAR(speed_profile[0], 0, 0.0001);
    EXPECT_NEAR(speed_profile[1], 3, 0.0001);
    EXPECT_NEAR(speed_profile[2], sqrt(6), 0.0001);
    EXPECT_NEAR(speed_profile[3], 0, 0.0001);
}

TEST(TrajectoryPlannerimplTest, test_generate_maximum_speed_curve_for_a_straight_line)
{
    float max_allowable_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    FirmwareRobotPathParameters_t path_parameters;
    path_parameters.max_allowable_linear_acceleration = 1;
    path_parameters.max_allowable_linear_speed        = FLT_MAX;
    path_parameters.num_elements         = TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS - 1;
    path_parameters.initial_linear_speed = 1;
    path_parameters.final_linear_speed   = 1;
    path_parameters.path                 = {.x = {0, 0, 1, 0}, .y = {0, 0, 1, 0}};
    path_parameters.t_start              = 0;
    path_parameters.t_end                = 1;

    app_trajectory_planner_impl_getMaximumSpeedProfile(
        path_parameters.path, path_parameters.num_elements, path_parameters.t_start,
        path_parameters.t_end, path_parameters.max_allowable_linear_acceleration,
        path_parameters.max_allowable_linear_speed, max_allowable_speed_profile);

    for (unsigned int i = 0; i < path_parameters.num_elements; i++)
    {
        EXPECT_GE(max_allowable_speed_profile[i], 1e+10);
    }
}

TEST(TrajectoryPlannerImplTest,
     test_generate_forwards_and_backwards_continuous_speed_profiles_angular_and_linear)

{
    Polynomial1dOrder3_t orientation_profile = {.coefficients = {0, 0, 1, 0}};
    Polynomial2dOrder3_t path                = {.x = {0, 0, 1, 0}, .y = {0, 0, 1, 0}};
    // Generate parameters for a trajectory
    FirmwareRobotPathParameters_t path_parameters{
        .path                               = path,
        .orientation_profile                = orientation_profile,
        .t_start                            = 1,
        .t_end                              = 2,
        .num_elements                       = 10,
        .max_allowable_linear_acceleration  = 1,
        .max_allowable_linear_speed         = 1,
        .max_allowable_angular_acceleration = 1,
        .max_allowable_angular_speed        = 1,
        .initial_linear_speed               = 0,
        .final_linear_speed                 = 0,

    };

    float linear_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    float angular_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    float max_allowable_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    float segment_lengths_linear[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    float segment_lengths_angular[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    float angular_states[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    float x_states[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    float y_states[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    // Get the segment states and the segment lengths
    app_trajectory_planner_impl_generate1dSegmentNodesAndLengths(
        path_parameters.t_start, path_parameters.t_end,
        path_parameters.orientation_profile, path_parameters.num_elements, angular_states,
        segment_lengths_angular);
    app_trajectory_planner_impl_generate2dSegmentNodesAndLengths(
        path_parameters.t_start, path_parameters.t_end, path_parameters.path,
        path_parameters.num_elements, x_states, y_states, segment_lengths_linear);

    // Get the forwards linear speed profile
    app_trajectory_planner_impl_getMaximumSpeedProfile(
        path_parameters.path, path_parameters.num_elements, path_parameters.t_start,
        path_parameters.t_end, path_parameters.max_allowable_linear_acceleration,
        path_parameters.max_allowable_linear_speed, max_allowable_speed_profile);
    app_trajectory_planner_impl_createForwardsContinuousSpeedProfile(
        path_parameters.final_linear_speed, segment_lengths_linear,
        max_allowable_speed_profile, path_parameters.max_allowable_linear_acceleration,
        path_parameters.initial_linear_speed, path_parameters.num_elements,
        linear_speed_profile);

    // Get the forwards angular speed profile
    Polynomial2dOrder3_t orientation_2d = {.x = path_parameters.orientation_profile,
                                           .y = {0, 0, 0, 0}};
    app_trajectory_planner_impl_getMaximumSpeedProfile(
        orientation_2d, path_parameters.num_elements, path_parameters.t_start,
        path_parameters.t_end, path_parameters.max_allowable_angular_acceleration,
        path_parameters.max_allowable_angular_speed, max_allowable_speed_profile);
    app_trajectory_planner_impl_createForwardsContinuousSpeedProfile(
        0.0f, segment_lengths_angular, max_allowable_speed_profile,
        path_parameters.max_allowable_angular_acceleration, 0.0f,
        path_parameters.num_elements, angular_speed_profile);

    // Check that the values of the acceleration period are correct
    EXPECT_NEAR(linear_speed_profile[0], 0, 0.0001);
    EXPECT_NEAR(linear_speed_profile[1], 0.5606, 0.0001);
    EXPECT_NEAR(linear_speed_profile[2], 0.7928, 0.0001);
    EXPECT_NEAR(linear_speed_profile[3], 0.9709, 0.0001);
    EXPECT_NEAR(linear_speed_profile[4], 1.0, 0.0001);

    EXPECT_NEAR(angular_speed_profile[0], 0, 0.0001);
    EXPECT_NEAR(angular_speed_profile[1], 0.4714, 0.0001);
    EXPECT_NEAR(angular_speed_profile[2], 0.6666, 0.0001);
    EXPECT_NEAR(angular_speed_profile[3], 0.8165, 0.0001);
    EXPECT_NEAR(angular_speed_profile[4], 0.9428, 0.0001);

    // Check that the path speed is limited by the maximum value set in the path
    // parameters
    for (unsigned int i = 5; i < path_parameters.num_elements - 2; i++)
    {
        EXPECT_FLOAT_EQ(linear_speed_profile[i],
                        path_parameters.max_allowable_linear_speed);
        EXPECT_FLOAT_EQ(angular_speed_profile[i],
                        path_parameters.max_allowable_angular_speed);
    }

    app_trajectory_planner_impl_modifySpeedsToBeBackwardsContinuous(
        path_parameters.initial_linear_speed, segment_lengths_linear,
        path_parameters.max_allowable_linear_acceleration, path_parameters.num_elements,
        linear_speed_profile);
    app_trajectory_planner_impl_modifySpeedsToBeBackwardsContinuous(
        0.0f, segment_lengths_angular, path_parameters.max_allowable_angular_acceleration,
        path_parameters.num_elements, angular_speed_profile);

    // Check that the decceleration period is correct
    // Check that the values of the acceleration period are correct
    EXPECT_NEAR(linear_speed_profile[path_parameters.num_elements - 5], 1 - 0, 0.0001);
    EXPECT_NEAR(linear_speed_profile[path_parameters.num_elements - 4], 0.9709, 0.0001);
    EXPECT_NEAR(linear_speed_profile[path_parameters.num_elements - 3], 0.7928, 0.0001);
    EXPECT_NEAR(linear_speed_profile[path_parameters.num_elements - 2], 0.5606, 0.0001);
    EXPECT_NEAR(linear_speed_profile[path_parameters.num_elements - 1],
                path_parameters.final_linear_speed, 0.0001);

    EXPECT_NEAR(angular_speed_profile[path_parameters.num_elements - 5], 0.9428, 0.0001);
    EXPECT_NEAR(angular_speed_profile[path_parameters.num_elements - 4], 0.8165, 0.0001);
    EXPECT_NEAR(angular_speed_profile[path_parameters.num_elements - 3], 0.6666, 0.0001);
    EXPECT_NEAR(angular_speed_profile[path_parameters.num_elements - 2], 0.4714, 0.0001);
    EXPECT_NEAR(angular_speed_profile[path_parameters.num_elements - 1], 0, 0.0001);
}

TEST(TrajectoryPlannerImplTest, test_rebalance_trajectory_segment_to_mach_duration)
{
    float segment_lengths_meters[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    float speeds[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    // Create an artificial trajectory here for testing
    speeds[0]                  = 1;
    speeds[1]                  = 4;
    float desired_time_seconds = 0.01f;

    segment_lengths_meters[0] = 1.0;

    speeds[1] = app_trajectory_planner_impl_calculateSpeedToMatchDuration(
        speeds[0], desired_time_seconds, segment_lengths_meters[0]);
    EXPECT_FLOAT_EQ(speeds[1], 199);
    EXPECT_FLOAT_EQ(speeds[0], 1);
}

TEST(TrajectoryPlannerImplTest,
     test_rebalance_trajectory_segment_to_match_duration_decreasing_speed)
{
    // This test was added to address a bug where the trajectory rebalancing could not
    // account for decreasing speeds. This led to the accumulation of speed when it was
    // supposed to be decreasing
    float segment_lengths_meters[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    float speeds[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    // Create an artificial trajectory here for testing
    speeds[0]                  = 4;
    speeds[1]                  = 3;
    float desired_time_seconds = 0.15f;

    segment_lengths_meters[0] = 0.35f;

    speeds[1] = app_trajectory_planner_impl_calculateSpeedToMatchDuration(
        speeds[0], desired_time_seconds, segment_lengths_meters[0]);
    EXPECT_NEAR(speeds[1], 0.666666f, 0.0001f);
    EXPECT_FLOAT_EQ(speeds[0], 4.0f);
}
