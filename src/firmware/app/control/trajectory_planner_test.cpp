extern "C"
{
#include "firmware/app/control/trajectory_planner.h"
}

#include <gtest/gtest.h>
#include <math.h>

#include <vector>

#include "firmware/shared/math/polynomial_1d.h"
#include "firmware/shared/math/polynomial_2d.h"
#include "firmware/shared/math/vector_2d.h"
#include "software/new_geom/vector.h"

class TrajectoryPlannerTest : public testing::Test
{
   protected:
    virtual void SetUp() {}

    virtual void TearDown() {}

    static std::vector<Vector> getDirectionVectorsFromPositionTrajectory(
        PositionTrajectory_t* position_trajectory)
    {
        PositionTrajectoryElement_t* position_elements =
            position_trajectory->trajectory_elements;
        std::vector<Vector> unit_vectors;

        for (unsigned int i = 0;
             i < position_trajectory->path_parameters.num_segments - 1; i++)
        {
            const float delta_x =
                position_elements[i + 1].position.x - position_elements[i].position.x;
            const float delta_y =
                position_elements[i + 1].position.y - position_elements[i].position.y;

            // Copy data into the Vector
            Vector direction = Vector(delta_x, delta_y).normalize(1);
            unit_vectors.push_back(direction);
        }

        // Assume that the velocity of the final segment is in the direction of the
        // previous
        const float delta_x =
            position_elements[position_trajectory->path_parameters.num_segments - 1]
                .position.x -
            position_elements[position_trajectory->path_parameters.num_segments - 2]
                .position.x;
        const float delta_y =
            position_elements[position_trajectory->path_parameters.num_segments - 1]
                .position.y -
            position_elements[position_trajectory->path_parameters.num_segments - 2]
                .position.y;

        unit_vectors.push_back(Vector(delta_x, delta_y).normalize(1));
        return unit_vectors;
    }
};

TEST_F(TrajectoryPlannerTest, check_end_points_match_path)
{
    Polynomial2dOrder3_t path = {
        .x = {.coefficients = {2, 0, 1, 0}},
        .y = {.coefficients = {1, 0, 1, 0}},

    };
    Polynomial1dOrder3_t orientation_profile = {.coefficients = {0, 0, 0, 0}};

    FirmwareRobotPathParameters_t path_parameters;
    path_parameters.path                = path;
    path_parameters.orientation_profile = orientation_profile;
    path_parameters.t_start             = 0;
    path_parameters.t_end               = 1;
    path_parameters.num_segments =
        TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS;  // For tests the number of segments is
                                              // arbitrary, so use the maximum possible
    path_parameters.max_allowable_linear_acceleration = 3;
    path_parameters.max_allowable_linear_speed        = 3;
    path_parameters.initial_linear_speed              = 0;
    path_parameters.final_linear_speed                = 0;

    PositionTrajectoryElement_t const_param_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    PositionTrajectory_t trajectory = {.trajectory_elements = const_param_elements,
                                       .path_parameters     = path_parameters};

    TrajectoryPlannerGenerationStatus_t status =
        app_trajectory_planner_generateConstantParameterizationPositionTrajectory(
            &trajectory);
    EXPECT_EQ(OK, status);

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


TEST_F(TrajectoryPlannerTest, check_trajectory_path_length_reverse_parameterization)
{
    Polynomial2dOrder3_t path = {
        .x = {.coefficients = {0, 0, 1, 0}},
        .y = {.coefficients = {0, 0, 1, 0}},

    };
    Polynomial1dOrder3_t orientation_profile = {.coefficients = {0, 0, 0, 0}};
    FirmwareRobotPathParameters_t path_parameters;
    path_parameters.path                = path;
    path_parameters.orientation_profile = orientation_profile;
    path_parameters.t_start             = 0;
    path_parameters.t_end               = 1;
    path_parameters.num_segments        = TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS - 1;
    path_parameters.max_allowable_linear_acceleration = 3;
    path_parameters.max_allowable_linear_speed        = 3;
    path_parameters.initial_linear_speed              = 0;
    path_parameters.final_linear_speed                = 0;

    PositionTrajectoryElement_t const_param_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    PositionTrajectory_t trajectory = {.trajectory_elements = const_param_elements,
                                       .path_parameters     = path_parameters};


    TrajectoryPlannerGenerationStatus_t status =
        app_trajectory_planner_generateConstantParameterizationPositionTrajectory(
            &trajectory);
    EXPECT_EQ(OK, status);
    float segment_length_sum = 0;

    // Test by checking the length between points
    // and the length of the total path
    for (uint i = 0; i < trajectory.path_parameters.num_segments - 1; i++)
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
    EXPECT_NEAR(segment_length_sum, sqrt(2), 0.001);
}

TEST_F(TrajectoryPlannerTest,
       check_trajectory_end_points_match_path_reverse_parameterization)
{
    Polynomial2dOrder3_t path = {
        .x = {.coefficients = {0, 0, 1, 0}},
        .y = {.coefficients = {0, 0, 1, 0}},

    };
    Polynomial1dOrder3_t orientation_profile = {.coefficients = {0, 0, 0, 0}};
    FirmwareRobotPathParameters_t path_parameters;
    path_parameters.path                = path;
    path_parameters.orientation_profile = orientation_profile;
    path_parameters.t_start             = 0;
    path_parameters.t_end               = 1;
    path_parameters.num_segments        = TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS - 1;
    path_parameters.max_allowable_linear_acceleration = 3;
    path_parameters.max_allowable_linear_speed        = 3;
    path_parameters.initial_linear_speed              = 0;
    path_parameters.final_linear_speed                = 0;

    PositionTrajectoryElement_t const_param_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    PositionTrajectory_t trajectory = {.trajectory_elements = const_param_elements,
                                       .path_parameters     = path_parameters};


    TrajectoryPlannerGenerationStatus_t status =
        app_trajectory_planner_generateConstantParameterizationPositionTrajectory(
            &trajectory);
    EXPECT_EQ(OK, status);

    EXPECT_NEAR(
        trajectory.trajectory_elements[trajectory.path_parameters.num_segments - 1]
            .position.x,
        shared_polynomial2d_getValueOrder3(path, path_parameters.t_end).x, 0.01);
    EXPECT_NEAR(
        trajectory.trajectory_elements[trajectory.path_parameters.num_segments - 1]
            .position.y,
        shared_polynomial2d_getValueOrder3(path, path_parameters.t_end).y, 0.01);

    EXPECT_NEAR(trajectory.trajectory_elements[0].position.x,
                shared_polynomial2d_getValueOrder3(path, path_parameters.t_start).x,
                0.01);
    EXPECT_NEAR(trajectory.trajectory_elements[0].position.y,
                shared_polynomial2d_getValueOrder3(path, path_parameters.t_start).y,
                0.01);
}

TEST_F(TrajectoryPlannerTest, check_trajectory_speed_profile_reverse_parameterization)
{
    Polynomial2dOrder3_t path = {
        .x = {.coefficients = {0, 0, 1, 0}},
        .y = {.coefficients = {0, 0, 1, 0}},

    };
    Polynomial1dOrder3_t orientation_profile = {.coefficients = {0, 0, 0, 0}};
    FirmwareRobotPathParameters_t path_parameters;
    path_parameters.path                = path;
    path_parameters.orientation_profile = orientation_profile;
    path_parameters.t_start             = 0;
    path_parameters.t_end               = 1;
    path_parameters.num_segments        = TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS - 1;
    path_parameters.max_allowable_linear_acceleration = 3;
    path_parameters.max_allowable_linear_speed        = 3;
    path_parameters.initial_linear_speed              = 0;
    path_parameters.final_linear_speed                = 0;

    PositionTrajectoryElement_t const_param_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    PositionTrajectory_t trajectory = {.trajectory_elements = const_param_elements,
                                       .path_parameters     = path_parameters};


    TrajectoryPlannerGenerationStatus_t status =
        app_trajectory_planner_generateConstantParameterizationPositionTrajectory(
            &trajectory);
    EXPECT_EQ(OK, status);

    float forwards_linear_speed[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    float forwards_angular_speed[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    float forwards_orientation[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    float forwards_position_x[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    float forwards_position_y[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    float forwards_time[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    // Copy the 'forwards' velocity profile (as if t1 < t2)
    for (unsigned int i = 0; i < path_parameters.num_segments; i++)
    {
        forwards_linear_speed[i]  = trajectory.trajectory_elements[i].linear_speed;
        forwards_angular_speed[i] = trajectory.trajectory_elements[i].angular_speed;
        forwards_orientation[i]   = trajectory.trajectory_elements[i].orientation;
        forwards_position_x[i]    = trajectory.trajectory_elements[i].position.x;
        forwards_position_y[i]    = trajectory.trajectory_elements[i].position.y;
        forwards_time[i]          = trajectory.trajectory_elements[i].time;
    }

    // Reverse the parameterization
    float temp                         = trajectory.path_parameters.t_start;
    trajectory.path_parameters.t_start = trajectory.path_parameters.t_end;
    trajectory.path_parameters.t_end   = temp;

    status = app_trajectory_planner_generateConstantParameterizationPositionTrajectory(
        &trajectory);
    EXPECT_EQ(OK, status);

    for (unsigned int i = 0; i < trajectory.path_parameters.num_segments; i++)
    {
        EXPECT_NEAR(forwards_linear_speed[i],
                    trajectory.trajectory_elements[path_parameters.num_segments - 1 - i]
                        .linear_speed,
                    0.0001);
        EXPECT_NEAR(forwards_angular_speed[i],
                    trajectory.trajectory_elements[path_parameters.num_segments - 1 - i]
                        .angular_speed,
                    0.0001);
        EXPECT_NEAR(forwards_orientation[i],
                    trajectory.trajectory_elements[path_parameters.num_segments - 1 - i]
                        .orientation,
                    0.0001);
        EXPECT_NEAR(forwards_position_x[i],
                    trajectory.trajectory_elements[path_parameters.num_segments - 1 - i]
                        .position.x,
                    0.0001);
        EXPECT_NEAR(forwards_position_y[i],
                    trajectory.trajectory_elements[path_parameters.num_segments - 1 - i]
                        .position.y,
                    0.0001);
        EXPECT_NEAR(forwards_time[i], trajectory.trajectory_elements[i].time, 0.0001);
    }
}

TEST_F(TrajectoryPlannerTest, dynamics_dont_exceed_maximums_curved_path)
{
    Polynomial2dOrder3_t path = {
        .x = {.coefficients = {0, 2, 1, 0}},
        .y = {.coefficients = {3, 0, 1, 0}},

    };
    Polynomial1dOrder3_t orientation_profile = {.coefficients = {0, 0, 0, 0}};
    FirmwareRobotPathParameters_t path_parameters;
    path_parameters.path                = path;
    path_parameters.orientation_profile = orientation_profile;
    path_parameters.t_start             = -1;
    path_parameters.t_end               = 1;
    path_parameters.num_segments        = 500;  // Use a low segment density for variety
    path_parameters.max_allowable_linear_acceleration = 5;
    path_parameters.max_allowable_linear_speed        = 5;
    path_parameters.initial_linear_speed              = 1.87;
    path_parameters.final_linear_speed                = 5;

    PositionTrajectoryElement_t const_param_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    float speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    PositionTrajectory_t trajectory = {.trajectory_elements = const_param_elements,
                                       .path_parameters     = path_parameters};

    TrajectoryPlannerGenerationStatus_t status =
        app_trajectory_planner_generateConstantParameterizationPositionTrajectory(
            &trajectory);
    EXPECT_EQ(OK, status);

    for (unsigned int i = 0; i < path_parameters.num_segments; i++)
    {
        EXPECT_TRUE(const_param_elements[i].linear_speed <=
                    path_parameters.max_allowable_linear_speed);
    }

    EXPECT_NEAR(const_param_elements[path_parameters.num_segments - 1].linear_speed,
                path_parameters.final_linear_speed, 0.0001);

    EXPECT_FLOAT_EQ(const_param_elements[0].linear_speed,
                    path_parameters.initial_linear_speed);

    EXPECT_NEAR(
        trajectory.trajectory_elements[trajectory.path_parameters.num_segments - 1]
            .position.x,
        shared_polynomial2d_getValueOrder3(path, path_parameters.t_end).x, 0.025);
    EXPECT_NEAR(
        trajectory.trajectory_elements[trajectory.path_parameters.num_segments - 1]
            .position.y,
        shared_polynomial2d_getValueOrder3(path, path_parameters.t_end).y, 0.025);

    EXPECT_NEAR(trajectory.trajectory_elements[0].position.x,
                shared_polynomial2d_getValueOrder3(path, path_parameters.t_start).x,
                0.025);
    EXPECT_NEAR(trajectory.trajectory_elements[0].position.y,
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
    path_parameters.path                              = path;
    path_parameters.t_start                           = 0;
    path_parameters.t_end                             = 1;
    path_parameters.num_segments                      = 500;
    path_parameters.max_allowable_linear_acceleration = 3;
    path_parameters.max_allowable_linear_speed        = 3;
    path_parameters.initial_linear_speed              = 0;
    path_parameters.final_linear_speed                = 0;

    PositionTrajectoryElement_t const_param_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    PositionTrajectory_t trajectory = {.trajectory_elements = const_param_elements,
                                       .path_parameters     = path_parameters};

    PositionTrajectory_t const_interp_trajectory;

    PositionTrajectoryElement_t
        const_interp_traj_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    const_interp_trajectory.trajectory_elements = const_interp_traj_elements;

    TrajectoryPlannerGenerationStatus_t status =
        app_trajectory_planner_generateConstantInterpolationPeriodPositionTrajectory(
            &trajectory, &const_interp_trajectory, 0.001);

    EXPECT_EQ(OK, status);

    EXPECT_NEAR(
        trajectory.trajectory_elements[trajectory.path_parameters.num_segments - 1]
            .position.x,
        const_interp_trajectory
            .trajectory_elements[const_interp_trajectory.path_parameters.num_segments - 1]
            .position.x,
        0.001);
    EXPECT_NEAR(
        trajectory.trajectory_elements[trajectory.path_parameters.num_segments - 1]
            .position.y,
        const_interp_trajectory
            .trajectory_elements[const_interp_trajectory.path_parameters.num_segments - 1]
            .position.y,
        0.001);
    EXPECT_NEAR(
        trajectory.trajectory_elements[trajectory.path_parameters.num_segments - 1].time,
        const_interp_trajectory
            .trajectory_elements[const_interp_trajectory.path_parameters.num_segments - 1]
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
    Polynomial1dOrder3_t orientation_profile = {.coefficients = {0, 0, 0, 0}

    };

    FirmwareRobotPathParameters_t path_parameters;
    path_parameters.path                              = path;
    path_parameters.t_start                           = 0;
    path_parameters.t_end                             = 1;
    path_parameters.num_segments                      = 500;
    path_parameters.max_allowable_linear_acceleration = 3;
    path_parameters.max_allowable_linear_speed        = 3;
    path_parameters.initial_linear_speed              = 0;
    path_parameters.final_linear_speed                = 0;

    PositionTrajectoryElement_t const_param_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    PositionTrajectory_t trajectory = {.trajectory_elements = const_param_elements,
                                       .path_parameters     = path_parameters};

    PositionTrajectory_t const_interp_trajectory;

    PositionTrajectoryElement_t
        const_interp_traj_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    const_interp_trajectory.trajectory_elements = const_interp_traj_elements;

    TrajectoryPlannerGenerationStatus_t status =
        app_trajectory_planner_generateConstantInterpolationPeriodPositionTrajectory(
            &trajectory, &const_interp_trajectory, 0.001);

    EXPECT_EQ(OK, status);

    EXPECT_NEAR(
        trajectory.trajectory_elements[trajectory.path_parameters.num_segments - 1]
            .position.x,
        const_interp_trajectory
            .trajectory_elements[const_interp_trajectory.path_parameters.num_segments - 1]
            .position.x,
        0.001);
    EXPECT_NEAR(
        trajectory.trajectory_elements[trajectory.path_parameters.num_segments - 1]
            .position.y,
        const_interp_trajectory
            .trajectory_elements[const_interp_trajectory.path_parameters.num_segments - 1]
            .position.y,
        0.001);
    EXPECT_NEAR(
        trajectory.trajectory_elements[trajectory.path_parameters.num_segments - 1].time,
        const_interp_trajectory
            .trajectory_elements[const_interp_trajectory.path_parameters.num_segments - 1]
            .time,
        0.001);

    EXPECT_NEAR(trajectory.trajectory_elements[0].position.x,
                const_interp_trajectory.trajectory_elements[0].position.x, 0.001);
    EXPECT_NEAR(trajectory.trajectory_elements[0].position.y,
                const_interp_trajectory.trajectory_elements[0].position.x, 0.001);
    EXPECT_NEAR(trajectory.trajectory_elements[0].time,
                const_interp_trajectory.trajectory_elements[0].time, 0.001);
}


// This test generates a scenario where there is not enough elements in the
// TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS specified array as there path is quite long and
// the maximum speed of the path is low. This means that most (likely all) arc-length
// segments require multiple interpolation periods to traverse, and therefore requires
// more space than the constant arc_length trajectory that is already 1 element away
// from TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS
TEST_F(TrajectoryPlannerTest, test_get_constant_time_interpolation_too_many_elements)
{
    Polynomial2dOrder3_t path = {
        .x = {.coefficients = {2, 0, 1, 0}},
        .y = {.coefficients = {1, 0, 1, 0}},

    };

    FirmwareRobotPathParameters_t path_parameters;
    path_parameters.path         = path;
    path_parameters.t_start      = 0;
    path_parameters.t_end        = 2;
    path_parameters.num_segments = TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS - 1;
    path_parameters.max_allowable_linear_acceleration = 10;
    path_parameters.max_allowable_linear_speed        = 0.1;
    path_parameters.initial_linear_speed              = 0;
    path_parameters.final_linear_speed                = 0;

    PositionTrajectoryElement_t const_param_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    PositionTrajectory_t trajectory = {.trajectory_elements = const_param_elements,
                                       .path_parameters     = path_parameters};

    PositionTrajectory_t const_interp_trajectory;

    PositionTrajectoryElement_t
        const_interp_traj_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    const_interp_trajectory.path_parameters     = path_parameters;
    const_interp_trajectory.trajectory_elements = const_interp_traj_elements;

    // Calculate the constant-interpolation period equivalent of the trajectory
    TrajectoryPlannerGenerationStatus_t status =
        app_trajectory_planner_generateConstantInterpolationPeriodPositionTrajectory(
            &trajectory, &const_interp_trajectory, 0.001);

    EXPECT_EQ(INTERPOLATION_ELEMENT_MAXED_OUT, status);
}

TEST_F(TrajectoryPlannerTest, test_assert_cannot_reach_final_velocity)
{
    Polynomial2dOrder3_t path = {
        .x = {.coefficients = {0, 0, 1, 0}},
        .y = {.coefficients = {0, 0, 1, 0}},

    };

    FirmwareRobotPathParameters_t path_parameters;
    path_parameters.path         = path;
    path_parameters.t_start      = 0;
    path_parameters.t_end        = 1;
    path_parameters.num_segments = TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS - 1;
    path_parameters.max_allowable_linear_acceleration = 3;
    path_parameters.max_allowable_linear_speed        = 2000;
    path_parameters.initial_linear_speed              = 0;
    path_parameters.final_linear_speed                = 3000;

    PositionTrajectoryElement_t const_param_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    PositionTrajectory_t trajectory = {.trajectory_elements = const_param_elements,
                                       .path_parameters     = path_parameters};


    TrajectoryPlannerGenerationStatus_t status =
        app_trajectory_planner_generateConstantParameterizationPositionTrajectory(
            &trajectory);

    EXPECT_EQ(FINAL_VELOCITY_TOO_HIGH, status);
}

// Set the initial velocity to a speed that is so high the robot cannot possible slow
// down in time to follow the path
TEST_F(TrajectoryPlannerTest, test_assert_initial_velocity_too_high)
{
    Polynomial2dOrder3_t path = {
        .x = {.coefficients = {0, 0, 1, 0}},
        .y = {.coefficients = {0, 0, 1, 0}},

    };

    FirmwareRobotPathParameters_t path_parameters;
    path_parameters.path         = path;
    path_parameters.t_start      = 0;
    path_parameters.t_end        = 3;
    path_parameters.num_segments = TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS;
    path_parameters.max_allowable_linear_acceleration = 3;
    path_parameters.max_allowable_linear_speed        = 50;
    path_parameters.initial_linear_speed              = 20;
    path_parameters.final_linear_speed                = 0;

    PositionTrajectoryElement_t const_param_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    PositionTrajectory_t trajectory = {.trajectory_elements = const_param_elements,
                                       .path_parameters     = path_parameters};


    TrajectoryPlannerGenerationStatus_t status =
        app_trajectory_planner_generateConstantParameterizationPositionTrajectory(
            &trajectory);

    EXPECT_EQ(INITIAL_VELOCITY_TOO_HIGH, status);
}

TEST_F(TrajectoryPlannerTest, velocity_trajectory_straight_line_high_acceleration)

{
    Polynomial2dOrder3_t path = {
        .x = {.coefficients = {0, 0, 1, 0}},
        .y = {.coefficients = {0, 0, 1, 0}},

    };

    FirmwareRobotPathParameters_t path_parameters;
    path_parameters.path                              = path;
    path_parameters.t_start                           = 0;
    path_parameters.t_end                             = 3;
    path_parameters.num_segments                      = 1000;
    path_parameters.max_allowable_linear_acceleration = 10;
    path_parameters.max_allowable_linear_speed        = 9;
    path_parameters.initial_linear_speed              = 2;
    path_parameters.final_linear_speed                = 5;

    PositionTrajectoryElement_t const_param_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    float max_allowable_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    PositionTrajectory_t position_trajectory = {
        .trajectory_elements = const_param_elements, .path_parameters = path_parameters};

    TrajectorySegment_t segment_lengths[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    app_trajectory_planner_getMaxAllowableSpeedProfile_2(
        position_trajectory.path_parameters.path,
        position_trajectory.path_parameters.num_segments,
        position_trajectory.path_parameters.t_start,
        position_trajectory.path_parameters.t_end,
        position_trajectory.path_parameters.max_allowable_linear_acceleration,
        max_allowable_speed_profile);
    app_trajectory_planner_generateStatesAndReturnSegmentLengths(&position_trajectory,
                                                                 segment_lengths);

    TrajectoryPlannerGenerationStatus_t status = OK;
    status = app_trajectory_planner_generateForwardsContinuousLinearSpeedProfile_2(
        &position_trajectory, max_allowable_speed_profile, segment_lengths);
    EXPECT_EQ(status, OK);

    app_trajectory_planner_generateForwardsContinuousAngularSpeedProfile_2(
        &position_trajectory, segment_lengths);

    status = app_trajectory_planner_generateBackwardsContinuousSpeedProfile_2(
        &position_trajectory, segment_lengths);
    EXPECT_EQ(status, OK);

    app_trajectory_planner_generatePositionTrajectoryTimeProfile_2(&position_trajectory,
                                                                   segment_lengths);

    VelocityTrajectory_t velocity_trajectory;
    VelocityTrajectoryElement_t velocity_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    velocity_trajectory.trajectory_elements = velocity_elements;

    app_trajectory_planner_generateVelocityTrajectory_2(&position_trajectory,
                                                        &velocity_trajectory);


    // Run test class function to get all the unit vectors for the velocity at each
    // point
    // on the trajectory
    std::vector<Vector> direction_unit_vectors =
        getDirectionVectorsFromPositionTrajectory(&position_trajectory);

    // Check that every velocity has the correct magnitude
    for (unsigned int i = 0; i < velocity_trajectory.path_parameters.num_segments; i++)
    {
        const float element_speed =
            sqrt(pow(velocity_trajectory.trajectory_elements[i].linear_velocity.x, 2) +
                 pow(velocity_trajectory.trajectory_elements[i].linear_velocity.y, 2));
        EXPECT_NEAR(position_trajectory.trajectory_elements[i].linear_speed,
                    element_speed, 0.001);

        EXPECT_NEAR(direction_unit_vectors[i].x() *
                        position_trajectory.trajectory_elements[i].linear_speed,
                    velocity_trajectory.trajectory_elements[i].linear_velocity.x, 0.001);
        EXPECT_NEAR(direction_unit_vectors[i].y() *
                        position_trajectory.trajectory_elements[i].linear_speed,
                    velocity_trajectory.trajectory_elements[i].linear_velocity.y, 0.001);
    }
}

TEST_F(TrajectoryPlannerTest, velocity_trajectory_parabola_path_high_acceleration)
{
    Polynomial2dOrder3_t path = {
        .x = {.coefficients = {0, 0, 1, 0}},
        .y = {.coefficients = {0, 2, 0, 0}},

    };

    FirmwareRobotPathParameters_t path_parameters;
    path_parameters.path                              = path;
    path_parameters.t_start                           = -1;
    path_parameters.t_end                             = 1;
    path_parameters.num_segments                      = 1000;
    path_parameters.max_allowable_linear_acceleration = 10;
    path_parameters.max_allowable_linear_speed        = 5;
    path_parameters.initial_linear_speed              = 0;
    path_parameters.final_linear_speed                = 0;

    PositionTrajectoryElement_t const_param_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    float max_allowable_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    PositionTrajectory_t position_trajectory = {
        .trajectory_elements = const_param_elements, .path_parameters = path_parameters};

    TrajectorySegment_t segment_lengths[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    app_trajectory_planner_getMaxAllowableSpeedProfile_2(
        position_trajectory.path_parameters.path,
        position_trajectory.path_parameters.num_segments,
        position_trajectory.path_parameters.t_start,
        position_trajectory.path_parameters.t_end,
        position_trajectory.path_parameters.max_allowable_linear_acceleration,
        max_allowable_speed_profile);
    app_trajectory_planner_generateStatesAndReturnSegmentLengths(&position_trajectory,
                                                                 segment_lengths);

    TrajectoryPlannerGenerationStatus_t status = OK;
    status = app_trajectory_planner_generateForwardsContinuousLinearSpeedProfile_2(
        &position_trajectory, max_allowable_speed_profile, segment_lengths);
    EXPECT_EQ(status, OK);

    app_trajectory_planner_generateForwardsContinuousAngularSpeedProfile_2(
        &position_trajectory, segment_lengths);

    status = app_trajectory_planner_generateBackwardsContinuousSpeedProfile_2(
        &position_trajectory, segment_lengths);
    EXPECT_EQ(status, OK);

    app_trajectory_planner_generatePositionTrajectoryTimeProfile_2(&position_trajectory,
                                                                   segment_lengths);

    VelocityTrajectory_t velocity_trajectory;
    VelocityTrajectoryElement_t velocity_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    velocity_trajectory.trajectory_elements = velocity_elements;

    app_trajectory_planner_generateVelocityTrajectory_2(&position_trajectory,
                                                        &velocity_trajectory);

    // Run test class function to get all the unit vectors for the velocity at each point
    // on the trajectory
    std::vector<Vector> direction_unit_vectors =
        getDirectionVectorsFromPositionTrajectory(&position_trajectory);

    // Check that every velocity has the correct magnitude
    for (unsigned int i = 0; i < velocity_trajectory.path_parameters.num_segments; i++)
    {
        const float element_speed =
            sqrt(pow(velocity_trajectory.trajectory_elements[i].linear_velocity.x, 2) +
                 pow(velocity_trajectory.trajectory_elements[i].linear_velocity.y, 2));
        EXPECT_FLOAT_EQ(position_trajectory.trajectory_elements[i].linear_speed,
                        element_speed);

        EXPECT_FLOAT_EQ(direction_unit_vectors[i].x() *
                            position_trajectory.trajectory_elements[i].linear_speed,
                        velocity_trajectory.trajectory_elements[i].linear_velocity.x);
        EXPECT_FLOAT_EQ(direction_unit_vectors[i].y() *
                            position_trajectory.trajectory_elements[i].linear_speed,
                        velocity_trajectory.trajectory_elements[i].linear_velocity.y);
    }
}

TEST_F(TrajectoryPlannerTest, velocity_trajectory_curved_path_low_acceleration)
{
    Polynomial2dOrder3_t path = {
        .x = {.coefficients = {3, 0, 1, 0}},
        .y = {.coefficients = {0, 2, 0, 0}},

    };

    FirmwareRobotPathParameters_t path_parameters;
    path_parameters.path                              = path;
    path_parameters.t_start                           = -1;
    path_parameters.t_end                             = 1;
    path_parameters.num_segments                      = 1000;
    path_parameters.max_allowable_linear_acceleration = 3;
    path_parameters.max_allowable_linear_speed        = 5;
    path_parameters.initial_linear_speed              = 2;
    path_parameters.final_linear_speed                = 1;

    PositionTrajectoryElement_t const_param_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    float max_allowable_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    PositionTrajectory_t position_trajectory = {
        .trajectory_elements = const_param_elements, .path_parameters = path_parameters};

    VelocityTrajectory_t velocity_trajectory;
    VelocityTrajectoryElement_t velocity_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    velocity_trajectory.trajectory_elements = velocity_elements;

    TrajectorySegment_t segment_lengths[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    app_trajectory_planner_getMaxAllowableSpeedProfile_2(
        position_trajectory.path_parameters.path,
        position_trajectory.path_parameters.num_segments,
        position_trajectory.path_parameters.t_start,
        position_trajectory.path_parameters.t_end,
        position_trajectory.path_parameters.max_allowable_linear_acceleration,
        max_allowable_speed_profile);
    app_trajectory_planner_generateStatesAndReturnSegmentLengths(&position_trajectory,
                                                                 segment_lengths);

    TrajectoryPlannerGenerationStatus_t status = OK;
    status = app_trajectory_planner_generateForwardsContinuousLinearSpeedProfile_2(
        &position_trajectory, max_allowable_speed_profile, segment_lengths);
    EXPECT_EQ(status, OK);

    app_trajectory_planner_generateForwardsContinuousAngularSpeedProfile_2(
        &position_trajectory, segment_lengths);

    status = app_trajectory_planner_generateBackwardsContinuousSpeedProfile_2(
        &position_trajectory, segment_lengths);
    EXPECT_EQ(status, OK);

    app_trajectory_planner_generatePositionTrajectoryTimeProfile_2(&position_trajectory,
                                                                   segment_lengths);

    app_trajectory_planner_generateVelocityTrajectory_2(&position_trajectory,
                                                        &velocity_trajectory);


    // Run test class function to get all the unit vectors for the velocity at each
    // on the trajectory
    std::vector<Vector> direction_unit_vectors =
        getDirectionVectorsFromPositionTrajectory(&position_trajectory);

    // Check that every velocity has the correct magnitude
    for (unsigned int i = 0; i < velocity_trajectory.path_parameters.num_segments; i++)
    {
        const float element_speed =
            sqrt(pow(velocity_trajectory.trajectory_elements[i].linear_velocity.x, 2) +
                 pow(velocity_trajectory.trajectory_elements[i].linear_velocity.y, 2));
        EXPECT_FLOAT_EQ(position_trajectory.trajectory_elements[i].linear_speed,
                        element_speed);

        EXPECT_FLOAT_EQ(direction_unit_vectors[i].x() *
                            position_trajectory.trajectory_elements[i].linear_speed,
                        velocity_trajectory.trajectory_elements[i].linear_velocity.x);
        EXPECT_FLOAT_EQ(direction_unit_vectors[i].y() *
                            position_trajectory.trajectory_elements[i].linear_speed,
                        velocity_trajectory.trajectory_elements[i].linear_velocity.y);
    }
}

TEST_F(TrajectoryPlannerTest, test_robot_state_generation_at_constant_t_intervals)
{
    Polynomial2dOrder3_t path = {
        .x = {.coefficients = {0, 0, 1, 0}},
        .y = {.coefficients = {0, 0, 1, 0}},

    };

    Polynomial1dOrder3_t orientation_profile = {.coefficients = {0, 0, 1, 0}};
    // Generate parameters for a trajectory
    FirmwareRobotPathParameters_t path_parameters;
    path_parameters.path    = path;
    path_parameters.t_start = 0;
    path_parameters.t_end   = 1;
    path_parameters.num_segments =
        TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS;  // For tests the number of segments is
    // arbitrary, so use the maximum possible
    path_parameters.max_allowable_linear_acceleration = 3;
    path_parameters.max_allowable_linear_speed        = 3;
    path_parameters.initial_linear_speed              = 0;
    path_parameters.final_linear_speed                = 0;

    path_parameters.orientation_profile                = orientation_profile;
    path_parameters.max_allowable_angular_acceleration = 9;
    path_parameters.max_allowable_angular_speed        = 6;


    PositionTrajectoryElement_t const_t_segments[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    TrajectorySegment_t segment_lengths[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    PositionTrajectory_t trajectory = {.trajectory_elements = const_t_segments,
                                       .path_parameters     = path_parameters};

    app_trajectory_planner_generateStatesAndReturnSegmentLengths(&trajectory,
                                                                 segment_lengths);

    const float delta_t = (path_parameters.t_end - path_parameters.t_start) /
                          (path_parameters.num_segments - 1);

    // Check that all of the state variables are correct
    for (unsigned int i = 0; i < path_parameters.num_segments; i++)
    {
        const Vector2d_t expected_position =
            shared_polynomial2d_getValueOrder3(path, i * delta_t);
        const float expected_orientation =
            shared_polynomial1d_getValueOrder3(orientation_profile, i * delta_t);

        EXPECT_FLOAT_EQ(expected_orientation,
                        trajectory.trajectory_elements[i].orientation);
        EXPECT_FLOAT_EQ(expected_position.x,
                        trajectory.trajectory_elements[i].position.x);
        EXPECT_FLOAT_EQ(expected_position.y,
                        trajectory.trajectory_elements[i].position.y);
    }
    EXPECT_FLOAT_EQ(
        trajectory.trajectory_elements[path_parameters.num_segments - 1].position.y, 1);
    EXPECT_FLOAT_EQ(
        trajectory.trajectory_elements[path_parameters.num_segments - 1].position.x, 1);
    EXPECT_FLOAT_EQ(
        trajectory.trajectory_elements[path_parameters.num_segments - 1].orientation, 1);
}

TEST_F(TrajectoryPlannerTest, test_robot_segment_size_generation_at_constant_t_intervals)
{
    const float total_path_length_linear  = sqrt(2.0);
    const float total_path_length_angular = 1;
    Polynomial2dOrder3_t path             = {
        .x = {.coefficients = {0, 0, 1, 0}},
        .y = {.coefficients = {0, 0, 1, 0}},

    };

    Polynomial1dOrder3_t orientation_profile = {.coefficients = {0, 0, 1, 0}};
    // Generate parameters for a trajectory
    FirmwareRobotPathParameters_t path_parameters;
    path_parameters.path    = path;
    path_parameters.t_start = 0;
    path_parameters.t_end   = 1;
    path_parameters.num_segments =
        TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS;  // For tests the number of segments is
    // arbitrary, so use the maximum possible
    path_parameters.max_allowable_linear_acceleration = 3;
    path_parameters.max_allowable_linear_speed        = 3;
    path_parameters.initial_linear_speed              = 0;
    path_parameters.final_linear_speed                = 0;

    path_parameters.orientation_profile                = orientation_profile;
    path_parameters.max_allowable_angular_acceleration = 9;
    path_parameters.max_allowable_angular_speed        = 6;


    PositionTrajectoryElement_t const_t_segments[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    TrajectorySegment_t segment_lengths[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    PositionTrajectory_t trajectory = {.trajectory_elements = const_t_segments,
                                       .path_parameters     = path_parameters};

    app_trajectory_planner_generateStatesAndReturnSegmentLengths(&trajectory,
                                                                 segment_lengths);

    const float delta_length = total_path_length_linear / path_parameters.num_segments;
    const float delta_length_orientation =
        total_path_length_angular / path_parameters.num_segments;
    // Check that all of the segment lengths are correct
    for (unsigned int i = 0; i < path_parameters.num_segments - 1; i++)
    {
        EXPECT_NEAR(segment_lengths[i].linear_segment_length, delta_length, 0.00001);
        EXPECT_NEAR(segment_lengths[i].angular_segment_length, delta_length_orientation,
                    0.00001);
    }
}

TEST_F(TrajectoryPlannerTest,
       test_robot_segment_size_generation_at_constant_t_intervals_low_segment_count)
{
    const float total_path_length_linear  = sqrt(2.0);
    const float total_path_length_angular = 1;
    Polynomial2dOrder3_t path             = {
        .x = {.coefficients = {0, 0, 1, 0}},
        .y = {.coefficients = {0, 0, 1, 0}},

    };

    Polynomial1dOrder3_t orientation_profile = {.coefficients = {0, 0, 1, 0}};
    // Generate parameters for a trajectory
    FirmwareRobotPathParameters_t path_parameters;
    path_parameters.path         = path;
    path_parameters.t_start      = 0;
    path_parameters.t_end        = 1;
    path_parameters.num_segments = 5;  // For tests the number of segments is
    // arbitrary, so use the maximum possible
    path_parameters.max_allowable_linear_acceleration = 3;
    path_parameters.max_allowable_linear_speed        = 3;
    path_parameters.initial_linear_speed              = 0;
    path_parameters.final_linear_speed                = 0;

    path_parameters.orientation_profile                = orientation_profile;
    path_parameters.max_allowable_angular_acceleration = 9;
    path_parameters.max_allowable_angular_speed        = 6;


    PositionTrajectoryElement_t const_t_segments[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    TrajectorySegment_t segment_lengths[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    PositionTrajectory_t trajectory = {.trajectory_elements = const_t_segments,
                                       .path_parameters     = path_parameters};

    app_trajectory_planner_generateStatesAndReturnSegmentLengths(&trajectory,
                                                                 segment_lengths);

    const float delta_length =
        total_path_length_linear / (path_parameters.num_segments - 1);
    const float delta_length_orientation =
        total_path_length_angular / (path_parameters.num_segments - 1);
    // Check that all of the segment lengths are correct
    for (unsigned int i = 0; i < path_parameters.num_segments - 1; i++)
    {
        EXPECT_NEAR(segment_lengths[i].linear_segment_length, delta_length, 0.00001);
        EXPECT_NEAR(segment_lengths[i].angular_segment_length, delta_length_orientation,
                    0.00001);
    }
}

TEST_F(TrajectoryPlannerTest,
       test_forward_continuity_path_curvature_max_speed_limited_constant_segment_length)
{
    float max_allowable_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    const unsigned int num_segments = 10;

    for (unsigned int i = 0; i < num_segments; i++)
    {
        max_allowable_speed_profile[i] = 1;
    }


    TrajectorySegment_t segment_lengths[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    segment_lengths[0].linear_segment_length = 1.0;
    segment_lengths[1].linear_segment_length = 1.0;
    segment_lengths[2].linear_segment_length = 1.0;
    segment_lengths[3].linear_segment_length = 1.0;
    segment_lengths[4].linear_segment_length = 1.0;
    segment_lengths[5].linear_segment_length = 1.0;
    segment_lengths[6].linear_segment_length = 1.0;
    segment_lengths[7].linear_segment_length = 1.0;
    segment_lengths[8].linear_segment_length = 1.0;

    FirmwareRobotPathParameters_t path_parameters;
    path_parameters.max_allowable_angular_acceleration = 8;
    path_parameters.max_allowable_angular_speed        = 3;
    path_parameters.max_allowable_linear_acceleration  = 3;
    path_parameters.max_allowable_linear_speed         = 3;
    path_parameters.num_segments                       = num_segments;

    float speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    PositionTrajectory_t trajectory;
    PositionTrajectoryElement_t trajectory_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    trajectory.trajectory_elements  = trajectory_elements;
    trajectory.linear_speed_profile = speed_profile;
    trajectory.path_parameters      = path_parameters;

    app_trajectory_planner_generateForwardsContinuousLinearSpeedProfile_2(
        &trajectory, max_allowable_speed_profile, segment_lengths);

    for (unsigned int i = 0; i < num_segments; i++)
    {
        EXPECT_TRUE(trajectory.trajectory_elements[i].linear_speed <= 1);
    }
}

TEST_F(TrajectoryPlannerTest, test_forwards_continuity_varying_segment_length)
{
    float max_allowable_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    const unsigned int num_segments = 5;

    for (unsigned int i = 0; i < num_segments; i++)
    {
        max_allowable_speed_profile[i] = 10;
    }

    TrajectorySegment_t segment_lengths[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    FirmwareRobotPathParameters_t path_parameters;
    path_parameters.max_allowable_angular_acceleration = 8;
    path_parameters.max_allowable_angular_speed        = 3;
    path_parameters.max_allowable_linear_acceleration  = 1;
    path_parameters.max_allowable_linear_speed         = 10;
    path_parameters.num_segments                       = num_segments;
    path_parameters.initial_linear_speed               = 1;
    path_parameters.final_linear_speed                 = 2;


    float speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    PositionTrajectory_t trajectory;
    PositionTrajectoryElement_t trajectory_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    trajectory.linear_speed_profile = speed_profile;
    trajectory.path_parameters      = path_parameters;
    trajectory.trajectory_elements  = trajectory_elements;

    segment_lengths[0].linear_segment_length = 1.0;
    segment_lengths[1].linear_segment_length = 2.0;
    segment_lengths[2].linear_segment_length = 3.0;
    segment_lengths[3].linear_segment_length = 4.0;

    TrajectoryPlannerGenerationStatus_t status =
        app_trajectory_planner_generateForwardsContinuousLinearSpeedProfile_2(
            &trajectory, max_allowable_speed_profile, segment_lengths);
    EXPECT_EQ(status, OK);

    EXPECT_NEAR(trajectory.trajectory_elements[0].linear_speed,
                path_parameters.initial_linear_speed, 0.0001);
    EXPECT_NEAR(trajectory.trajectory_elements[1].linear_speed, sqrt(3), 0.00001);
    EXPECT_NEAR(trajectory.trajectory_elements[2].linear_speed, sqrt(7), 0.00001);
    EXPECT_NEAR(trajectory.trajectory_elements[3].linear_speed, sqrt(13), 0.00001);
    EXPECT_NEAR(trajectory.trajectory_elements[4].linear_speed, 2, 0.00001);
}

TEST_F(TrajectoryPlannerTest, test_forwards_continuity_final_velocity_too_high)
{
    // Create a trajectory where the final velocity is much to high to reach given the
    // path length and acceleration
    float max_allowable_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    const unsigned int num_segments = 5;

    for (unsigned int i = 0; i < num_segments; i++)
    {
        max_allowable_speed_profile[i] = 10;
    }

    float velocity_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    const float arc_segment_length         = 1.0;
    const float max_allowable_acceleration = 0.1;
    const float max_allowable_speed        = 10;

    TrajectorySegment_t segment_lengths[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    segment_lengths[0].linear_segment_length = 1.0;
    segment_lengths[1].linear_segment_length = 2.0;
    segment_lengths[2].linear_segment_length = 3.0;
    segment_lengths[3].linear_segment_length = 4.0;

    FirmwareRobotPathParameters_t path_parameters;
    path_parameters.max_allowable_angular_acceleration = 8;
    path_parameters.max_allowable_angular_speed        = 3;
    path_parameters.max_allowable_linear_acceleration  = 1;
    path_parameters.max_allowable_linear_speed         = 10;
    path_parameters.num_segments                       = num_segments;
    path_parameters.initial_linear_speed               = 0;
    path_parameters.final_linear_speed                 = 200;


    float speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    PositionTrajectory_t trajectory;
    PositionTrajectoryElement_t trajectory_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    trajectory.linear_speed_profile = speed_profile;
    trajectory.path_parameters      = path_parameters;
    trajectory.trajectory_elements  = trajectory_elements;

    TrajectoryPlannerGenerationStatus_t status =
        app_trajectory_planner_generateForwardsContinuousLinearSpeedProfile_2(
            &trajectory, max_allowable_speed_profile, segment_lengths);
    EXPECT_EQ(status, FINAL_VELOCITY_TOO_HIGH);
}

TEST_F(TrajectoryPlannerTest, test_forwards_continuity_varying_segment_length_angular)
{
    float max_allowable_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    const unsigned int num_segments = 5;

    for (unsigned int i = 0; i < num_segments; i++)
    {
        max_allowable_speed_profile[i] = 10;
    }

    TrajectorySegment_t segment_lengths[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    segment_lengths[0].angular_segment_length = 1.0;
    segment_lengths[1].angular_segment_length = 2.0;
    segment_lengths[2].angular_segment_length = 3.0;
    segment_lengths[3].angular_segment_length = 4.0;

    FirmwareRobotPathParameters_t path_parameters;
    path_parameters.max_allowable_angular_acceleration = 1;
    path_parameters.max_allowable_angular_speed        = 10;
    path_parameters.max_allowable_linear_acceleration  = 1;
    path_parameters.max_allowable_linear_speed         = 10;
    path_parameters.num_segments                       = num_segments;
    path_parameters.initial_linear_speed               = 1;
    path_parameters.final_linear_speed                 = 2;



    float speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    PositionTrajectory_t trajectory;
    PositionTrajectoryElement_t trajectory_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    trajectory.angular_speed_profile = speed_profile;
    trajectory.path_parameters       = path_parameters;
    trajectory.trajectory_elements   = trajectory_elements;

    app_trajectory_planner_generateForwardsContinuousAngularSpeedProfile_2(
        &trajectory, segment_lengths);

    EXPECT_NEAR(trajectory.trajectory_elements[0].angular_speed, 0, 0.0001);
    EXPECT_NEAR(trajectory.trajectory_elements[1].angular_speed, sqrt(2), 0.00001);
    EXPECT_NEAR(trajectory.trajectory_elements[2].angular_speed, sqrt(6), 0.00001);
    EXPECT_NEAR(trajectory.trajectory_elements[3].angular_speed, sqrt(12), 0.00001);
    EXPECT_NEAR(trajectory.trajectory_elements[4].angular_speed, 0, 0.00001);
}

TEST_F(TrajectoryPlannerTest,
       test_forwards_continuity_constant_segment_speed_limited_angular)
{
    float max_allowable_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    const unsigned int num_segments = 5;

    for (unsigned int i = 0; i < num_segments; i++)
    {
        max_allowable_speed_profile[i] = 10;
    }

    TrajectorySegment_t segment_lengths[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    segment_lengths[0].angular_segment_length = 1.0;
    segment_lengths[1].angular_segment_length = 1.0;
    segment_lengths[2].angular_segment_length = 1.0;
    segment_lengths[3].angular_segment_length = 1.0;

    FirmwareRobotPathParameters_t path_parameters;
    path_parameters.max_allowable_angular_acceleration = 10;
    path_parameters.max_allowable_angular_speed        = 1;
    path_parameters.max_allowable_linear_acceleration  = 1;
    path_parameters.max_allowable_linear_speed         = 10;
    path_parameters.num_segments                       = num_segments;
    path_parameters.initial_linear_speed               = 1;
    path_parameters.final_linear_speed                 = 2;



    float speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    PositionTrajectory_t trajectory;
    PositionTrajectoryElement_t trajectory_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    trajectory.angular_speed_profile = speed_profile;
    trajectory.path_parameters       = path_parameters;
    trajectory.trajectory_elements   = trajectory_elements;

    app_trajectory_planner_generateForwardsContinuousAngularSpeedProfile_2(
        &trajectory, segment_lengths);

    EXPECT_NEAR(trajectory.trajectory_elements[0].angular_speed, 0, 0.0001);
    EXPECT_NEAR(trajectory.trajectory_elements[1].angular_speed, 1, 0.00001);
    EXPECT_NEAR(trajectory.trajectory_elements[2].angular_speed, 1, 0.00001);
    EXPECT_NEAR(trajectory.trajectory_elements[3].angular_speed, 1, 0.00001);
    EXPECT_NEAR(trajectory.trajectory_elements[4].angular_speed, 0, 0.00001);
}

TEST_F(TrajectoryPlannerTest,
       test_backwards_continuity_constant_segment_length_continuous)
{
    float max_allowable_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    const unsigned int num_segments = 4;


    TrajectorySegment_t segment_lengths[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    segment_lengths[0].linear_segment_length = 1.0;
    segment_lengths[1].linear_segment_length = 1.0;
    segment_lengths[2].linear_segment_length = 1.0;


    FirmwareRobotPathParameters_t path_parameters;
    path_parameters.max_allowable_angular_acceleration = 1;
    path_parameters.max_allowable_angular_speed        = 1;
    path_parameters.max_allowable_linear_acceleration  = 1;
    path_parameters.max_allowable_linear_speed         = 1;
    path_parameters.num_segments                       = num_segments;
    path_parameters.initial_linear_speed               = sqrt(7);
    path_parameters.final_linear_speed                 = 1;

    float speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    PositionTrajectory_t trajectory;
    PositionTrajectoryElement_t trajectory_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    trajectory.linear_speed_profile = speed_profile;
    trajectory.path_parameters      = path_parameters;
    trajectory.trajectory_elements  = trajectory_elements;

    trajectory.trajectory_elements[0].linear_speed = 4;
    trajectory.trajectory_elements[1].linear_speed = 3;
    trajectory.trajectory_elements[2].linear_speed = 2;
    trajectory.trajectory_elements[3].linear_speed = 1;


    TrajectoryPlannerGenerationStatus_t status =
        app_trajectory_planner_generateBackwardsContinuousSpeedProfile_2(&trajectory,
                                                                         segment_lengths);
    EXPECT_EQ(status, OK);

    EXPECT_NEAR(trajectory.trajectory_elements[0].linear_speed, sqrt(7), 0.0001);
    EXPECT_NEAR(trajectory.trajectory_elements[1].linear_speed, sqrt(5), 0.0001);
    EXPECT_NEAR(trajectory.trajectory_elements[2].linear_speed, sqrt(3), 0.0001);
    EXPECT_NEAR(trajectory.trajectory_elements[3].linear_speed, 1, 0.0001);
}

TEST_F(TrajectoryPlannerTest,
       test_backwards_continuity_variable_segment_length_continuous)
{
    float max_allowable_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    const unsigned int num_segments = 4;


    TrajectorySegment_t segment_lengths[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    segment_lengths[0].linear_segment_length = 1.0;
    segment_lengths[1].linear_segment_length = 2.0;
    segment_lengths[2].linear_segment_length = 3.0;


    FirmwareRobotPathParameters_t path_parameters;
    path_parameters.max_allowable_angular_acceleration = 1;
    path_parameters.max_allowable_angular_speed        = 1;
    path_parameters.max_allowable_linear_acceleration  = 1;
    path_parameters.max_allowable_linear_speed         = 1;
    path_parameters.num_segments                       = num_segments;
    path_parameters.initial_linear_speed               = sqrt(7);
    path_parameters.final_linear_speed                 = 1;

    float speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];


    PositionTrajectory_t trajectory;
    PositionTrajectoryElement_t trajectory_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    trajectory.linear_speed_profile = speed_profile;
    trajectory.path_parameters      = path_parameters;
    trajectory.trajectory_elements  = trajectory_elements;

    trajectory.trajectory_elements[0].linear_speed = 4;
    trajectory.trajectory_elements[1].linear_speed = 3;
    trajectory.trajectory_elements[2].linear_speed = 10;
    trajectory.trajectory_elements[3].linear_speed = 1;

    TrajectoryPlannerGenerationStatus_t status =
        app_trajectory_planner_generateBackwardsContinuousSpeedProfile_2(&trajectory,
                                                                         segment_lengths);
    EXPECT_EQ(status, OK);

    EXPECT_NEAR(trajectory.trajectory_elements[0].linear_speed, sqrt(11), 0.0001);
    EXPECT_NEAR(trajectory.trajectory_elements[1].linear_speed, 3, 0.0001);
    EXPECT_NEAR(trajectory.trajectory_elements[2].linear_speed, sqrt(7), 0.0001);
    EXPECT_NEAR(trajectory.trajectory_elements[3].linear_speed, 1, 0.0001);
}

TEST_F(TrajectoryPlannerTest,
       test_backwards_continuity_variable_segment_length_initial_velocity_too_high)
{
    // Create an 'initial velocity too high' state by requesting an initial velocity that
    // is much greater than physically possible given the path length and acceleration
    float max_allowable_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    const unsigned int num_segments = 4;


    TrajectorySegment_t segment_lengths[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    segment_lengths[0].linear_segment_length = 1.0;
    segment_lengths[1].linear_segment_length = 2.0;
    segment_lengths[2].linear_segment_length = 3.0;


    FirmwareRobotPathParameters_t path_parameters;
    path_parameters.max_allowable_angular_acceleration = 1;
    path_parameters.max_allowable_angular_speed        = 1;
    path_parameters.max_allowable_linear_acceleration  = 1;
    path_parameters.max_allowable_linear_speed         = 1;
    path_parameters.num_segments                       = num_segments;
    path_parameters.initial_linear_speed               = 20;
    path_parameters.final_linear_speed                 = 1;

    float speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    PositionTrajectory_t trajectory;
    PositionTrajectoryElement_t trajectory_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    trajectory.linear_speed_profile = speed_profile;
    trajectory.path_parameters      = path_parameters;
    trajectory.trajectory_elements  = trajectory_elements;

    trajectory.trajectory_elements[0].linear_speed = 20;
    trajectory.trajectory_elements[1].linear_speed = 3;
    trajectory.trajectory_elements[2].linear_speed = 10;
    trajectory.trajectory_elements[3].linear_speed = 1;

    TrajectoryPlannerGenerationStatus_t status =
        app_trajectory_planner_generateBackwardsContinuousSpeedProfile_2(&trajectory,
                                                                         segment_lengths);
    EXPECT_EQ(status, INITIAL_VELOCITY_TOO_HIGH);
}

TEST_F(TrajectoryPlannerTest,
       test_backwards_continuity_constant_segment_length_continuous_angular)
{
    float max_allowable_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    const unsigned int num_segments = 4;


    TrajectorySegment_t segment_lengths[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    segment_lengths[0].angular_segment_length = 1.0;
    segment_lengths[1].angular_segment_length = 1.0;
    segment_lengths[2].angular_segment_length = 1.0;


    FirmwareRobotPathParameters_t path_parameters;
    path_parameters.max_allowable_angular_acceleration = 1;
    path_parameters.max_allowable_angular_speed        = 1;
    path_parameters.max_allowable_linear_acceleration  = 1;
    path_parameters.max_allowable_linear_speed         = 1;
    path_parameters.num_segments                       = num_segments;
    path_parameters.initial_linear_speed               = sqrt(7);
    path_parameters.final_linear_speed                 = 1;

    float speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    speed_profile[0] = 0;
    speed_profile[1] = 3;
    speed_profile[2] = 2;
    speed_profile[3] = 0;

    PositionTrajectory_t trajectory;
    PositionTrajectoryElement_t trajectory_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    trajectory.angular_speed_profile = speed_profile;
    trajectory.path_parameters       = path_parameters;
    trajectory.trajectory_elements   = trajectory_elements;

    trajectory.trajectory_elements[0].angular_speed = 0;
    trajectory.trajectory_elements[1].angular_speed = 3;
    trajectory.trajectory_elements[2].angular_speed = 2;
    trajectory.trajectory_elements[3].angular_speed = 0;

    TrajectoryPlannerGenerationStatus_t status =
        app_trajectory_planner_generateBackwardsContinuousSpeedProfile_2(&trajectory,
                                                                         segment_lengths);
    EXPECT_EQ(status, OK);

    EXPECT_NEAR(trajectory.trajectory_elements[0].angular_speed, 0, 0.0001);
    EXPECT_NEAR(trajectory.trajectory_elements[1].angular_speed, sqrt(4), 0.0001);
    EXPECT_NEAR(trajectory.trajectory_elements[2].angular_speed, sqrt(2), 0.0001);
    EXPECT_NEAR(trajectory.trajectory_elements[3].angular_speed, 0, 0.0001);
}

TEST_F(TrajectoryPlannerTest,
       test_backwards_continuity_variable_segment_length_continuous_angular)
{
    float max_allowable_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    const unsigned int num_segments = 4;


    TrajectorySegment_t segment_lengths[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    segment_lengths[0].angular_segment_length = 1.0;
    segment_lengths[1].angular_segment_length = 2.0;
    segment_lengths[2].angular_segment_length = 3.0;


    FirmwareRobotPathParameters_t path_parameters;
    path_parameters.max_allowable_angular_acceleration = 1;
    path_parameters.max_allowable_angular_speed        = 1;
    path_parameters.max_allowable_linear_acceleration  = 1;
    path_parameters.max_allowable_linear_speed         = 1;
    path_parameters.num_segments                       = num_segments;
    path_parameters.initial_linear_speed               = sqrt(7);
    path_parameters.final_linear_speed                 = 1;

    float speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    PositionTrajectory_t trajectory;
    PositionTrajectoryElement_t trajectory_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    trajectory.angular_speed_profile = speed_profile;
    trajectory.path_parameters       = path_parameters;
    trajectory.trajectory_elements   = trajectory_elements;

    trajectory.trajectory_elements[0].angular_speed = 0;
    trajectory.trajectory_elements[1].angular_speed = 3;
    trajectory.trajectory_elements[2].angular_speed = 10;
    trajectory.trajectory_elements[3].angular_speed = 0;

    TrajectoryPlannerGenerationStatus_t status =
        app_trajectory_planner_generateBackwardsContinuousSpeedProfile_2(&trajectory,
                                                                         segment_lengths);
    EXPECT_EQ(status, OK);

    EXPECT_NEAR(trajectory.trajectory_elements[0].angular_speed, 0, 0.0001);
    EXPECT_NEAR(trajectory.trajectory_elements[1].angular_speed, 3, 0.0001);
    EXPECT_NEAR(trajectory.trajectory_elements[2].angular_speed, sqrt(6), 0.0001);
    EXPECT_NEAR(trajectory.trajectory_elements[3].angular_speed, 0, 0.0001);
}

TEST_F(TrajectoryPlannerTest, test_generate_maximum_speed_curve_straight_line)
{
    float max_allowable_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    FirmwareRobotPathParameters_t path_parameters;
    path_parameters.max_allowable_angular_acceleration = 1;
    path_parameters.max_allowable_angular_speed        = 1;
    path_parameters.max_allowable_linear_acceleration  = 1;
    path_parameters.max_allowable_linear_speed         = 1;
    path_parameters.num_segments         = TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS - 1;
    path_parameters.initial_linear_speed = 1;
    path_parameters.final_linear_speed   = 1;

    path_parameters.path = {.x = {0, 0, 1, 0}, .y = {0, 0, 1, 0}};

    app_trajectory_planner_getMaxAllowableSpeedProfile_2(
        path_parameters.path, path_parameters.num_segments, path_parameters.t_start,
        path_parameters.t_end, path_parameters.max_allowable_linear_acceleration,
        max_allowable_speed_profile);

    for (unsigned int i = 0; i < path_parameters.num_segments; i++)
    {
        EXPECT_GE(max_allowable_speed_profile[i], 1e+10);
    }
}

TEST_F(TrajectoryPlannerTest,
       test_generate_forwards_and_backwards_continuous_speed_profiles_angular_and_linear)

{
    FirmwareRobotPathParameters_t path_parameters;
    path_parameters.max_allowable_angular_acceleration = 1;
    path_parameters.max_allowable_angular_speed        = 1;
    path_parameters.max_allowable_linear_acceleration  = 1;
    path_parameters.max_allowable_linear_speed         = 1;
    path_parameters.num_segments =
        10;  // Use a small number of segments because this test was done manually
    path_parameters.initial_linear_speed = 0;
    path_parameters.final_linear_speed   = 0;
    path_parameters.t_start              = 1;
    path_parameters.t_end                = 2;
    path_parameters.path                 = {.x = {0, 0, 1, 0}, .y = {0, 0, 1, 0}};
    path_parameters.orientation_profile  = {.coefficients = {0, 0, 1, 0}};

    float linear_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    float angular_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    float max_allowable_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    TrajectorySegment_t segment_lengths[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    PositionTrajectoryElement_t trajectory_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    PositionTrajectory_t trajectory;
    trajectory.angular_speed_profile = angular_speed_profile;
    trajectory.linear_speed_profile  = linear_speed_profile;
    trajectory.trajectory_elements   = trajectory_elements;

    trajectory.path_parameters.path = {.x = {0, 0, 1, 0}, .y = {0, 0, 1, 0}};
    trajectory.path_parameters.orientation_profile = {.coefficients = {0, 0, 1, 0}};

    trajectory.path_parameters = path_parameters;

    // Get the segment states and the segment lengths
    app_trajectory_planner_generateStatesAndReturnSegmentLengths(&trajectory,
                                                                 segment_lengths);

    app_trajectory_planner_getMaxAllowableSpeedProfile_2(
        path_parameters.path, path_parameters.num_segments, path_parameters.t_start,
        path_parameters.t_end, path_parameters.max_allowable_linear_acceleration,
        max_allowable_speed_profile);

    TrajectoryPlannerGenerationStatus_t status =
        app_trajectory_planner_generateForwardsContinuousLinearSpeedProfile_2(
            &trajectory, max_allowable_speed_profile, segment_lengths);
    EXPECT_EQ(status, OK);

    app_trajectory_planner_generateForwardsContinuousAngularSpeedProfile_2(
        &trajectory, segment_lengths);

    // Check that the values of the acceleration period are correct
    EXPECT_NEAR(trajectory.trajectory_elements[0].linear_speed, 0, 0.0001);
    EXPECT_NEAR(trajectory.trajectory_elements[1].linear_speed, 0.5606, 0.0001);
    EXPECT_NEAR(trajectory.trajectory_elements[2].linear_speed, 0.7928, 0.0001);
    EXPECT_NEAR(trajectory.trajectory_elements[3].linear_speed, 0.9709, 0.0001);
    EXPECT_NEAR(trajectory.trajectory_elements[4].linear_speed, 1.0, 0.0001);

    EXPECT_NEAR(trajectory.trajectory_elements[0].angular_speed, 0, 0.0001);
    EXPECT_NEAR(trajectory.trajectory_elements[1].angular_speed, 0.4714, 0.0001);
    EXPECT_NEAR(trajectory.trajectory_elements[2].angular_speed, 0.6666, 0.0001);
    EXPECT_NEAR(trajectory.trajectory_elements[3].angular_speed, 0.8165, 0.0001);
    EXPECT_NEAR(trajectory.trajectory_elements[4].angular_speed, 0.9428, 0.0001);

    // Check that the path speed is limited by the maximum value set in the path
    // parameters
    for (unsigned int i = 5; i < trajectory.path_parameters.num_segments - 2; i++)
    {
        EXPECT_FLOAT_EQ(trajectory.trajectory_elements[i].linear_speed,
                        path_parameters.max_allowable_linear_speed);
        EXPECT_FLOAT_EQ(trajectory.trajectory_elements[i].angular_speed,
                        path_parameters.max_allowable_angular_speed);
    }

    status = app_trajectory_planner_generateBackwardsContinuousSpeedProfile_2(
        &trajectory, segment_lengths);
    EXPECT_EQ(status, OK);
    status = app_trajectory_planner_generateBackwardsContinuousSpeedProfile_2(
        &trajectory, segment_lengths);
    EXPECT_EQ(status, OK);

    // Check that the decceleration period is correct
    // Check that the values of the acceleration period are correct
    EXPECT_NEAR(
        trajectory.trajectory_elements[path_parameters.num_segments - 5].linear_speed,
        1 - 0, 0.0001);
    EXPECT_NEAR(
        trajectory.trajectory_elements[path_parameters.num_segments - 4].linear_speed,
        0.9709, 0.0001);
    EXPECT_NEAR(
        trajectory.trajectory_elements[path_parameters.num_segments - 3].linear_speed,
        0.7928, 0.0001);
    EXPECT_NEAR(
        trajectory.trajectory_elements[path_parameters.num_segments - 2].linear_speed,
        0.5606, 0.0001);
    EXPECT_NEAR(
        trajectory.trajectory_elements[path_parameters.num_segments - 1].linear_speed,
        path_parameters.final_linear_speed, 0.0001);


    EXPECT_NEAR(
        trajectory.trajectory_elements[path_parameters.num_segments - 5].angular_speed,
        0.9428, 0.0001);
    EXPECT_NEAR(
        trajectory.trajectory_elements[path_parameters.num_segments - 4].angular_speed,
        0.8165, 0.0001);
    EXPECT_NEAR(
        trajectory.trajectory_elements[path_parameters.num_segments - 3].angular_speed,
        0.6666, 0.0001);
    EXPECT_NEAR(
        trajectory.trajectory_elements[path_parameters.num_segments - 2].angular_speed,
        0.4714, 0.0001);
    EXPECT_NEAR(
        trajectory.trajectory_elements[path_parameters.num_segments - 1].angular_speed, 0,
        0.0001);
}

TEST_F(TrajectoryPlannerTest, test_generate_time_profile_no_angular_profile)
{
    // Create a scenario where the linear path parameters will limit the speed of the
    // trajectory because there is no orientation path to follow
    FirmwareRobotPathParameters_t path_parameters;
    path_parameters.max_allowable_angular_acceleration = 100;
    path_parameters.max_allowable_angular_speed        = 100;
    path_parameters.max_allowable_linear_acceleration  = 1;
    path_parameters.max_allowable_linear_speed         = 1;
    path_parameters.num_segments                       = 100;
    path_parameters.initial_linear_speed               = 1;
    path_parameters.final_linear_speed                 = 1;
    path_parameters.t_start                            = 1;
    path_parameters.t_end                              = 2;
    path_parameters.path                = {.x = {0, 0, 1, 0}, .y = {0, 0, 1, 0}};
    path_parameters.orientation_profile = {.coefficients = {0, 0, 0, 0}};

    float linear_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    float angular_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    float max_allowable_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    TrajectorySegment_t segment_lengths[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    PositionTrajectoryElement_t trajectory_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    PositionTrajectory_t trajectory;
    trajectory.angular_speed_profile = angular_speed_profile;
    trajectory.linear_speed_profile  = linear_speed_profile;
    trajectory.trajectory_elements   = trajectory_elements;

    trajectory.path_parameters.path = {.x = {0, 0, 1, 0}, .y = {0, 0, 1, 0}};
    trajectory.path_parameters.orientation_profile = {.coefficients = {0, 0, 1, 0}};

    trajectory.path_parameters = path_parameters;

    // Get the segment states and the segment lengths
    app_trajectory_planner_generateStatesAndReturnSegmentLengths(&trajectory,
                                                                 segment_lengths);

    app_trajectory_planner_getMaxAllowableSpeedProfile_2(
        path_parameters.path, path_parameters.num_segments, path_parameters.t_start,
        path_parameters.t_end, path_parameters.max_allowable_linear_acceleration,
        max_allowable_speed_profile);

    TrajectoryPlannerGenerationStatus_t status =
        app_trajectory_planner_generateForwardsContinuousLinearSpeedProfile_2(
            &trajectory, max_allowable_speed_profile, segment_lengths);
    EXPECT_EQ(status, OK);

    app_trajectory_planner_generateForwardsContinuousAngularSpeedProfile_2(
        &trajectory, segment_lengths);

    status = app_trajectory_planner_generateBackwardsContinuousSpeedProfile_2(
        &trajectory, segment_lengths);
    EXPECT_EQ(status, OK);
    status = app_trajectory_planner_generateBackwardsContinuousSpeedProfile_2(
        &trajectory, segment_lengths);
    EXPECT_EQ(status, OK);

    app_trajectory_planner_generatePositionTrajectoryTimeProfile_2(&trajectory,
                                                                   segment_lengths);

    EXPECT_FLOAT_EQ(trajectory.trajectory_elements[0].time, 0.0);
    for (unsigned int i = 1; i < path_parameters.num_segments; i++)
    {
        EXPECT_EQ(trajectory.trajectory_elements[i].time,
                  trajectory.trajectory_elements[i - 1].time +
                      segment_lengths[i - 1].linear_segment_length /
                          path_parameters.initial_linear_speed);
    }
    EXPECT_NEAR(trajectory.trajectory_elements[path_parameters.num_segments - 1].time,
                sqrt(2) / path_parameters.initial_linear_speed, 0.0001);
}

TEST_F(TrajectoryPlannerTest, test_generate_time_profile_linear_limiting)
{
    // Create a scenario where the linear path parameters will limit the speed of the
    // trajectory by using angular parameters that are much greater
    FirmwareRobotPathParameters_t path_parameters;
    path_parameters.max_allowable_angular_acceleration = 100;
    path_parameters.max_allowable_angular_speed        = 100;
    path_parameters.max_allowable_linear_acceleration  = 1;
    path_parameters.max_allowable_linear_speed         = 1;
    path_parameters.num_segments                       = 100;
    path_parameters.initial_linear_speed               = 1;
    path_parameters.final_linear_speed                 = 1;
    path_parameters.t_start                            = 1;
    path_parameters.t_end                              = 2;
    path_parameters.path                = {.x = {0, 0, 1, 0}, .y = {0, 0, 1, 0}};
    path_parameters.orientation_profile = {.coefficients = {0, 0, 1, 0}};

    float linear_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    float angular_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    float max_allowable_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    TrajectorySegment_t segment_lengths[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    PositionTrajectoryElement_t trajectory_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    PositionTrajectory_t trajectory;
    trajectory.angular_speed_profile = angular_speed_profile;
    trajectory.linear_speed_profile  = linear_speed_profile;
    trajectory.trajectory_elements   = trajectory_elements;

    trajectory.path_parameters = path_parameters;

    // Get the segment states and the segment lengths
    app_trajectory_planner_generateStatesAndReturnSegmentLengths(&trajectory,
                                                                 segment_lengths);

    app_trajectory_planner_getMaxAllowableSpeedProfile_2(
        path_parameters.path, path_parameters.num_segments, path_parameters.t_start,
        path_parameters.t_end, path_parameters.max_allowable_linear_acceleration,
        max_allowable_speed_profile);

    TrajectoryPlannerGenerationStatus_t status =
        app_trajectory_planner_generateForwardsContinuousLinearSpeedProfile_2(
            &trajectory, max_allowable_speed_profile, segment_lengths);
    EXPECT_EQ(status, OK);

    app_trajectory_planner_generateForwardsContinuousAngularSpeedProfile_2(
        &trajectory, segment_lengths);

    status = app_trajectory_planner_generateBackwardsContinuousSpeedProfile_2(
        &trajectory, segment_lengths);
    EXPECT_EQ(status, OK);
    status = app_trajectory_planner_generateBackwardsContinuousSpeedProfile_2(
        &trajectory, segment_lengths);
    EXPECT_EQ(status, OK);

    app_trajectory_planner_generatePositionTrajectoryTimeProfile_2(&trajectory,
                                                                   segment_lengths);

    EXPECT_FLOAT_EQ(trajectory.trajectory_elements[0].time, 0.0);
    for (unsigned int i = 1; i < path_parameters.num_segments; i++)
    {
        EXPECT_EQ(trajectory.trajectory_elements[i].time,
                  trajectory.trajectory_elements[i - 1].time +
                      segment_lengths[i - 1].linear_segment_length /
                          path_parameters.initial_linear_speed);
    }
    EXPECT_NEAR(trajectory.trajectory_elements[path_parameters.num_segments - 1].time,
                sqrt(2) / path_parameters.initial_linear_speed, 0.0001);
}

TEST_F(TrajectoryPlannerTest, test_generate_time_profile_no_linear_profile)
{
    // Create a scenario where the linear path parameters will limit the speed of the
    // trajectory by using angular parameters that are much greater
    FirmwareRobotPathParameters_t path_parameters;
    path_parameters.max_allowable_angular_acceleration = 100;
    path_parameters.max_allowable_angular_speed        = 1;
    path_parameters.max_allowable_linear_acceleration  = 1;
    path_parameters.max_allowable_linear_speed         = 1;
    path_parameters.num_segments                       = 100;
    path_parameters.initial_linear_speed               = 1;
    path_parameters.final_linear_speed                 = 1;
    path_parameters.t_start                            = 1;
    path_parameters.t_end                              = 10;
    path_parameters.path                = {.x = {0, 0, 0, 0}, .y = {0, 0, 0, 0}};
    path_parameters.orientation_profile = {.coefficients = {0, 0, 1, 0}};

    float linear_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    float angular_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    float max_allowable_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    TrajectorySegment_t segment_lengths[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    PositionTrajectoryElement_t trajectory_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    PositionTrajectory_t trajectory;
    trajectory.angular_speed_profile = angular_speed_profile;
    trajectory.linear_speed_profile  = linear_speed_profile;
    trajectory.trajectory_elements   = trajectory_elements;

    trajectory.path_parameters = path_parameters;

    // Get the segment states and the segment lengths
    app_trajectory_planner_generateStatesAndReturnSegmentLengths(&trajectory,
                                                                 segment_lengths);

    app_trajectory_planner_getMaxAllowableSpeedProfile_2(
        path_parameters.path, path_parameters.num_segments, path_parameters.t_start,
        path_parameters.t_end, path_parameters.max_allowable_linear_acceleration,
        max_allowable_speed_profile);

    TrajectoryPlannerGenerationStatus_t status =
        app_trajectory_planner_generateForwardsContinuousLinearSpeedProfile_2(
            &trajectory, max_allowable_speed_profile, segment_lengths);
    EXPECT_EQ(status, OK);

    app_trajectory_planner_generateForwardsContinuousAngularSpeedProfile_2(
        &trajectory, segment_lengths);

    status = app_trajectory_planner_generateBackwardsContinuousSpeedProfile_2(
        &trajectory, segment_lengths);
    EXPECT_EQ(status, OK);
    status = app_trajectory_planner_generateBackwardsContinuousSpeedProfile_2(
        &trajectory, segment_lengths);
    EXPECT_EQ(status, OK);

    app_trajectory_planner_generatePositionTrajectoryTimeProfile_2(&trajectory,
                                                                   segment_lengths);

    EXPECT_FLOAT_EQ(trajectory.trajectory_elements[0].time, 0.0);
    EXPECT_NEAR(trajectory.trajectory_elements[1].time, 0.1818, 0.0001);
    for (unsigned int i = 2; i < path_parameters.num_segments - 1; i++)
    {
        EXPECT_EQ(trajectory.trajectory_elements[i].time,
                  trajectory.trajectory_elements[i - 1].time +
                      segment_lengths[i - 1].angular_segment_length /
                          path_parameters.max_allowable_angular_speed);
    }
    const float radians_moved_in_acceleration_phase =
        trajectory.trajectory_elements[path_parameters.num_segments - 1].orientation -
        trajectory.trajectory_elements[path_parameters.num_segments - 2].orientation;

    EXPECT_NEAR(trajectory.trajectory_elements[path_parameters.num_segments - 1].time,
                9.1818, 0.0001);
}

TEST_F(TrajectoryPlannerTest, test_generate_time_profile_orientation_limiting)
{
    // Create a scenario where the linear path parameters will limit the speed of the
    // trajectory by using angular parameters that are much greater
    FirmwareRobotPathParameters_t path_parameters;
    path_parameters.max_allowable_angular_acceleration = 100;
    path_parameters.max_allowable_angular_speed        = 1;
    path_parameters.max_allowable_linear_acceleration  = 1;
    path_parameters.max_allowable_linear_speed         = 1;
    path_parameters.num_segments                       = 100;
    path_parameters.initial_linear_speed               = 1;
    path_parameters.final_linear_speed                 = 1;
    path_parameters.t_start                            = 1;
    path_parameters.t_end                              = 10;
    path_parameters.path                = {.x = {0, 0, 0, 0}, .y = {0, 0, 0, 0}};
    path_parameters.orientation_profile = {.coefficients = {0, 0, 1, 0}};

    float linear_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    float angular_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    float max_allowable_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    TrajectorySegment_t segment_lengths[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    PositionTrajectoryElement_t trajectory_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    PositionTrajectory_t trajectory;
    trajectory.angular_speed_profile = angular_speed_profile;
    trajectory.linear_speed_profile  = linear_speed_profile;
    trajectory.trajectory_elements   = trajectory_elements;

    trajectory.path_parameters = path_parameters;

    // Get the segment states and the segment lengths
    app_trajectory_planner_generateStatesAndReturnSegmentLengths(&trajectory,
                                                                 segment_lengths);

    app_trajectory_planner_getMaxAllowableSpeedProfile_2(
        path_parameters.path, path_parameters.num_segments, path_parameters.t_start,
        path_parameters.t_end, path_parameters.max_allowable_linear_acceleration,
        max_allowable_speed_profile);

    TrajectoryPlannerGenerationStatus_t status =
        app_trajectory_planner_generateForwardsContinuousLinearSpeedProfile_2(
            &trajectory, max_allowable_speed_profile, segment_lengths);
    EXPECT_EQ(status, OK);

    app_trajectory_planner_generateForwardsContinuousAngularSpeedProfile_2(
        &trajectory, segment_lengths);

    status = app_trajectory_planner_generateBackwardsContinuousSpeedProfile_2(
        &trajectory, segment_lengths);
    EXPECT_EQ(status, OK);
    status = app_trajectory_planner_generateBackwardsContinuousSpeedProfile_2(
        &trajectory, segment_lengths);
    EXPECT_EQ(status, OK);

    app_trajectory_planner_generatePositionTrajectoryTimeProfile_2(&trajectory,
                                                                   segment_lengths);

    EXPECT_FLOAT_EQ(trajectory.trajectory_elements[0].time, 0.0);
    EXPECT_NEAR(trajectory.trajectory_elements[1].time, 0.1818, 0.0001);
    for (unsigned int i = 2; i < path_parameters.num_segments - 1; i++)
    {
        EXPECT_EQ(trajectory.trajectory_elements[i].time,
                  trajectory.trajectory_elements[i - 1].time +
                      segment_lengths[i - 1].angular_segment_length /
                          path_parameters.max_allowable_angular_speed);
    }
    const float radians_moved_in_acceleration_phase =
        trajectory.trajectory_elements[path_parameters.num_segments - 1].orientation -
        trajectory.trajectory_elements[path_parameters.num_segments - 2].orientation;

    EXPECT_NEAR(trajectory.trajectory_elements[path_parameters.num_segments - 1].time,
                9.1818, 0.0001);
}

TEST_F(TrajectoryPlannerTest, test_rebalance_trajectory_segment_linear)
{
    // Create a scenario where the linear path parameters will limit the speed of the
    // trajectory by using angular parameters that are much greater
    FirmwareRobotPathParameters_t path_parameters;
    path_parameters.max_allowable_angular_acceleration = 1;
    path_parameters.max_allowable_angular_speed        = 1;
    path_parameters.max_allowable_linear_acceleration  = 1;
    path_parameters.max_allowable_linear_speed         = 1;
    path_parameters.num_segments                       = 100;
    path_parameters.initial_linear_speed               = 1;
    path_parameters.final_linear_speed                 = 1;
    path_parameters.t_start                            = 1;
    path_parameters.t_end                              = 10;
    path_parameters.path                = {.x = {0, 0, 1, 0}, .y = {0, 0, 1, 0}};
    path_parameters.orientation_profile = {.coefficients = {0, 0, 1, 0}};

    float linear_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    float angular_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    float max_allowable_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    TrajectorySegment_t segment_lengths[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    PositionTrajectoryElement_t trajectory_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    PositionTrajectory_t trajectory;
    trajectory.angular_speed_profile = angular_speed_profile;
    trajectory.linear_speed_profile  = linear_speed_profile;
    trajectory.trajectory_elements   = trajectory_elements;

    trajectory.path_parameters = path_parameters;

    // Create an artificial trajectory here for testing
    trajectory.trajectory_elements[2].linear_speed = 1;
    trajectory.trajectory_elements[3].linear_speed = 4;
    float desired_time                             = 0.01;  // seconds

    segment_lengths[2].linear_segment_length = 1.0;  // meters

    app_trajectory_planner_rebalanceAngularAndLinearTrajectorySegmentsForEquivalentDuration(
        &trajectory, segment_lengths, desired_time, 3, false);
    EXPECT_FLOAT_EQ(trajectory.trajectory_elements[3].linear_speed, 201.0);

    // Create an artificial trajectory here for testing
    trajectory.trajectory_elements[2].linear_speed = 1;
    trajectory.trajectory_elements[3].linear_speed = 4;
    desired_time                                   = 0.01;  // seconds

    segment_lengths[2].linear_segment_length = 0.01;  // meters

    app_trajectory_planner_rebalanceAngularAndLinearTrajectorySegmentsForEquivalentDuration(
        &trajectory, segment_lengths, desired_time, 3, false);

    EXPECT_FLOAT_EQ(trajectory.trajectory_elements[3].linear_speed, 3.0);
}

TEST_F(TrajectoryPlannerTest, test_rebalance_trajectory_segment_angular)
{
    // Create a scenario where the linear path parameters will limit the speed of the
    // trajectory by using angular parameters that are much greater
    FirmwareRobotPathParameters_t path_parameters;
    path_parameters.max_allowable_angular_acceleration = 1;
    path_parameters.max_allowable_angular_speed        = 1;
    path_parameters.max_allowable_linear_acceleration  = 1;
    path_parameters.max_allowable_linear_speed         = 1;
    path_parameters.num_segments                       = 100;
    path_parameters.initial_linear_speed               = 1;
    path_parameters.final_linear_speed                 = 1;
    path_parameters.t_start                            = 1;
    path_parameters.t_end                              = 10;
    path_parameters.path                = {.x = {0, 0, 1, 0}, .y = {0, 0, 1, 0}};
    path_parameters.orientation_profile = {.coefficients = {0, 0, 1, 0}};

    float linear_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    float angular_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    float max_allowable_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    TrajectorySegment_t segment_lengths[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    PositionTrajectoryElement_t trajectory_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    PositionTrajectory_t trajectory;
    trajectory.angular_speed_profile = angular_speed_profile;
    trajectory.linear_speed_profile  = linear_speed_profile;
    trajectory.trajectory_elements   = trajectory_elements;

    trajectory.path_parameters = path_parameters;

    // Create an artificial trajectory here for testing
    trajectory.trajectory_elements[2].angular_speed = 1;
    trajectory.trajectory_elements[3].angular_speed = 4;
    float desired_time                              = 0.01;  // seconds

    segment_lengths[2].angular_segment_length = 1.0;  // meters

    app_trajectory_planner_rebalanceAngularAndLinearTrajectorySegmentsForEquivalentDuration(
        &trajectory, segment_lengths, desired_time, 3, true);
    EXPECT_FLOAT_EQ(trajectory.trajectory_elements[3].angular_speed, 201.0);

    // Create an artificial trajectory here for testing
    trajectory.trajectory_elements[2].angular_speed = 1;
    trajectory.trajectory_elements[3].angular_speed = 4;
    desired_time                                    = 0.01;  // seconds

    segment_lengths[2].angular_segment_length = 0.01;  // meters

    app_trajectory_planner_rebalanceAngularAndLinearTrajectorySegmentsForEquivalentDuration(
        &trajectory, segment_lengths, desired_time, 3, true);

    EXPECT_FLOAT_EQ(trajectory.trajectory_elements[3].angular_speed, 3.0);
}

TEST_F(TrajectoryPlannerTest, test_generate_velocity_trajectory_zeros)
{
    // Create a scenario where the linear path parameters will limit the speed of the
    // trajectory by using angular parameters that are much greater
    FirmwareRobotPathParameters_t path_parameters;
    path_parameters.max_allowable_angular_acceleration = 1;
    path_parameters.max_allowable_angular_speed        = 1;
    path_parameters.max_allowable_linear_acceleration  = 1;
    path_parameters.max_allowable_linear_speed         = 1;
    path_parameters.num_segments                       = 100;
    path_parameters.initial_linear_speed               = 0;
    path_parameters.final_linear_speed                 = 0;
    path_parameters.t_start                            = 1;
    path_parameters.t_end                              = 10;
    path_parameters.path                = {.x = {0, 0, 0, 0}, .y = {0, 0, 0, 0}};
    path_parameters.orientation_profile = {.coefficients = {0, 0, 0, 0}};

    float max_allowable_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    TrajectorySegment_t segment_lengths[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];


    PositionTrajectory_t position_trajectory;
    PositionTrajectoryElement_t
        position_trajectory_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    VelocityTrajectory_t velocity_trajectory;
    VelocityTrajectoryElement_t
        velocity_trajectory_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    position_trajectory.trajectory_elements = position_trajectory_elements;
    velocity_trajectory.trajectory_elements = velocity_trajectory_elements;

    position_trajectory.path_parameters = path_parameters;
    velocity_trajectory.path_parameters = path_parameters;

    app_trajectory_planner_getMaxAllowableSpeedProfile_2(
        position_trajectory.path_parameters.path,
        position_trajectory.path_parameters.num_segments,
        position_trajectory.path_parameters.t_start,
        position_trajectory.path_parameters.t_end,
        position_trajectory.path_parameters.max_allowable_linear_acceleration,
        max_allowable_speed_profile);
    app_trajectory_planner_generateStatesAndReturnSegmentLengths(&position_trajectory,
                                                                 segment_lengths);

    app_trajectory_planner_generateForwardsContinuousLinearSpeedProfile_2(
        &position_trajectory, max_allowable_speed_profile, segment_lengths);
    app_trajectory_planner_generateForwardsContinuousAngularSpeedProfile_2(
        &position_trajectory, segment_lengths);

    app_trajectory_planner_generateBackwardsContinuousSpeedProfile_2(&position_trajectory,
                                                                     segment_lengths);
    app_trajectory_planner_generatePositionTrajectoryTimeProfile_2(&position_trajectory,
                                                                   segment_lengths);

    app_trajectory_planner_generateVelocityTrajectory_2(&position_trajectory,
                                                        &velocity_trajectory);

    for (unsigned int i = 0; i < velocity_trajectory.path_parameters.num_segments; i++)
    {
        EXPECT_FLOAT_EQ(velocity_trajectory.trajectory_elements[i].angular_velocity, 0);
        EXPECT_FLOAT_EQ(velocity_trajectory.trajectory_elements[i].linear_velocity.x, 0);
        EXPECT_FLOAT_EQ(velocity_trajectory.trajectory_elements[i].linear_velocity.y, 0);
        EXPECT_FLOAT_EQ(velocity_trajectory.trajectory_elements[i].time, 0);
    }
}

TEST_F(TrajectoryPlannerTest,
       test_generate_velocity_trajectory_linear_path_no_orientation)
{
    // Create a scenario where the linear path parameters will limit the speed of the
    // trajectory by using angular parameters that are much greater
    FirmwareRobotPathParameters_t path_parameters;
    path_parameters.max_allowable_angular_acceleration = 1;
    path_parameters.max_allowable_angular_speed        = 1;
    path_parameters.max_allowable_linear_acceleration  = 1;
    path_parameters.max_allowable_linear_speed         = 1;
    path_parameters.num_segments                       = 100;
    path_parameters.initial_linear_speed               = 1;
    path_parameters.final_linear_speed                 = 1;
    path_parameters.t_start                            = 1;
    path_parameters.t_end                              = 10;
    path_parameters.path                = {.x = {0, 0, 1, 0}, .y = {0, 0, 1, 0}};
    path_parameters.orientation_profile = {.coefficients = {0, 0, 0, 0}};

    float max_allowable_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    TrajectorySegment_t segment_lengths[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];


    PositionTrajectory_t position_trajectory;
    PositionTrajectoryElement_t
        position_trajectory_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    VelocityTrajectory_t velocity_trajectory;
    VelocityTrajectoryElement_t
        velocity_trajectory_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    position_trajectory.trajectory_elements = position_trajectory_elements;
    velocity_trajectory.trajectory_elements = velocity_trajectory_elements;

    position_trajectory.path_parameters = path_parameters;
    velocity_trajectory.path_parameters = path_parameters;

    app_trajectory_planner_getMaxAllowableSpeedProfile_2(
        position_trajectory.path_parameters.path,
        position_trajectory.path_parameters.num_segments,
        position_trajectory.path_parameters.t_start,
        position_trajectory.path_parameters.t_end,
        position_trajectory.path_parameters.max_allowable_linear_acceleration,
        max_allowable_speed_profile);
    app_trajectory_planner_generateStatesAndReturnSegmentLengths(&position_trajectory,
                                                                 segment_lengths);

    app_trajectory_planner_generateForwardsContinuousLinearSpeedProfile_2(
        &position_trajectory, max_allowable_speed_profile, segment_lengths);
    app_trajectory_planner_generateForwardsContinuousAngularSpeedProfile_2(
        &position_trajectory, segment_lengths);

    app_trajectory_planner_generateBackwardsContinuousSpeedProfile_2(&position_trajectory,
                                                                     segment_lengths);
    app_trajectory_planner_generatePositionTrajectoryTimeProfile_2(&position_trajectory,
                                                                   segment_lengths);

    app_trajectory_planner_generateVelocityTrajectory_2(&position_trajectory,
                                                        &velocity_trajectory);

    std::vector<Vector> directions =
        getDirectionVectorsFromPositionTrajectory(&position_trajectory);
    const float segment_length = 9 * sqrt(2) / (path_parameters.num_segments - 1);

    for (unsigned int i = 0; i < velocity_trajectory.path_parameters.num_segments; i++)
    {
        EXPECT_FLOAT_EQ(velocity_trajectory.trajectory_elements[i].angular_velocity, 0);
        EXPECT_FLOAT_EQ(velocity_trajectory.trajectory_elements[i].linear_velocity.x,
                        directions[i].x());
        EXPECT_FLOAT_EQ(velocity_trajectory.trajectory_elements[i].linear_velocity.y,
                        directions[i].y());
        EXPECT_NEAR(velocity_trajectory.trajectory_elements[i].time, i * segment_length,
                    0.00001);
    }
}

TEST_F(TrajectoryPlannerTest,
       test_generate_velocity_trajectory_no_path_linear_orientation)
{
    // Create a scenario where the linear path parameters will limit the speed of the
    // trajectory by using angular parameters that are much greater
    FirmwareRobotPathParameters_t path_parameters;
    path_parameters.max_allowable_angular_acceleration =
        100;  // Make linear acceleration large to reach steady state fast
    path_parameters.max_allowable_angular_speed       = 1;
    path_parameters.max_allowable_linear_acceleration = 1;
    path_parameters.max_allowable_linear_speed        = 1;
    path_parameters.num_segments                      = 100;
    path_parameters.initial_linear_speed              = 0;
    path_parameters.final_linear_speed                = 0;
    path_parameters.t_start                           = 1;
    path_parameters.t_end                             = 10;
    path_parameters.path                = {.x = {0, 0, 0, 0}, .y = {0, 0, 0, 0}};
    path_parameters.orientation_profile = {.coefficients = {0, 0, 1, 0}};

    float max_allowable_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    TrajectorySegment_t segment_lengths[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];


    PositionTrajectory_t position_trajectory;
    PositionTrajectoryElement_t
        position_trajectory_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    VelocityTrajectory_t velocity_trajectory;
    VelocityTrajectoryElement_t
        velocity_trajectory_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    position_trajectory.trajectory_elements = position_trajectory_elements;
    velocity_trajectory.trajectory_elements = velocity_trajectory_elements;

    position_trajectory.path_parameters = path_parameters;
    velocity_trajectory.path_parameters = path_parameters;

    app_trajectory_planner_getMaxAllowableSpeedProfile_2(
        position_trajectory.path_parameters.path,
        position_trajectory.path_parameters.num_segments,
        position_trajectory.path_parameters.t_start,
        position_trajectory.path_parameters.t_end,
        position_trajectory.path_parameters.max_allowable_linear_acceleration,
        max_allowable_speed_profile);
    app_trajectory_planner_generateStatesAndReturnSegmentLengths(&position_trajectory,
                                                                 segment_lengths);

    app_trajectory_planner_generateForwardsContinuousLinearSpeedProfile_2(
        &position_trajectory, max_allowable_speed_profile, segment_lengths);
    app_trajectory_planner_generateForwardsContinuousAngularSpeedProfile_2(
        &position_trajectory, segment_lengths);

    app_trajectory_planner_generateBackwardsContinuousSpeedProfile_2(&position_trajectory,
                                                                     segment_lengths);
    app_trajectory_planner_generatePositionTrajectoryTimeProfile_2(&position_trajectory,
                                                                   segment_lengths);

    app_trajectory_planner_generateVelocityTrajectory_2(&position_trajectory,
                                                        &velocity_trajectory);

    std::vector<Vector> directions =
        getDirectionVectorsFromPositionTrajectory(&position_trajectory);
    const float segment_length           = 9.0 / (path_parameters.num_segments - 1);
    const float initial_segment_duration = (2 * segment_length) / (1);

    EXPECT_FLOAT_EQ(velocity_trajectory.trajectory_elements[0].angular_velocity, 0);
    for (unsigned int i = 1; i < velocity_trajectory.path_parameters.num_segments - 1;
         i++)
    {
        EXPECT_FLOAT_EQ(velocity_trajectory.trajectory_elements[i].angular_velocity, 1);
        EXPECT_FLOAT_EQ(velocity_trajectory.trajectory_elements[i].linear_velocity.x, 0);
        EXPECT_FLOAT_EQ(velocity_trajectory.trajectory_elements[i].linear_velocity.y, 0);
        EXPECT_NEAR(velocity_trajectory.trajectory_elements[i].time,
                    initial_segment_duration + (i - 1) * segment_length, 0.00001);
    }
    EXPECT_FLOAT_EQ(
        velocity_trajectory.trajectory_elements[path_parameters.num_segments - 1]
            .angular_velocity,
        0);
}

TEST_F(TrajectoryPlannerTest,
       test_generate_velocity_trajectory_polynomial_path_polynomial_orientation)
{
    // Create a scenario where the linear path parameters will limit the speed of the
    // trajectory by using angular parameters that are much greater
    FirmwareRobotPathParameters_t path_parameters;
    path_parameters.max_allowable_angular_acceleration =
        100;  // Make linear acceleration large to reach steady state fast
    path_parameters.max_allowable_angular_speed       = 1.2;
    path_parameters.max_allowable_linear_acceleration = 1;
    path_parameters.max_allowable_linear_speed        = 1.1;
    path_parameters.num_segments                      = 100;
    path_parameters.initial_linear_speed              = 0;
    path_parameters.final_linear_speed                = 0;
    path_parameters.t_start                           = 0;
    path_parameters.t_end                             = 10;
    path_parameters.path                = {.x = {1, 0, 2, 0}, .y = {0, 0, 2, 0}};
    path_parameters.orientation_profile = {.coefficients = {1, 0, 1, 0}};

    float max_allowable_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    TrajectorySegment_t segment_lengths[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];


    PositionTrajectory_t position_trajectory;
    PositionTrajectoryElement_t
        position_trajectory_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    VelocityTrajectory_t velocity_trajectory;
    VelocityTrajectoryElement_t
        velocity_trajectory_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    position_trajectory.trajectory_elements = position_trajectory_elements;
    velocity_trajectory.trajectory_elements = velocity_trajectory_elements;

    position_trajectory.path_parameters = path_parameters;
    velocity_trajectory.path_parameters = path_parameters;

    app_trajectory_planner_getMaxAllowableSpeedProfile_2(
        position_trajectory.path_parameters.path,
        position_trajectory.path_parameters.num_segments,
        position_trajectory.path_parameters.t_start,
        position_trajectory.path_parameters.t_end,
        position_trajectory.path_parameters.max_allowable_linear_acceleration,
        max_allowable_speed_profile);
    app_trajectory_planner_generateStatesAndReturnSegmentLengths(&position_trajectory,
                                                                 segment_lengths);

    app_trajectory_planner_generateForwardsContinuousLinearSpeedProfile_2(
        &position_trajectory, max_allowable_speed_profile, segment_lengths);
    app_trajectory_planner_generateForwardsContinuousAngularSpeedProfile_2(
        &position_trajectory, segment_lengths);

    app_trajectory_planner_generateBackwardsContinuousSpeedProfile_2(&position_trajectory,
                                                                     segment_lengths);
    app_trajectory_planner_generatePositionTrajectoryTimeProfile_2(&position_trajectory,
                                                                   segment_lengths);

    app_trajectory_planner_generateVelocityTrajectory_2(&position_trajectory,
                                                        &velocity_trajectory);

    std::vector<Vector> directions =
        getDirectionVectorsFromPositionTrajectory(&position_trajectory);
    const float segment_length           = sqrt(8.0) / (path_parameters.num_segments - 1);
    const float initial_segment_duration = (2 * segment_length) / (1);

    for (unsigned int i = 0; i < velocity_trajectory.path_parameters.num_segments - 1;
         i++)
    {
        EXPECT_FLOAT_EQ(velocity_trajectory.trajectory_elements[i].angular_velocity,
                        position_trajectory.trajectory_elements[i].angular_speed);
        EXPECT_FLOAT_EQ(
            velocity_trajectory.trajectory_elements[i].linear_velocity.x,
            position_trajectory.trajectory_elements[i].linear_speed * directions[i].x());
        EXPECT_FLOAT_EQ(
            velocity_trajectory.trajectory_elements[i].linear_velocity.y,
            position_trajectory.trajectory_elements[i].linear_speed * directions[i].y());
        EXPECT_NEAR(velocity_trajectory.trajectory_elements[i].time,
                    position_trajectory.trajectory_elements[i].time, 0.00001);
    }
    EXPECT_FLOAT_EQ(
        velocity_trajectory.trajectory_elements[path_parameters.num_segments - 1]
            .angular_velocity,
        0);
}
