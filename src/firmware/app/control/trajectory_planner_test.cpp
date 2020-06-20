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
        PositionTrajectory_t* position_trajectory,
        FirmwareRobotPathParameters_t path_parameters)
    {
        std::vector<Vector> unit_vectors;

        for (unsigned int i = 0; i < path_parameters.num_elements - 1; i++)
        {
            const float delta_x = position_trajectory->x_position[i + 1] -
                                  position_trajectory->x_position[i];
            const float delta_y = position_trajectory->y_position[i + 1] -
                                  position_trajectory->y_position[i];

            // Copy data into the Vector
            Vector direction = Vector(delta_x, delta_y).normalize(1);
            unit_vectors.push_back(direction);
        }

        // Assume that the velocity of the final segment is in the direction of the
        // previous
        const float delta_x =
            position_trajectory->x_position[path_parameters.num_elements - 1] -
            position_trajectory->x_position[path_parameters.num_elements - 2];
        const float delta_y =
            position_trajectory->y_position[path_parameters.num_elements - 1] -
            position_trajectory->y_position[path_parameters.num_elements - 2];

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

    FirmwareRobotPathParameters_t path_parameters{
        .path                               = path,
        .orientation_profile                = orientation_profile,
        .t_start                            = 0,
        .t_end                              = 1,
        .num_elements                       = TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS,
        .max_allowable_linear_acceleration  = 3,
        .max_allowable_linear_speed         = 3,
        .max_allowable_angular_acceleration = 0,
        .max_allowable_angular_speed        = 0,
        .initial_linear_speed               = 0,
        .final_linear_speed                 = 0,

    };

    PositionTrajectory_t trajectory;

    TrajectoryPlannerGenerationStatus_t status =
        app_trajectory_planner_generateConstantParameterizationPositionTrajectory(
            path_parameters, &trajectory);
    EXPECT_EQ(OK, status);

    EXPECT_NEAR(
        trajectory.x_position[path_parameters.num_elements - 1],
        shared_polynomial2d_getValueOrder3(path_parameters.path, path_parameters.t_end).x,
        0.01);
    EXPECT_NEAR(
        trajectory.y_position[path_parameters.num_elements - 1],
        shared_polynomial2d_getValueOrder3(path_parameters.path, path_parameters.t_end).y,
        0.01);

    EXPECT_NEAR(
        trajectory.x_position[0],
        shared_polynomial2d_getValueOrder3(path_parameters.path, path_parameters.t_start)
            .x,
        0.01);
    EXPECT_NEAR(
        trajectory.y_position[0],
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

    FirmwareRobotPathParameters_t path_parameters{
        .path                               = path,
        .orientation_profile                = orientation_profile,
        .t_start                            = 0,
        .t_end                              = 1,
        .num_elements                       = TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS - 1,
        .max_allowable_linear_acceleration  = 3,
        .max_allowable_linear_speed         = 3,
        .max_allowable_angular_acceleration = 0,
        .max_allowable_angular_speed        = 0,
        .initial_linear_speed               = 0,
        .final_linear_speed                 = 0,
    };

    PositionTrajectory_t trajectory;

    TrajectoryPlannerGenerationStatus_t status =
        app_trajectory_planner_generateConstantParameterizationPositionTrajectory(
            path_parameters, &trajectory);
    EXPECT_EQ(OK, status);
    float segment_length_sum = 0;

    // Test by checking the length between points
    // and the length of the total path
    for (uint i = 0; i < path_parameters.num_elements - 1; i++)
    {
        float length = static_cast<float>(
            sqrt(pow(trajectory.x_position[i + 1] - trajectory.x_position[i], 2) +
                 pow(trajectory.y_position[i + 1] - trajectory.y_position[i], 2)));
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

    FirmwareRobotPathParameters_t path_parameters{
        .path                               = path,
        .orientation_profile                = orientation_profile,
        .t_start                            = 0,
        .t_end                              = 1,
        .num_elements                       = TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS - 1,
        .max_allowable_linear_acceleration  = 3,
        .max_allowable_linear_speed         = 3,
        .max_allowable_angular_acceleration = 0,
        .max_allowable_angular_speed        = 0,
        .initial_linear_speed               = 0,
        .final_linear_speed                 = 0,

    };

    PositionTrajectory_t trajectory;

    TrajectoryPlannerGenerationStatus_t status =
        app_trajectory_planner_generateConstantParameterizationPositionTrajectory(
            path_parameters, &trajectory);
    EXPECT_EQ(OK, status);

    EXPECT_NEAR(trajectory.x_position[path_parameters.num_elements - 1],
                shared_polynomial2d_getValueOrder3(path, path_parameters.t_end).x, 0.01);
    EXPECT_NEAR(trajectory.y_position[path_parameters.num_elements - 1],
                shared_polynomial2d_getValueOrder3(path, path_parameters.t_end).y, 0.01);

    EXPECT_NEAR(trajectory.x_position[0],
                shared_polynomial2d_getValueOrder3(path, path_parameters.t_start).x,
                0.01);
    EXPECT_NEAR(trajectory.y_position[0],
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

    FirmwareRobotPathParameters_t path_parameters{
        .path                               = path,
        .orientation_profile                = orientation_profile,
        .t_start                            = 0,
        .t_end                              = 1,
        .num_elements                       = TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS - 1,
        .max_allowable_linear_acceleration  = 3,
        .max_allowable_linear_speed         = 3,
        .max_allowable_angular_acceleration = 0,
        .max_allowable_angular_speed        = 0,
        .initial_linear_speed               = 0,
        .final_linear_speed                 = 0,

    };

    PositionTrajectory_t trajectory;

    TrajectoryPlannerGenerationStatus_t status =
        app_trajectory_planner_generateConstantParameterizationPositionTrajectory(
            path_parameters, &trajectory);
    EXPECT_EQ(OK, status);

    float forwards_linear_speed[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    float forwards_angular_speed[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    float forwards_orientation[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    float forwards_position_x[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    float forwards_position_y[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    float forwards_time[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    // Copy the 'forwards' velocity profile (as if t1 < t2)
    for (unsigned int i = 0; i < path_parameters.num_elements; i++)
    {
        forwards_linear_speed[i]  = trajectory.linear_speed[i];
        forwards_angular_speed[i] = trajectory.angular_speed[i];
        forwards_orientation[i]   = trajectory.orientation[i];
        forwards_position_x[i]    = trajectory.x_position[i];
        forwards_position_y[i]    = trajectory.y_position[i];
        forwards_time[i]          = trajectory.time_profile[i];
    }

    // Reverse the parameterization
    path_parameters.t_start = 1;
    path_parameters.t_end   = 0;

    status = app_trajectory_planner_generateConstantParameterizationPositionTrajectory(
        path_parameters, &trajectory);
    EXPECT_EQ(OK, status);

    for (unsigned int i = 0; i < path_parameters.num_elements; i++)
    {
        EXPECT_NEAR(forwards_linear_speed[i],
                    trajectory.linear_speed[path_parameters.num_elements - 1 - i],
                    0.0001);
        EXPECT_NEAR(forwards_angular_speed[i],
                    trajectory.angular_speed[path_parameters.num_elements - 1 - i],
                    0.0001);
        EXPECT_NEAR(forwards_orientation[i],
                    trajectory.orientation[path_parameters.num_elements - 1 - i], 0.0001);
        EXPECT_NEAR(forwards_position_x[i],
                    trajectory.x_position[path_parameters.num_elements - 1 - i], 0.0001);
        EXPECT_NEAR(forwards_position_y[i],
                    trajectory.y_position[path_parameters.num_elements - 1 - i], 0.0001);
        EXPECT_NEAR(forwards_time[i], trajectory.time_profile[i], 0.0001);
    }
}

TEST_F(TrajectoryPlannerTest, dynamics_dont_exceed_maximums_curved_path)
{
    Polynomial2dOrder3_t path = {
        .x = {.coefficients = {0, 2, 1, 0}},
        .y = {.coefficients = {3, 0, 1, 0}},

    };
    Polynomial1dOrder3_t orientation_profile = {.coefficients = {0, 0, 0, 0}};

    FirmwareRobotPathParameters_t path_parameters{
        .path                               = path,
        .orientation_profile                = orientation_profile,
        .t_start                            = -1,
        .t_end                              = 1,
        .num_elements                       = 500,
        .max_allowable_linear_acceleration  = 5,
        .max_allowable_linear_speed         = 5,
        .max_allowable_angular_acceleration = 0,
        .max_allowable_angular_speed        = 0,
        .initial_linear_speed               = 1.87f,
        .final_linear_speed                 = 5,

    };

    PositionTrajectory_t trajectory;

    TrajectoryPlannerGenerationStatus_t status =
        app_trajectory_planner_generateConstantParameterizationPositionTrajectory(
            path_parameters, &trajectory);
    EXPECT_EQ(OK, status);

    for (unsigned int i = 0; i < path_parameters.num_elements; i++)
    {
        EXPECT_TRUE(trajectory.linear_speed[i] <=
                    path_parameters.max_allowable_linear_speed);
    }

    EXPECT_NEAR(trajectory.linear_speed[path_parameters.num_elements - 1],
                path_parameters.final_linear_speed, 0.0001);

    EXPECT_FLOAT_EQ(trajectory.linear_speed[0], path_parameters.initial_linear_speed);

    EXPECT_NEAR(trajectory.x_position[path_parameters.num_elements - 1],
                shared_polynomial2d_getValueOrder3(path, path_parameters.t_end).x, 0.025);
    EXPECT_NEAR(trajectory.y_position[path_parameters.num_elements - 1],
                shared_polynomial2d_getValueOrder3(path, path_parameters.t_end).y, 0.025);

    EXPECT_NEAR(trajectory.x_position[0],
                shared_polynomial2d_getValueOrder3(path, path_parameters.t_start).x,
                0.025);
    EXPECT_NEAR(trajectory.y_position[0],
                shared_polynomial2d_getValueOrder3(path, path_parameters.t_start).y,
                0.025);
}

TEST_F(TrajectoryPlannerTest, test_get_constant_time_interpolation_straight_line)
{
    Polynomial2dOrder3_t path = {
        .x = {.coefficients = {0, 0, 1, 0}},
        .y = {.coefficients = {0, 0, 1, 0}},

    };
    Polynomial1dOrder3_t orientation_profile = {.coefficients = {0, 0, 0, 0}};

    FirmwareRobotPathParameters_t path_parameters{
        .path                               = path,
        .orientation_profile                = orientation_profile,
        .t_start                            = 0,
        .t_end                              = 1,
        .num_elements                       = 500,
        .max_allowable_linear_acceleration  = 3,
        .max_allowable_linear_speed         = 3,
        .max_allowable_angular_acceleration = 0,
        .max_allowable_angular_speed        = 0,
        .initial_linear_speed               = 0,
        .final_linear_speed                 = 0,

    };

    PositionTrajectory_t trajectory;

    TrajectoryPlannerGenerationStatus_t status =
        app_trajectory_planner_generateConstantParameterizationPositionTrajectory(
            path_parameters, &trajectory);
    const unsigned int variable_period_num_segments = path_parameters.num_elements;

    EXPECT_EQ(status, OK);

    PositionTrajectory_t const_interp_trajectory;

    status = app_trajectory_planner_generateConstantPeriodPositionTrajectory(
        0.001f, &path_parameters, &const_interp_trajectory);

    EXPECT_EQ(OK, status);

    EXPECT_NEAR(trajectory.x_position[variable_period_num_segments - 1],
                const_interp_trajectory.x_position[path_parameters.num_elements - 1],
                0.001);
    EXPECT_NEAR(trajectory.y_position[variable_period_num_segments - 1],
                const_interp_trajectory.y_position[path_parameters.num_elements - 1],
                0.001);
    EXPECT_NEAR(trajectory.time_profile[variable_period_num_segments - 1],
                const_interp_trajectory.time_profile[path_parameters.num_elements - 1],
                0.001);

    EXPECT_NEAR(trajectory.x_position[0], const_interp_trajectory.x_position[0], 0.001);
    EXPECT_NEAR(trajectory.y_position[0], const_interp_trajectory.x_position[0], 0.001);
    EXPECT_NEAR(trajectory.time_profile[0], const_interp_trajectory.time_profile[0],
                0.001);
}

TEST_F(TrajectoryPlannerTest, test_get_constant_time_iteration_curved_line)
{
    Polynomial2dOrder3_t path = {
        .x = {.coefficients = {2, 0, 1, 0}},
        .y = {.coefficients = {1, 0, 1, 0}},

    };
    Polynomial1dOrder3_t orientation_profile = {.coefficients = {0, 0, 0, 0}

    };

    FirmwareRobotPathParameters_t path_parameters{
        .path                               = path,
        .orientation_profile                = orientation_profile,
        .t_start                            = 0,
        .t_end                              = 1,
        .num_elements                       = 500,
        .max_allowable_linear_acceleration  = 3,
        .max_allowable_linear_speed         = 3,
        .max_allowable_angular_acceleration = 0,
        .max_allowable_angular_speed        = 0,
        .initial_linear_speed               = 0,
        .final_linear_speed                 = 0,

    };

    PositionTrajectory_t trajectory;
    TrajectoryPlannerGenerationStatus_t status =
        app_trajectory_planner_generateConstantParameterizationPositionTrajectory(
            path_parameters, &trajectory);
    EXPECT_EQ(status, OK);
    EXPECT_EQ(OK, status);
    const unsigned int variable_period_num_segments = path_parameters.num_elements;

    PositionTrajectory_t const_interp_trajectory;
    status = app_trajectory_planner_generateConstantPeriodPositionTrajectory(
        0.001, &path_parameters, &const_interp_trajectory);

    EXPECT_NEAR(trajectory.x_position[variable_period_num_segments - 1],
                const_interp_trajectory.x_position[path_parameters.num_elements - 1],
                0.001);
    EXPECT_NEAR(trajectory.y_position[variable_period_num_segments - 1],
                const_interp_trajectory.y_position[path_parameters.num_elements - 1],
                0.001);
    EXPECT_NEAR(trajectory.time_profile[variable_period_num_segments - 1],
                const_interp_trajectory.time_profile[path_parameters.num_elements - 1],
                0.001);

    EXPECT_NEAR(trajectory.x_position[0], const_interp_trajectory.x_position[0], 0.001);
    EXPECT_NEAR(trajectory.y_position[0], const_interp_trajectory.x_position[0], 0.001);
    EXPECT_NEAR(trajectory.time_profile[0], const_interp_trajectory.time_profile[0],
                0.001);
}


// This test generates a scenario where there is not enough elements in the
// TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS specified array as there path is quite long and
// the maximum speed of the path is low. This means that most (likely all) constant
// parameterization segments require multiple tion periods to traverse, and
// therefore requires more space than the constant parameterization trajectory that is
// already 1 element away from TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS
TEST_F(TrajectoryPlannerTest, test_get_constant_time_tion_too_many_elements)
{
    Polynomial2dOrder3_t path = {
        .x = {.coefficients = {2, 0, 1, 0}},
        .y = {.coefficients = {1, 0, 1, 0}},

    };
    Polynomial1dOrder3_t orientation_profile = {.coefficients = {0, 0, 0, 0}};

    FirmwareRobotPathParameters_t path_parameters{
        .path                               = path,
        .orientation_profile                = orientation_profile,
        .t_start                            = 0,
        .t_end                              = 2,
        .num_elements                       = TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS - 1,
        .max_allowable_linear_acceleration  = 10,
        .max_allowable_linear_speed         = 0.1f,
        .max_allowable_angular_acceleration = 0,
        .max_allowable_angular_speed        = 0,
        .initial_linear_speed               = 0,
        .final_linear_speed                 = 0,

    };

    PositionTrajectory_t const_interp_trajectory;

    // Calculate the constant-tion period equivalent of the trajectory
    TrajectoryPlannerGenerationStatus_t status =
        app_trajectory_planner_generateConstantPeriodPositionTrajectory(
            0.01, &path_parameters, &const_interp_trajectory);

    EXPECT_EQ(INTERPOLATION_ELEMENT_MAXED_OUT, status);
}

TEST_F(TrajectoryPlannerTest, test_assert_cannot_reach_final_velocity)
{
    Polynomial2dOrder3_t path = {
        .x = {.coefficients = {0, 0, 1, 0}},
        .y = {.coefficients = {0, 0, 1, 0}},

    };
    Polynomial1dOrder3_t orientation_profile = {.coefficients = {0, 0, 0, 0}};

    FirmwareRobotPathParameters_t path_parameters{
        .path                               = path,
        .orientation_profile                = orientation_profile,
        .t_start                            = 0,
        .t_end                              = 1,
        .num_elements                       = TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS - 1,
        .max_allowable_linear_acceleration  = 3,
        .max_allowable_linear_speed         = 2000,
        .max_allowable_angular_acceleration = 0,
        .max_allowable_angular_speed        = 0,
        .initial_linear_speed               = 0,
        .final_linear_speed                 = 3000,

    };

    PositionTrajectory_t trajectory;


    TrajectoryPlannerGenerationStatus_t status =
        app_trajectory_planner_generateConstantParameterizationPositionTrajectory(
            path_parameters, &trajectory);

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
    Polynomial1dOrder3_t orientation_profile = {.coefficients = {0, 0, 0, 0}};

    FirmwareRobotPathParameters_t path_parameters{
        .path                               = path,
        .orientation_profile                = orientation_profile,
        .t_start                            = 0,
        .t_end                              = 3,
        .num_elements                       = TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS,
        .max_allowable_linear_acceleration  = 3,
        .max_allowable_linear_speed         = 50,
        .max_allowable_angular_acceleration = 0,
        .max_allowable_angular_speed        = 0,
        .initial_linear_speed               = 20,
        .final_linear_speed                 = 0,

    };

    PositionTrajectory_t trajectory;

    TrajectoryPlannerGenerationStatus_t status =
        app_trajectory_planner_generateConstantParameterizationPositionTrajectory(
            path_parameters, &trajectory);

    EXPECT_EQ(INITIAL_VELOCITY_TOO_HIGH, status);
}

TEST_F(TrajectoryPlannerTest, velocity_trajectory_straight_line_high_acceleration)

{
    Polynomial2dOrder3_t path = {
        .x = {.coefficients = {0, 0, 1, 0}},
        .y = {.coefficients = {0, 0, 1, 0}},

    };
    Polynomial1dOrder3_t orientation_profile = {.coefficients = {0, 0, 0, 0}};

    FirmwareRobotPathParameters_t path_parameters{
        .path                               = path,
        .orientation_profile                = orientation_profile,
        .t_start                            = 0,
        .t_end                              = 3,
        .num_elements                       = 1000,
        .max_allowable_linear_acceleration  = 10,
        .max_allowable_linear_speed         = 9,
        .max_allowable_angular_acceleration = 0,
        .max_allowable_angular_speed        = 0,
        .initial_linear_speed               = 2,
        .final_linear_speed                 = 5,

    };

    float max_allowable_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    PositionTrajectory_t position_trajectory;

    app_trajectory_planner_impl_getMaximumSpeedProfile(
        path_parameters.path, path_parameters.num_elements, path_parameters.t_start,
        path_parameters.t_end, path_parameters.max_allowable_linear_acceleration,
        path_parameters.max_allowable_linear_speed, max_allowable_speed_profile);

    app_trajectory_planner_generateConstantParameterizationPositionTrajectory(
        path_parameters, &position_trajectory);

    VelocityTrajectory_t velocity_trajectory;

    app_trajectory_planner_generateVelocityTrajectory(
        &position_trajectory, path_parameters.num_elements, &velocity_trajectory);

    // Run test class function to get all the unit vectors for the velocity at each
    // point on the trajectory
    std::vector<Vector> direction_unit_vectors =
        getDirectionVectorsFromPositionTrajectory(&position_trajectory, path_parameters);

    // Check that every velocity has the correct magnitude
    for (unsigned int i = 0; i < path_parameters.num_elements; i++)
    {
        const float element_speed = sqrt(pow(velocity_trajectory.x_velocity[i], 2) +
                                         pow(velocity_trajectory.y_velocity[i], 2));
        EXPECT_NEAR(position_trajectory.linear_speed[i], element_speed, 0.001);

        EXPECT_NEAR(direction_unit_vectors[i].x() * position_trajectory.linear_speed[i],
                    velocity_trajectory.x_velocity[i], 0.001);
        EXPECT_NEAR(direction_unit_vectors[i].y() * position_trajectory.linear_speed[i],
                    velocity_trajectory.y_velocity[i], 0.001);
    }
}

TEST_F(TrajectoryPlannerTest, velocity_trajectory_parabola_path_high_acceleration)
{
    Polynomial2dOrder3_t path = {
        .x = {.coefficients = {0, 0, 1, 0}},
        .y = {.coefficients = {0, 2, 0, 0}},

    };
    Polynomial1dOrder3_t orientation_profile = {.coefficients = {0, 0, 0, 0}};

    FirmwareRobotPathParameters_t path_parameters{
        .path                               = path,
        .orientation_profile                = orientation_profile,
        .t_start                            = -1,
        .t_end                              = 1,
        .num_elements                       = 1000,
        .max_allowable_linear_acceleration  = 10,
        .max_allowable_linear_speed         = 5,
        .max_allowable_angular_acceleration = 0,
        .max_allowable_angular_speed        = 0,
        .initial_linear_speed               = 0,
        .final_linear_speed                 = 0,

    };

    float max_allowable_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    PositionTrajectory_t position_trajectory;

    app_trajectory_planner_generateConstantParameterizationPositionTrajectory(
        path_parameters, &position_trajectory);

    VelocityTrajectory_t velocity_trajectory;

    app_trajectory_planner_generateVelocityTrajectory(
        &position_trajectory, path_parameters.num_elements, &velocity_trajectory);

    // Run test class function to get all the unit vectors for the velocity at each point
    // on the trajectory
    std::vector<Vector> direction_unit_vectors =
        getDirectionVectorsFromPositionTrajectory(&position_trajectory, path_parameters);

    // Check that every velocity has the correct magnitude
    for (unsigned int i = 0; i < path_parameters.num_elements; i++)
    {
        const float element_speed = sqrt(pow(velocity_trajectory.x_velocity[i], 2) +
                                         pow(velocity_trajectory.y_velocity[i], 2));
        EXPECT_FLOAT_EQ(position_trajectory.linear_speed[i], element_speed);

        EXPECT_FLOAT_EQ(
            direction_unit_vectors[i].x() * position_trajectory.linear_speed[i],
            velocity_trajectory.x_velocity[i]);
        EXPECT_FLOAT_EQ(
            direction_unit_vectors[i].y() * position_trajectory.linear_speed[i],
            velocity_trajectory.y_velocity[i]);
    }
}

TEST_F(TrajectoryPlannerTest, velocity_trajectory_curved_path_low_acceleration)
{
    Polynomial2dOrder3_t path = {
        .x = {.coefficients = {3, 0, 1, 0}},
        .y = {.coefficients = {0, 2, 0, 0}},

    };
    Polynomial1dOrder3_t orientation_profile = {.coefficients = {0, 0, 0, 0}};

    FirmwareRobotPathParameters_t path_parameters{
        .path                               = path,
        .orientation_profile                = orientation_profile,
        .t_start                            = -1,
        .t_end                              = 1,
        .num_elements                       = 1000,
        .max_allowable_linear_acceleration  = 3,
        .max_allowable_linear_speed         = 5,
        .max_allowable_angular_acceleration = 0,
        .max_allowable_angular_speed        = 0,
        .initial_linear_speed               = 2,
        .final_linear_speed                 = 1,

    };

    PositionTrajectory_t position_trajectory;

    VelocityTrajectory_t velocity_trajectory;

    app_trajectory_planner_generateConstantParameterizationPositionTrajectory(
        path_parameters, &position_trajectory);

    app_trajectory_planner_generateVelocityTrajectory(
        &position_trajectory, path_parameters.num_elements, &velocity_trajectory);


    // Run test class function to get all the unit vectors for the velocity at each
    // on the trajectory
    std::vector<Vector> direction_unit_vectors =
        getDirectionVectorsFromPositionTrajectory(&position_trajectory, path_parameters);

    // Check that every velocity has the correct magnitude
    for (unsigned int i = 0; i < path_parameters.num_elements; i++)
    {
        const float element_speed = sqrt(pow(velocity_trajectory.x_velocity[i], 2) +
                                         pow(velocity_trajectory.y_velocity[i], 2));
        EXPECT_FLOAT_EQ(position_trajectory.linear_speed[i], element_speed);

        EXPECT_FLOAT_EQ(
            direction_unit_vectors[i].x() * position_trajectory.linear_speed[i],
            velocity_trajectory.x_velocity[i]);
        EXPECT_FLOAT_EQ(
            direction_unit_vectors[i].y() * position_trajectory.linear_speed[i],
            velocity_trajectory.y_velocity[i]);
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
                          (path_parameters.num_elements - 1);

    // Check that all of the state variables are correct
    for (unsigned int i = 0; i < path_parameters.num_elements; i++)
    {
        const Vector2d_t expected_position =
            shared_polynomial2d_getValueOrder3(path, i * delta_t);
        const float expected_orientation =
            shared_polynomial1d_getValueOrder3(orientation_profile, i * delta_t);

        EXPECT_FLOAT_EQ(expected_orientation, theta_values[i]);
        EXPECT_FLOAT_EQ(expected_position.x, x_values[i]);
        EXPECT_FLOAT_EQ(expected_position.y, y_values[i]);
    }
    EXPECT_FLOAT_EQ(y_values[path_parameters.num_elements - 1], 1);
    EXPECT_FLOAT_EQ(x_values[path_parameters.num_elements - 1], 1);
    EXPECT_FLOAT_EQ(theta_values[path_parameters.num_elements - 1], 1);
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

    const float delta_length = total_path_length_linear / path_parameters.num_elements;
    const float delta_length_orientation =
        total_path_length_angular / path_parameters.num_elements;
    // Check that all of the segment lengths are correct
    for (unsigned int i = 0; i < path_parameters.num_elements - 1; i++)
    {
        EXPECT_NEAR(segment_lengths_linear[i], delta_length, 0.00001);
        EXPECT_NEAR(segment_lengths_angular[i], delta_length_orientation, 0.00001);
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
        total_path_length_linear / (path_parameters.num_elements - 1);
    const float delta_length_orientation =
        total_path_length_angular / (path_parameters.num_elements - 1);
    // Check that all of the segment lengths are correct
    for (unsigned int i = 0; i < path_parameters.num_elements - 1; i++)
    {
        EXPECT_NEAR(segment_lengths_linear[i], delta_length, 0.00001);
        EXPECT_NEAR(segment_lengths_angular[i], delta_length_orientation, 0.00001);
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

TEST_F(TrajectoryPlannerTest, test_forwards_continuity_varying_segment_length)
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

TEST_F(TrajectoryPlannerTest, test_forwards_continuity_varying_segment_length_angular)
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

TEST_F(TrajectoryPlannerTest,
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

TEST_F(TrajectoryPlannerTest,
       test_backwards_continuity_constant_segment_length_continuous)
{
    float max_allowable_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    const unsigned int num_segments = 4;


    float segment_lengths_linear[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    segment_lengths_linear[0] = 1.0;
    segment_lengths_linear[1] = 1.0;
    segment_lengths_linear[2] = 1.0;


    FirmwareRobotPathParameters_t path_parameters;
    path_parameters.max_allowable_linear_acceleration = 1;
    path_parameters.max_allowable_linear_speed        = 1;
    path_parameters.num_elements                      = num_segments;
    path_parameters.initial_linear_speed              = sqrt(7);
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

TEST_F(TrajectoryPlannerTest, test_backwards_continuity_variable_segment_length_input)
{
    float max_allowable_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    const unsigned int num_segments = 4;


    float segment_lengths_linear[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    segment_lengths_linear[0] = 1.0;
    segment_lengths_linear[1] = 2.0;
    segment_lengths_linear[2] = 3.0;


    FirmwareRobotPathParameters_t path_parameters;
    path_parameters.max_allowable_linear_acceleration = 1;
    path_parameters.max_allowable_linear_speed        = 1;
    path_parameters.num_elements                      = num_segments;
    path_parameters.initial_linear_speed              = sqrt(7);
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

TEST_F(
    TrajectoryPlannerTest,
    test_backwards_continuity_variable_segment_length_input_and_initial_velocity_too_high)
{
    // Create an 'initial velocity too high' state by requesting an initial velocity that
    // is much greater than physically possible given the path length and acceleration
    float max_allowable_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

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

TEST_F(TrajectoryPlannerTest,
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

TEST_F(TrajectoryPlannerTest,
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

TEST_F(TrajectoryPlannerTest, test_generate_maximum_speed_curve_for_a_straight_line)
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

TEST_F(TrajectoryPlannerTest,
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

TEST_F(TrajectoryPlannerTest, test_generate_time_profile_no_angular_profile)
{
    // Create a scenario where the linear path parameters will limit the speed of the
    // trajectory because there is no orientation path to follow

    Polynomial2dOrder3_t path                = {.x = {0, 0, 1, 0}, .y = {0, 0, 1, 0}};
    Polynomial1dOrder3_t orientation_profile = {.coefficients = {0, 0, 0, 0}};
    // Generate parameters for a trajectory
    FirmwareRobotPathParameters_t path_parameters{
        .path                               = path,
        .orientation_profile                = orientation_profile,
        .t_start                            = 1,
        .t_end                              = 2,
        .num_elements                       = 100,
        .max_allowable_linear_acceleration  = 1,
        .max_allowable_linear_speed         = 1,
        .max_allowable_angular_acceleration = 100,
        .max_allowable_angular_speed        = 100,
        .initial_linear_speed               = 1,
        .final_linear_speed                 = 1,

    };

    float x_states[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    float y_states[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    float segment_lengths[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    PositionTrajectory_t trajectory;

    app_trajectory_planner_impl_generate2dSegmentNodesAndLengths(
        path_parameters.t_start, path_parameters.t_end, path_parameters.path,
        path_parameters.num_elements, x_states, y_states, segment_lengths);
    TrajectoryPlannerGenerationStatus_t status =
        app_trajectory_planner_generateConstantParameterizationPositionTrajectory(
            path_parameters, &trajectory);
    EXPECT_EQ(status, OK);

    EXPECT_FLOAT_EQ(trajectory.time_profile[0], 0.0);
    for (unsigned int i = 1; i < path_parameters.num_elements; i++)
    {
        EXPECT_NEAR(trajectory.time_profile[i],
                    trajectory.time_profile[i - 1] +
                        segment_lengths[i - 1] / path_parameters.initial_linear_speed,
                    0.0001);
    }
    EXPECT_NEAR(trajectory.time_profile[path_parameters.num_elements - 1],
                sqrt(2) / path_parameters.initial_linear_speed, 0.0001);
}

TEST_F(TrajectoryPlannerTest, test_generate_time_profile_linear_limiting_angular)
{
    // Create a scenario where the linear path parameters will limit the speed of the
    // trajectory because the orientation profile is much faster than the linear

    Polynomial2dOrder3_t path                = {.x = {0, 0, 1, 0}, .y = {0, 0, 1, 0}};
    Polynomial1dOrder3_t orientation_profile = {.coefficients = {0, 0, 1, 0}};
    // Generate parameters for a trajectory
    FirmwareRobotPathParameters_t path_parameters{
        .path                               = path,
        .orientation_profile                = orientation_profile,
        .t_start                            = 1,
        .t_end                              = 2,
        .num_elements                       = 100,
        .max_allowable_linear_acceleration  = 1,
        .max_allowable_linear_speed         = 1,
        .max_allowable_angular_acceleration = 100,
        .max_allowable_angular_speed        = 100,
        .initial_linear_speed               = 1,
        .final_linear_speed                 = 1,

    };

    float x_states[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    float y_states[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    float segment_lengths[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    PositionTrajectory_t trajectory;

    app_trajectory_planner_impl_generate2dSegmentNodesAndLengths(
        path_parameters.t_start, path_parameters.t_end, path_parameters.path,
        path_parameters.num_elements, x_states, y_states, segment_lengths);
    TrajectoryPlannerGenerationStatus_t status =
        app_trajectory_planner_generateConstantParameterizationPositionTrajectory(
            path_parameters, &trajectory);
    EXPECT_EQ(status, OK);

    EXPECT_FLOAT_EQ(trajectory.time_profile[0], 0.0);
    for (unsigned int i = 1; i < path_parameters.num_elements; i++)
    {
        EXPECT_NEAR(trajectory.time_profile[i],
                    trajectory.time_profile[i - 1] +
                        segment_lengths[i - 1] / path_parameters.initial_linear_speed,
                    0.0001);
    }
    EXPECT_NEAR(trajectory.time_profile[path_parameters.num_elements - 1],
                sqrt(2) / path_parameters.initial_linear_speed, 0.0001);
}

TEST_F(TrajectoryPlannerTest, test_generate_time_profile_no_linear_profile)
{
    // Create a scenario where the angular profile will limit the speed of the
    // trajectory because the linear trajectory is zeros and takes no time

    Polynomial2dOrder3_t path                = {.x = {0, 0, 0, 0}, .y = {0, 0, 0, 0}};
    Polynomial1dOrder3_t orientation_profile = {.coefficients = {0, 0, 1, 0}};
    // Generate parameters for a trajectory
    FirmwareRobotPathParameters_t path_parameters{
        .path                               = path,
        .orientation_profile                = orientation_profile,
        .t_start                            = 1,
        .t_end                              = 2,
        .num_elements                       = 100,
        .max_allowable_linear_acceleration  = 1,
        .max_allowable_linear_speed         = 1,
        .max_allowable_angular_acceleration = 100,
        .max_allowable_angular_speed        = 1,
        .initial_linear_speed               = 1,
        .final_linear_speed                 = 1,

    };

    float theta_states[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    float segment_lengths[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    PositionTrajectory_t trajectory;

    app_trajectory_planner_impl_generate1dSegmentNodesAndLengths(
        path_parameters.t_start, path_parameters.t_end,
        path_parameters.orientation_profile, path_parameters.num_elements, theta_states,
        segment_lengths);
    TrajectoryPlannerGenerationStatus_t status =
        app_trajectory_planner_generateConstantParameterizationPositionTrajectory(
            path_parameters, &trajectory);
    EXPECT_EQ(status, OK);

    EXPECT_FLOAT_EQ(trajectory.time_profile[0], 0.0);
    EXPECT_NEAR(trajectory.time_profile[1],
                (2 * segment_lengths[0] /
                 (trajectory.angular_speed[0] + trajectory.angular_speed[1])),
                0.0001);
    for (unsigned int i = 2; i < path_parameters.num_elements - 1; i++)
    {
        EXPECT_EQ(
            trajectory.time_profile[i],
            trajectory.time_profile[i - 1] +
                segment_lengths[i - 1] / path_parameters.max_allowable_angular_speed);
    }
    const float radians_moved_in_acceleration_phase =
        trajectory.orientation[path_parameters.num_elements - 1] -
        trajectory.orientation[path_parameters.num_elements - 2];

    EXPECT_NEAR(trajectory.time_profile[path_parameters.num_elements - 1], 1.0202,
                0.0001);
}

TEST_F(TrajectoryPlannerTest,
       test_generate_time_profile_orientation_profile_limiting_linear)
{
    // Create a scenario where the linear path parameters will limit the speed of the
    // trajectory by using angular parameters that are much greater
    Polynomial2dOrder3_t path                = {.x = {0, 0, 1, 0}, .y = {0, 0, 1, 0}};
    Polynomial1dOrder3_t orientation_profile = {.coefficients = {0, 0, 1, 0}};
    // Generate parameters for a trajectory
    FirmwareRobotPathParameters_t path_parameters{
        .path                               = path,
        .orientation_profile                = orientation_profile,
        .t_start                            = 1,
        .t_end                              = 10,
        .num_elements                       = 100,
        .max_allowable_linear_acceleration  = 100,
        .max_allowable_linear_speed         = 100,
        .max_allowable_angular_acceleration = 100,
        .max_allowable_angular_speed        = 1,
        .initial_linear_speed               = 1,
        .final_linear_speed                 = 1,

    };

    float theta_states[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    float segment_lengths[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    PositionTrajectory_t trajectory;

    app_trajectory_planner_impl_generate1dSegmentNodesAndLengths(
        path_parameters.t_start, path_parameters.t_end,
        path_parameters.orientation_profile, path_parameters.num_elements, theta_states,
        segment_lengths);
    TrajectoryPlannerGenerationStatus_t status =
        app_trajectory_planner_generateConstantParameterizationPositionTrajectory(
            path_parameters, &trajectory);

    EXPECT_FLOAT_EQ(trajectory.time_profile[0], 0.0);
    EXPECT_NEAR(trajectory.time_profile[1],
                (2 * segment_lengths[0] /
                 (trajectory.angular_speed[0] + trajectory.angular_speed[1])),
                0.0001);
    EXPECT_FLOAT_EQ(trajectory.time_profile[0], 0.0);
    EXPECT_NEAR(trajectory.time_profile[1], 0.1818, 0.0001);
    for (unsigned int i = 2; i < path_parameters.num_elements - 1; i++)
    {
        EXPECT_EQ(
            trajectory.time_profile[i],
            trajectory.time_profile[i - 1] +
                segment_lengths[i - 1] / path_parameters.max_allowable_angular_speed);
    }
    const float radians_moved_in_acceleration_phase =
        trajectory.orientation[path_parameters.num_elements - 1] -
        trajectory.orientation[path_parameters.num_elements - 2];

    EXPECT_NEAR(trajectory.time_profile[path_parameters.num_elements - 1], 9.1818,
                0.0001);
}

TEST_F(TrajectoryPlannerTest, test_rebalance_trajectory_segment_to_mach_duration)
{
    float segment_lengths_meters[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    float speeds[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    // Create an artificial trajectory here for testing
    speeds[0]                  = 1;
    speeds[1]                  = 4;
    float desired_time_seconds = 0.01f;

    segment_lengths_meters[0] = 1.0;

    speeds[1] = app_trajectory_planner_impl_modifySpeedToMatchDuration(
        speeds[0], desired_time_seconds, segment_lengths_meters[0]);
    EXPECT_FLOAT_EQ(speeds[1], 201);
    EXPECT_FLOAT_EQ(speeds[0], 1);
}

TEST_F(TrajectoryPlannerTest, test_generate_velocity_trajectory_zeros)
{
    Polynomial2dOrder3_t path                = {.x = {0, 0, 0, 0}, .y = {0, 0, 0, 0}};
    Polynomial1dOrder3_t orientation_profile = {.coefficients = {0, 0, 0, 0}};
    // Generate parameters for a trajectory
    FirmwareRobotPathParameters_t path_parameters{
        .path                               = path,
        .orientation_profile                = orientation_profile,
        .t_start                            = 0,
        .t_end                              = 1,
        .num_elements                       = 100,
        .max_allowable_linear_acceleration  = 1,
        .max_allowable_linear_speed         = 1,
        .max_allowable_angular_acceleration = 1,
        .max_allowable_angular_speed        = 1,
        .initial_linear_speed               = 0,
        .final_linear_speed                 = 0,

    };

    VelocityTrajectory_t velocity_trajectory;
    PositionTrajectory_t position_trajectory;

    TrajectoryPlannerGenerationStatus_t status =
        app_trajectory_planner_generateConstantParameterizationPositionTrajectory(
            path_parameters, &position_trajectory);
    EXPECT_EQ(status, OK);
    app_trajectory_planner_generateVelocityTrajectory(
        &position_trajectory, path_parameters.num_elements, &velocity_trajectory);

    for (unsigned int i = 0; i < path_parameters.num_elements; i++)
    {
        EXPECT_FLOAT_EQ(velocity_trajectory.angular_velocity[i], 0);
        EXPECT_FLOAT_EQ(velocity_trajectory.x_velocity[i], 0);
        EXPECT_FLOAT_EQ(velocity_trajectory.y_velocity[i], 0);
        EXPECT_FLOAT_EQ(velocity_trajectory.time_profile[i], 0);
    }
}

TEST_F(TrajectoryPlannerTest,
       test_generate_velocity_trajectory_linear_path_no_orientation)
{
    // Create a scenario where the linear path parameters will limit the speed of the
    // trajectory since there angular profile is zeros and takes no time to complete
    Polynomial2dOrder3_t path                = {.x = {0, 0, 1, 0}, .y = {0, 0, 1, 0}};
    Polynomial1dOrder3_t orientation_profile = {.coefficients = {0, 0, 0, 0}};
    // Generate parameters for a trajectory
    FirmwareRobotPathParameters_t path_parameters{
        .path                               = path,
        .orientation_profile                = orientation_profile,
        .t_start                            = 1,
        .t_end                              = 10,
        .num_elements                       = 100,
        .max_allowable_linear_acceleration  = 1,
        .max_allowable_linear_speed         = 1,
        .max_allowable_angular_acceleration = 1,
        .max_allowable_angular_speed        = 1,
        .initial_linear_speed               = 1,
        .final_linear_speed                 = 1,

    };

    VelocityTrajectory_t velocity_trajectory;
    PositionTrajectory_t position_trajectory;

    TrajectoryPlannerGenerationStatus_t status =
        app_trajectory_planner_generateConstantParameterizationPositionTrajectory(
            path_parameters, &position_trajectory);
    EXPECT_EQ(status, OK);
    app_trajectory_planner_generateVelocityTrajectory(
        &position_trajectory, path_parameters.num_elements, &velocity_trajectory);

    std::vector<Vector> directions =
        getDirectionVectorsFromPositionTrajectory(&position_trajectory, path_parameters);
    const float segment_length = 9 * sqrt(2) / (path_parameters.num_elements - 1);

    for (unsigned int i = 0; i < path_parameters.num_elements; i++)
    {
        EXPECT_FLOAT_EQ(velocity_trajectory.angular_velocity[i], 0);
        EXPECT_FLOAT_EQ(velocity_trajectory.x_velocity[i], directions[i].x());
        EXPECT_FLOAT_EQ(velocity_trajectory.y_velocity[i], directions[i].y());
        EXPECT_NEAR(velocity_trajectory.time_profile[i], i * segment_length, 0.00001);
    }
}

TEST_F(TrajectoryPlannerTest,
       test_generate_velocity_trajectory_no_path_linear_orientation)
{
    // Create a scenario where the linear path parameters will limit the speed of the
    // trajectory by using angular parameters that are much greater
    Polynomial2dOrder3_t path                = {.x = {0, 0, 0, 0}, .y = {0, 0, 0, 0}};
    Polynomial1dOrder3_t orientation_profile = {.coefficients = {0, 0, 1, 0}};
    // Generate parameters for a trajectory
    FirmwareRobotPathParameters_t path_parameters{
        .path                               = path,
        .orientation_profile                = orientation_profile,
        .t_start                            = 1,
        .t_end                              = 10,
        .num_elements                       = 100,
        .max_allowable_linear_acceleration  = 1,
        .max_allowable_linear_speed         = 1,
        .max_allowable_angular_acceleration = 100,
        .max_allowable_angular_speed        = 1,
        .initial_linear_speed               = 0,
        .final_linear_speed                 = 0,

    };
    VelocityTrajectory_t velocity_trajectory;
    PositionTrajectory_t position_trajectory;

    TrajectoryPlannerGenerationStatus_t status =
        app_trajectory_planner_generateConstantParameterizationPositionTrajectory(
            path_parameters, &position_trajectory);
    EXPECT_EQ(status, OK);
    app_trajectory_planner_generateVelocityTrajectory(
        &position_trajectory, path_parameters.num_elements, &velocity_trajectory);

    std::vector<Vector> directions =
        getDirectionVectorsFromPositionTrajectory(&position_trajectory, path_parameters);
    const float segment_length           = 9.0 / (path_parameters.num_elements - 1);
    const float initial_segment_duration = (2 * segment_length) / (1);

    EXPECT_FLOAT_EQ(velocity_trajectory.angular_velocity[0], 0);
    for (unsigned int i = 1; i < path_parameters.num_elements - 1; i++)
    {
        EXPECT_FLOAT_EQ(velocity_trajectory.angular_velocity[i], 1);
        EXPECT_FLOAT_EQ(velocity_trajectory.x_velocity[i], 0);
        EXPECT_FLOAT_EQ(velocity_trajectory.y_velocity[i], 0);
        EXPECT_NEAR(velocity_trajectory.time_profile[i],
                    initial_segment_duration + (i - 1) * segment_length, 0.00001);
    }
    EXPECT_FLOAT_EQ(
        velocity_trajectory.angular_velocity[path_parameters.num_elements - 1],

        0);
}

TEST_F(TrajectoryPlannerTest,
       test_generate_velocity_trajectory_polynomial_path_polynomial_orientation)
{
    // Create a scenario where the linear path parameters will limit the speed of the
    // trajectory by using angular parameters that are much greater
    Polynomial2dOrder3_t path                = {.x = {1, 0, 2, 0}, .y = {0, 0, 2, 0}};
    Polynomial1dOrder3_t orientation_profile = {.coefficients = {1, 0, 1, 0}};
    // Generate parameters for a trajectory
    FirmwareRobotPathParameters_t path_parameters = {
        .path                               = path,
        .orientation_profile                = orientation_profile,
        .t_start                            = 0,
        .t_end                              = 10,
        .num_elements                       = 100,
        .max_allowable_linear_acceleration  = 1,
        .max_allowable_linear_speed         = 1.1f,
        .max_allowable_angular_acceleration = 100,
        .max_allowable_angular_speed        = 1.2f,
        .initial_linear_speed               = 0,
        .final_linear_speed                 = 0,

    };
    VelocityTrajectory_t velocity_trajectory;
    PositionTrajectory_t position_trajectory;

    TrajectoryPlannerGenerationStatus_t status =
        app_trajectory_planner_generateConstantParameterizationPositionTrajectory(
            path_parameters, &position_trajectory);
    EXPECT_EQ(status, OK);
    app_trajectory_planner_generateVelocityTrajectory(
        &position_trajectory, path_parameters.num_elements, &velocity_trajectory);

    std::vector<Vector> directions =
        getDirectionVectorsFromPositionTrajectory(&position_trajectory, path_parameters);
    const float segment_length           = sqrt(8.0) / (path_parameters.num_elements - 1);
    const float initial_segment_duration = (2 * segment_length) / (1);

    for (unsigned int i = 0; i < path_parameters.num_elements - 1; i++)
    {
        EXPECT_FLOAT_EQ(velocity_trajectory.angular_velocity[i],
                        position_trajectory.angular_speed[i]);
        EXPECT_FLOAT_EQ(velocity_trajectory.x_velocity[i],
                        position_trajectory.linear_speed[i] * directions[i].x());
        EXPECT_FLOAT_EQ(velocity_trajectory.y_velocity[i],
                        position_trajectory.linear_speed[i] * directions[i].y());
        EXPECT_NEAR(velocity_trajectory.time_profile[i],
                    position_trajectory.time_profile[i], 0.00001);
    }
    EXPECT_FLOAT_EQ(
        velocity_trajectory.angular_velocity[path_parameters.num_elements - 1], 0);
}

TEST_F(TrajectoryPlannerTest,
       test_generate_constant_interpolation_velocity_trajectory_zeros)
{
    // Create a scenario where the linear path parameters will limit the speed of the
    // trajectory by using angular parameters that are much greater
    Polynomial2dOrder3_t path                = {.x = {0, 0, 0, 0}, .y = {0, 0, 0, 0}};
    Polynomial1dOrder3_t orientation_profile = {.coefficients = {0, 0, 0, 0}};
    // Generate parameters for a trajectory
    FirmwareRobotPathParameters_t path_parameters = {
        .path                               = path,
        .orientation_profile                = orientation_profile,
        .t_start                            = 1,
        .t_end                              = 10,
        .num_elements                       = 100,
        .max_allowable_linear_acceleration  = 1,
        .max_allowable_linear_speed         = 1,
        .max_allowable_angular_acceleration = 1,
        .max_allowable_angular_speed        = 1,
        .initial_linear_speed               = 0,
        .final_linear_speed                 = 0,

    };

    VelocityTrajectory_t velocity_trajectory;
    PositionTrajectory_t position_trajectory;

    TrajectoryPlannerGenerationStatus_t status =
        app_trajectory_planner_generateConstantPeriodPositionTrajectory(
            0.01f, &path_parameters, &position_trajectory);
    EXPECT_EQ(status, OK);
    app_trajectory_planner_generateVelocityTrajectory(
        &position_trajectory, path_parameters.num_elements, &velocity_trajectory);
    EXPECT_EQ(status, OK);

    for (unsigned int i = 0; i < path_parameters.num_elements; i++)
    {
        EXPECT_FLOAT_EQ(velocity_trajectory.angular_velocity[i], 0);
        EXPECT_FLOAT_EQ(velocity_trajectory.x_velocity[i], 0);
        EXPECT_FLOAT_EQ(velocity_trajectory.y_velocity[i], 0);
        EXPECT_FLOAT_EQ(velocity_trajectory.time_profile[i], 0);
    }
}

TEST_F(
    TrajectoryPlannerTest,
    test_generate_constant_interpolation_period_velocity_trajectory_polynomial_path_polynomial_orientation)
{
    // Create a scenario where the linear path parameters will limit the speed of the
    // trajectory by using angular parameters that are much greater
    Polynomial2dOrder3_t path                = {.x = {1, 0, 2, 0}, .y = {0, 0, 2, 0}};
    Polynomial1dOrder3_t orientation_profile = {.coefficients = {1, 0, 1, 0}};
    // Generate parameters for a trajectory
    FirmwareRobotPathParameters_t path_parameters = {
        .path                               = path,
        .orientation_profile                = orientation_profile,
        .t_start                            = 0,
        .t_end                              = 2,
        .num_elements                       = 100,
        .max_allowable_linear_acceleration  = 1,
        .max_allowable_linear_speed         = 1.1f,
        .max_allowable_angular_acceleration = 1,
        .max_allowable_angular_speed        = 1.2f,
        .initial_linear_speed               = 0,
        .final_linear_speed                 = 0,

    };

    VelocityTrajectory_t velocity_trajectory;
    PositionTrajectory_t position_trajectory;

    TrajectoryPlannerGenerationStatus_t status =
        app_trajectory_planner_generateConstantPeriodPositionTrajectory(
            0.01f, &path_parameters, &position_trajectory);
    EXPECT_EQ(status, OK);
    app_trajectory_planner_generateVelocityTrajectory(
        &position_trajectory, path_parameters.num_elements, &velocity_trajectory);
    EXPECT_EQ(status, OK);

    std::vector<Vector> directions =
        getDirectionVectorsFromPositionTrajectory(&position_trajectory, path_parameters);

    const float segment_length = sqrt(8.0) / (path_parameters.num_elements - 1);

    for (unsigned int i = 0; i < path_parameters.num_elements - 1; i++)
    {
        EXPECT_FLOAT_EQ(velocity_trajectory.angular_velocity[i],
                        position_trajectory.angular_speed[i]);
        EXPECT_FLOAT_EQ(velocity_trajectory.x_velocity[i],
                        position_trajectory.linear_speed[i] * directions[i].x());
        EXPECT_FLOAT_EQ(velocity_trajectory.y_velocity[i],
                        position_trajectory.linear_speed[i] * directions[i].y());
        EXPECT_NEAR(velocity_trajectory.time_profile[i],
                    position_trajectory.time_profile[i], 0.00001);
    }
    EXPECT_FLOAT_EQ(
        velocity_trajectory.angular_velocity[path_parameters.num_elements - 1], 0);
}
