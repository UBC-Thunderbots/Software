extern "C"
{
#include "firmware/app/control/trajectory_planner.h"

#include "firmware/app/control/trajectory_planner_private.h"
}

#include <gtest/gtest.h>
#include <math.h>

#include <vector>

#include "firmware/shared/math/polynomial_1d.h"
#include "firmware/shared/math/polynomial_2d.h"
#include "firmware/shared/math/vector_2d.h"
#include "software/geom/vector.h"

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
        0.001f, &path_parameters, &const_interp_trajectory);

    EXPECT_NEAR(trajectory.x_position[variable_period_num_segments - 1],
                const_interp_trajectory.x_position[path_parameters.num_elements - 1],
                0.001f);
    EXPECT_NEAR(trajectory.y_position[variable_period_num_segments - 1],
                const_interp_trajectory.y_position[path_parameters.num_elements - 1],
                0.001f);
    EXPECT_NEAR(trajectory.time_profile[variable_period_num_segments - 1],
                const_interp_trajectory.time_profile[path_parameters.num_elements - 1],
                0.001f);

    EXPECT_NEAR(trajectory.x_position[0], const_interp_trajectory.x_position[0], 0.001f);
    EXPECT_NEAR(trajectory.y_position[0], const_interp_trajectory.x_position[0], 0.001f);
    EXPECT_NEAR(trajectory.time_profile[0], const_interp_trajectory.time_profile[0],
                0.001f);
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
            0.01f, &path_parameters, &const_interp_trajectory);

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

    PositionTrajectory_t position_trajectory;

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
        const float element_speed = sqrtf(powf(velocity_trajectory.x_velocity[i], 2) +
                                          powf(velocity_trajectory.y_velocity[i], 2));
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
        const float element_speed = sqrtf(powf(velocity_trajectory.x_velocity[i], 2) +
                                          powf(velocity_trajectory.y_velocity[i], 2));
        EXPECT_FLOAT_EQ(position_trajectory.linear_speed[i], element_speed);

        EXPECT_FLOAT_EQ(static_cast<float>(direction_unit_vectors[i].x()) *
                            position_trajectory.linear_speed[i],
                        velocity_trajectory.x_velocity[i]);
        EXPECT_FLOAT_EQ(static_cast<float>(direction_unit_vectors[i].y()) *
                            position_trajectory.linear_speed[i],
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
        const float element_speed = sqrtf(powf(velocity_trajectory.x_velocity[i], 2) +
                                          powf(velocity_trajectory.y_velocity[i], 2));
        EXPECT_FLOAT_EQ(position_trajectory.linear_speed[i], element_speed);

        EXPECT_FLOAT_EQ(static_cast<float>(direction_unit_vectors[i].x()) *
                            position_trajectory.linear_speed[i],
                        velocity_trajectory.x_velocity[i]);
        EXPECT_FLOAT_EQ(static_cast<float>(direction_unit_vectors[i].y()) *
                            position_trajectory.linear_speed[i],
                        velocity_trajectory.y_velocity[i]);
    }
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
    EXPECT_EQ(status, OK);

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

    EXPECT_NEAR(trajectory.time_profile[path_parameters.num_elements - 1], 9.1818,
                0.0001);
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
    const float segment_length =
        9 * sqrtf(2) / static_cast<float>(path_parameters.num_elements - 1);

    for (unsigned int i = 0; i < path_parameters.num_elements; i++)
    {
        EXPECT_FLOAT_EQ(velocity_trajectory.angular_velocity[i], 0);
        EXPECT_FLOAT_EQ(velocity_trajectory.x_velocity[i],
                        static_cast<float>(directions[i].x()));
        EXPECT_FLOAT_EQ(velocity_trajectory.y_velocity[i],
                        static_cast<float>(directions[i].y()));
        EXPECT_NEAR(velocity_trajectory.time_profile[i],
                    static_cast<float>(i) * segment_length, 0.00001);
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
    const float segment_length =
        9.0f / static_cast<float>(path_parameters.num_elements - 1);
    const float initial_segment_duration = (2 * segment_length) / (1);

    EXPECT_FLOAT_EQ(velocity_trajectory.angular_velocity[0], 0);
    for (unsigned int i = 1; i < path_parameters.num_elements - 1; i++)
    {
        EXPECT_FLOAT_EQ(velocity_trajectory.angular_velocity[i], 1);
        EXPECT_FLOAT_EQ(velocity_trajectory.x_velocity[i], 0);
        EXPECT_FLOAT_EQ(velocity_trajectory.y_velocity[i], 0);
        EXPECT_NEAR(velocity_trajectory.time_profile[i],
                    initial_segment_duration + static_cast<float>(i - 1) * segment_length,
                    0.00001);
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

    float max_orientation = 0.0f;
    for (unsigned int i = 0; i < path_parameters.num_elements - 1; i++)
    {
        // orientation should always be increasing
        EXPECT_GE(position_trajectory.orientation[i], max_orientation);
        if (position_trajectory.orientation[i] > max_orientation)
        {
            max_orientation = position_trajectory.orientation[i];
        }
    }
    EXPECT_GT(max_orientation, 900.0f);
    EXPECT_LT(max_orientation, 1000.0f);

    app_trajectory_planner_generateVelocityTrajectory(
        &position_trajectory, path_parameters.num_elements, &velocity_trajectory);

    std::vector<Vector> directions =
        getDirectionVectorsFromPositionTrajectory(&position_trajectory, path_parameters);

    float max_angular_speed = 0.0f;
    for (unsigned int i = 0; i < path_parameters.num_elements - 1; i++)
    {
        EXPECT_FLOAT_EQ(velocity_trajectory.angular_velocity[i],
                        position_trajectory.angular_speed[i]);
        EXPECT_FLOAT_EQ(
            velocity_trajectory.x_velocity[i],
            position_trajectory.linear_speed[i] * static_cast<float>(directions[i].x()));
        EXPECT_FLOAT_EQ(
            velocity_trajectory.y_velocity[i],
            position_trajectory.linear_speed[i] * static_cast<float>(directions[i].y()));
        if (velocity_trajectory.angular_velocity[i] > max_angular_speed)
        {
            max_angular_speed = velocity_trajectory.angular_velocity[i];
        }
        EXPECT_NEAR(velocity_trajectory.time_profile[i],
                    position_trajectory.time_profile[i], 0.00001);
    }
    EXPECT_GT(max_angular_speed, 1.0f);
    EXPECT_LT(max_angular_speed, 1.5f);
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

    for (unsigned int i = 0; i < path_parameters.num_elements - 1; i++)
    {
        EXPECT_FLOAT_EQ(velocity_trajectory.angular_velocity[i],
                        position_trajectory.angular_speed[i]);
        EXPECT_FLOAT_EQ(
            velocity_trajectory.x_velocity[i],
            position_trajectory.linear_speed[i] * static_cast<float>(directions[i].x()));
        EXPECT_FLOAT_EQ(
            velocity_trajectory.y_velocity[i],
            position_trajectory.linear_speed[i] * static_cast<float>(directions[i].y()));
        EXPECT_NEAR(velocity_trajectory.time_profile[i],
                    position_trajectory.time_profile[i], 0.00001);
    }
    EXPECT_FLOAT_EQ(
        velocity_trajectory.angular_velocity[path_parameters.num_elements - 1], 0);
}

// A robot with very little linear motion required, rotates just under 180 deg.
TEST_F(TrajectoryPlannerTest,
       test_angular_limiting_the_trajectory_with_large_linear_slowdown)
{
    // This test was created to address a bug that was found where the corrected linear
    // speeds accumulated instead of decreasing to the final value. It was caused by the
    // speed correction only adding speed vs taking it away when the speed is decreasing.
    FirmwareRobotPathParameters_t path_parameters = {
        .path                = {.x = {.coefficients = {0.0f, 0.0f, 0.05f, 4.6f}},
                 .y = {.coefficients = {0.0f, 0.0f, 0.0f, -3.09f}}},
        .orientation_profile = {.coefficients = {0.0f, 0.0f, 2.55f, 0.0f}},
        .t_start             = 0.0f,
        .t_end               = 1.0f,
        .num_elements        = 10,
        .max_allowable_linear_acceleration  = 1.5f,
        .max_allowable_linear_speed         = 1.0f,
        .max_allowable_angular_acceleration = 5.0f,
        .max_allowable_angular_speed        = 6.28f,
        .initial_linear_speed               = 0.0f,
        .final_linear_speed                 = 0.0f};

    PositionTrajectory_t position_trajectory;
    auto status =
        app_trajectory_planner_generateConstantParameterizationPositionTrajectory(
            path_parameters, &position_trajectory);
    EXPECT_EQ(status, OK);

    // Check that the speeds meet the start and end conditions and did not accumulate
    EXPECT_NEAR(position_trajectory.linear_speed[0], path_parameters.initial_linear_speed,
                0.0001f);
    EXPECT_NEAR(position_trajectory.linear_speed[9], path_parameters.final_linear_speed,
                0.0001f);
    EXPECT_NEAR(position_trajectory.angular_speed[0], 0.0f, 0.0001f);
    EXPECT_NEAR(position_trajectory.angular_speed[9], 0.0f, 0.0001f);
}
