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

    static std::vector<double> getSpeedsFromTrajectory(
        FirmwareRobotPathParameters_t path_parameters)
    {
        std::vector<double> velocity;


        // Create the parmeterization to contain the desired number of segments
        CREATE_STATIC_ARC_LENGTH_PARAMETRIZATION(arc_length_parameterization,
                                                 TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS);

        float max_allowable_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

        shared_polynomial_getArcLengthParametrizationOrder3(
            path_parameters.path, path_parameters.t_start, path_parameters.t_end,
            arc_length_parameterization);

        const float arc_segment_length =
            shared_polynomial2d_getTotalArcLength(arc_length_parameterization) /
            path_parameters.num_segments;


        app_trajectory_planner_getMaxAllowableSpeedProfile(
            max_allowable_speed_profile, path_parameters.path,
            path_parameters.num_segments, arc_length_parameterization, arc_segment_length,
            path_parameters.max_allowable_linear_acceleration);

        float velocity_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

        velocity_profile[0] = path_parameters.initial_linear_speed;
        app_trajectory_planner_generateForwardsContinuousLinearSpeedProfile(
            path_parameters.num_segments, velocity_profile, max_allowable_speed_profile,
            arc_segment_length, path_parameters.max_allowable_linear_acceleration,
            path_parameters.max_allowable_linear_speed);

        // Check that it was physically possible for the robot to reach the final velocity
        // requested
        if (velocity_profile[path_parameters.num_segments - 1] >=
            path_parameters.final_linear_speed)
        {
            velocity_profile[path_parameters.num_segments - 1] =
                path_parameters.final_linear_speed;
        }

        app_trajectory_planner_generateBackwardsContinuousSpeedProfile(
            path_parameters.num_segments, velocity_profile, arc_segment_length,
            path_parameters.max_allowable_linear_acceleration);

        velocity_profile[0] = path_parameters.initial_linear_speed;

        for (unsigned int i = 0; i < path_parameters.num_segments; i++)
        {
            velocity.push_back(velocity_profile[i]);
        }

        return velocity;
    }

    static std::vector<double> getAccelerationsFromSpeed(std::vector<double> speeds,
                                                         PositionTrajectory_t* trajectory)
    {
        std::vector<double> acceleration;


        for (unsigned int i = 0; i < trajectory->path_parameters.num_segments - 1; i++)
        {
            const double dv = speeds[i + 1] - speeds[i];
            const double dt = trajectory->trajectory_elements[i + 1].time -
                              trajectory->trajectory_elements[i].time;
            acceleration.push_back(dv / dt);
        }

        return acceleration;
    }

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

// TEST_F(TrajectoryPlannerTest, test_forward_continuity_path_curvature_max_speed_limited)
//{
//    float max_allowable_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
//
//    const unsigned int num_segments = 10;
//
//    for (unsigned int i = 0; i < num_segments; i++)
//    {
//        max_allowable_speed_profile[i] = 1;
//    }
//
//    float velocity_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
//    const float arc_segment_length         = 1.0;
//    const float max_allowable_acceleration = 0.1;
//    const float max_allowable_speed        = 10;
//
//    app_trajectory_planner_generateForwardsContinuousLinearSpeedProfile(
//        num_segments, velocity_profile, max_allowable_speed_profile, arc_segment_length,
//        max_allowable_acceleration, max_allowable_speed);
//
//    for (unsigned int i = 0; i < num_segments; i++)
//    {
//        EXPECT_TRUE(velocity_profile[i] <= 1);
//    }
//}
//
// TEST_F(TrajectoryPlannerTest, test_forward_continuity_max_speed_limited)
//{
//    float max_allowable_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
//
//    const unsigned int num_segments = 10;
//
//    for (unsigned int i = 0; i < num_segments; i++)
//    {
//        max_allowable_speed_profile[i] = 8;
//    }
//
//    float velocity_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
//    const float arc_segment_length         = 1.0;
//    const float max_allowable_acceleration = 1.3;
//    const float max_allowable_speed        = 4;
//
//    app_trajectory_planner_generateForwardsContinuousLinearSpeedProfile(
//        num_segments, velocity_profile, max_allowable_speed_profile, arc_segment_length,
//        max_allowable_acceleration, max_allowable_speed);
//
//    for (unsigned int i = 0; i < num_segments; i++)
//    {
//        EXPECT_TRUE(velocity_profile[i] <= 4);
//    }
//}
//
// TEST_F(TrajectoryPlannerTest, test_backwards_continuity)
//{
//    float max_allowable_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
//
//    const unsigned int num_segments        = 3;
//    const float arc_segment_length         = 1.0;
//    const float max_allowable_acceleration = 1.3;
//    const float max_allowable_speed        = 15;
//
//    float velocity_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
//
//    velocity_profile[0] = 0;
//    velocity_profile[1] = velocity_profile[0] + 10;
//    velocity_profile[2] = 0;
//
//    app_trajectory_planner_generateBackwardsContinuousSpeedProfile(
//        num_segments, velocity_profile, arc_segment_length, max_allowable_acceleration);
//
//    EXPECT_EQ(velocity_profile[0], 0);
//    EXPECT_NEAR(velocity_profile[1], 1.61245155, 0.001);
//    EXPECT_EQ(velocity_profile[2], 0);
//}
//
// TEST_F(TrajectoryPlannerTest, test_backwards_continuity_to_initial_velocity)
//{
//    float max_allowable_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
//
//    const unsigned int num_segments        = 3;
//    const float arc_segment_length         = 1.0;
//    const float max_allowable_acceleration = 1.3;
//    const float max_allowable_speed        = 15;
//
//    float velocity_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
//
//    velocity_profile[0] = 40;
//    velocity_profile[1] = velocity_profile[0] + 10;
//    velocity_profile[2] = 0;
//
//    app_trajectory_planner_generateBackwardsContinuousSpeedProfile(
//        num_segments, velocity_profile, arc_segment_length, max_allowable_acceleration);
//
//    EXPECT_NEAR(velocity_profile[0], 2.28035092, 0.001);
//    EXPECT_NEAR(velocity_profile[1], 1.61245155, 0.001);
//    EXPECT_EQ(velocity_profile[2], 0);
//}
//
// TEST_F(TrajectoryPlannerTest, check_trajectory_length)
//{
//    Polynomial2dOrder3_t path = {
//        .x = {.coefficients = {2, 0, 1, 0}},
//        .y = {.coefficients = {1, 0, 1, 0}},
//
//    };
//
//    FirmwareRobotPathParameters_t path_parameters;
//    path_parameters.path    = path;
//    path_parameters.t_start = 0;
//    path_parameters.t_end   = 1;
//    path_parameters.num_segments =
//        TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS;  // For tests the number of segments is
//                                              // arbitrary, so use the maximum possible
//    path_parameters.max_allowable_linear_acceleration = 3;
//    path_parameters.max_allowable_linear_speed        = 3;
//    path_parameters.initial_linear_speed              = 0;
//    path_parameters.final_linear_speed                = 0;
//
//    PositionTrajectoryElement_t const_arc_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
//    float speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
//    PositionTrajectory_t trajectory = {.trajectory_elements  = const_arc_elements,
//                                       .path_parameters      = path_parameters,
//                                       .linear_speed_profile = speed_profile};
//
//    TrajectoryPlannerGenerationStatus_t status =
//        app_trajectory_planner_generateConstantArcLengthPositionTrajectory(&trajectory);
//    EXPECT_EQ(OK, status);
//    // Create the parmeterization to contain the desired number of segments
//    CREATE_STATIC_ARC_LENGTH_PARAMETRIZATION(arc_length_param,
//                                             TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS);
//    // Get all of the points for the arc length parameterization (Not constant arc
//    length
//    // segments)
//    shared_polynomial_getArcLengthParametrizationOrder3(
//        path_parameters.path, path_parameters.t_start, path_parameters.t_end,
//        arc_length_param);
//
//    const float arc_segment_length =
//        shared_polynomial2d_getTotalArcLength(arc_length_param);
//
//    float segment_length_sum = 0;
//
//    // Test by checking the length between points
//    // and the length of the total path
//    for (uint i = 0; i < trajectory.path_parameters.num_segments - 1; i++)
//    {
//        float length = static_cast<float>(
//            sqrt(pow(trajectory.trajectory_elements[i].position.x -
//                         trajectory.trajectory_elements[i + 1].position.x,
//                     2) +
//                 pow(trajectory.trajectory_elements[i].position.y -
//                         trajectory.trajectory_elements[i + 1].position.y,
//                     2)));
//        segment_length_sum += length;
//    }
//    EXPECT_NEAR(segment_length_sum, arc_segment_length, 0.01);
//}
//
// TEST_F(TrajectoryPlannerTest, check_end_points_match_path)
//{
//    Polynomial2dOrder3_t path = {
//        .x = {.coefficients = {2, 0, 1, 0}},
//        .y = {.coefficients = {1, 0, 1, 0}},
//
//    };
//
//    FirmwareRobotPathParameters_t path_parameters;
//    path_parameters.path    = path;
//    path_parameters.t_start = 0;
//    path_parameters.t_end   = 1;
//    path_parameters.num_segments =
//        TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS;  // For tests the number of segments is
//                                              // arbitrary, so use the maximum possible
//    path_parameters.max_allowable_linear_acceleration = 3;
//    path_parameters.max_allowable_linear_speed        = 3;
//    path_parameters.initial_linear_speed              = 0;
//    path_parameters.final_linear_speed                = 0;
//
//    PositionTrajectoryElement_t const_arc_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
//    float speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
//    PositionTrajectory_t trajectory = {.trajectory_elements  = const_arc_elements,
//                                       .path_parameters      = path_parameters,
//                                       .linear_speed_profile = speed_profile};
//
//    TrajectoryPlannerGenerationStatus_t status =
//        app_trajectory_planner_generateConstantArcLengthPositionTrajectory(&trajectory);
//    EXPECT_EQ(OK, status);
//
//    // Create the parmeterization to contain the desired number of segments
//    CREATE_STATIC_ARC_LENGTH_PARAMETRIZATION(arc_length_param,
//                                             TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS);
//    // Get all of the points for the arc length parameterization (Not constant arc
//    length
//    // segments)
//    shared_polynomial_getArcLengthParametrizationOrder3(
//        path_parameters.path, path_parameters.t_start, path_parameters.t_end,
//        arc_length_param);
//
//    EXPECT_NEAR(
//        trajectory.trajectory_elements[path_parameters.num_segments - 1].position.x,
//        shared_polynomial2d_getValueOrder3(path_parameters.path,
//        path_parameters.t_end).x, 0.01);
//    EXPECT_NEAR(
//        trajectory.trajectory_elements[path_parameters.num_segments - 1].position.y,
//        shared_polynomial2d_getValueOrder3(path_parameters.path,
//        path_parameters.t_end).y, 0.01);
//
//    EXPECT_NEAR(
//        trajectory.trajectory_elements[0].position.x,
//        shared_polynomial2d_getValueOrder3(path_parameters.path,
//        path_parameters.t_start)
//            .x,
//        0.01);
//    EXPECT_NEAR(
//        trajectory.trajectory_elements[0].position.x,
//        shared_polynomial2d_getValueOrder3(path_parameters.path,
//        path_parameters.t_start)
//            .y,
//        0.01);
//}
//
// TEST_F(TrajectoryPlannerTest,
//       dynamics_dont_exceed_maximums_straight_line_medium_acceleration)
//{
//    Polynomial2dOrder3_t path = {
//        .x = {.coefficients = {0, 0, 1, 0}},
//        .y = {.coefficients = {0, 0, 1, 0}},
//
//    };
//
//    FirmwareRobotPathParameters_t path_parameters;
//    path_parameters.path    = path;
//    path_parameters.t_start = 0;
//    path_parameters.t_end   = 3;
//    path_parameters.num_segments =
//        500;  // Choose a low fidelity path to switch up the test cases
//    path_parameters.max_allowable_linear_acceleration = 3;
//    path_parameters.max_allowable_linear_speed        = 9;
//    path_parameters.initial_linear_speed              = 2;
//    path_parameters.final_linear_speed                = 5;
//
//    PositionTrajectoryElement_t const_arc_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
//    float speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
//    PositionTrajectory_t trajectory = {.trajectory_elements  = const_arc_elements,
//                                       .path_parameters      = path_parameters,
//                                       .linear_speed_profile = speed_profile};
//
//
//    TrajectoryPlannerGenerationStatus_t status =
//        app_trajectory_planner_generateConstantArcLengthPositionTrajectory(&trajectory);
//    EXPECT_EQ(OK, status);
//
//    // Create the parmeterization to contain the desired number of segments
//    CREATE_STATIC_ARC_LENGTH_PARAMETRIZATION(arc_length_param,
//                                             TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS);
//
//    // Get all of the points for the arc length parameterization (Not constant arc
//    length
//    // segments)
//    shared_polynomial_getArcLengthParametrizationOrder3(
//        path, path_parameters.t_start, path_parameters.t_end, arc_length_param);
//
//    std::vector<double> velocity     = getSpeedsFromTrajectory(path_parameters);
//    std::vector<double> acceleration = getAccelerationsFromSpeed(velocity, &trajectory);
//
//    for (float vel : velocity)
//    {
//        EXPECT_TRUE(vel <= path_parameters.max_allowable_linear_speed &&
//                    vel >= path_parameters.initial_linear_speed);
//    }
//
//    EXPECT_NEAR(velocity.back(), path_parameters.final_linear_speed, 0.1);
//    EXPECT_FLOAT_EQ(velocity.front(), path_parameters.initial_linear_speed);
//
//    // Loop through the positive acceleration of the bang-bang profile
//    unsigned int i = 0;
//    while (acceleration[i] > path_parameters.max_allowable_linear_acceleration)
//    {
//        EXPECT_NEAR(path_parameters.max_allowable_linear_acceleration, acceleration[i],
//                    0.01);
//        i++;
//    }
//    i++;  // Skip the transition acceleration from positive to negative
//
//    // Loop through the negative acceleration portion of the bang-bang profile
//    while (acceleration[i] < -path_parameters.max_allowable_linear_acceleration)
//    {
//        EXPECT_NEAR(-path_parameters.max_allowable_linear_acceleration, acceleration[i],
//                    0.01);
//        i++;
//    }
//
//    EXPECT_NEAR(
//        trajectory.trajectory_elements[path_parameters.num_segments - 1].position.x,
//        shared_polynomial2d_getValueOrder3(path_parameters.path,
//        path_parameters.t_end).x, 0.01);
//    EXPECT_NEAR(
//        trajectory.trajectory_elements[path_parameters.num_segments - 1].position.y,
//        shared_polynomial2d_getValueOrder3(path_parameters.path,
//        path_parameters.t_end).y, 0.01);
//
//    EXPECT_NEAR(trajectory.trajectory_elements[0].position.x,
//                shared_polynomial2d_getValueOrder3(path, path_parameters.t_start).x,
//                0.01);
//    EXPECT_NEAR(trajectory.trajectory_elements[0].position.y,
//                shared_polynomial2d_getValueOrder3(path, path_parameters.t_start).y,
//                0.01);
//}
//
// TEST_F(TrajectoryPlannerTest,
//       dynamics_dont_exceed_maximums_straight_line_acceleration_high_acceleration)
//{
//    Polynomial2dOrder3_t path = {
//        .x = {.coefficients = {0, 0, 1, 0}},
//        .y = {.coefficients = {0, 0, 1, 0}},
//
//    };
//
//    FirmwareRobotPathParameters_t path_parameters;
//    path_parameters.path                              = path;
//    path_parameters.t_start                           = 0;
//    path_parameters.t_end                             = 3;
//    path_parameters.num_segments                      = 500;
//    path_parameters.max_allowable_linear_acceleration = 10;
//    path_parameters.max_allowable_linear_speed        = 9;
//    path_parameters.initial_linear_speed              = 2;
//    path_parameters.final_linear_speed                = 5;
//
//    PositionTrajectoryElement_t const_arc_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
//    float speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
//    PositionTrajectory_t trajectory = {.trajectory_elements  = const_arc_elements,
//                                       .path_parameters      = path_parameters,
//                                       .linear_speed_profile = speed_profile};
//
//
//    TrajectoryPlannerGenerationStatus_t status =
//        app_trajectory_planner_generateConstantArcLengthPositionTrajectory(&trajectory);
//    EXPECT_EQ(OK, status);
//
//    // Create the parmeterization to contain the desired number of segments
//    CREATE_STATIC_ARC_LENGTH_PARAMETRIZATION(arc_length_param,
//                                             TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS);
//
//    // Get all of the points for the arc length parameterization (Not constant arc
//    length
//    // segments)
//    shared_polynomial_getArcLengthParametrizationOrder3(
//        path, path_parameters.t_start, path_parameters.t_end, arc_length_param);
//
//    std::vector<double> velocity     = getSpeedsFromTrajectory(path_parameters);
//    std::vector<double> acceleration = getAccelerationsFromSpeed(velocity, &trajectory);
//
//    for (float vel : velocity)
//    {
//        EXPECT_TRUE(vel <= path_parameters.max_allowable_linear_speed &&
//                    vel >= path_parameters.initial_linear_speed);
//    }
//
//    EXPECT_NEAR(velocity.back(), path_parameters.final_linear_speed, 0.1);
//    EXPECT_FLOAT_EQ(velocity.front(), path_parameters.initial_linear_speed);
//
//    // Loop through the positive acceleration of the bang-bang profile
//    unsigned int i = 0;
//    while (acceleration[i] > path_parameters.max_allowable_linear_acceleration)
//    {
//        EXPECT_NEAR(path_parameters.max_allowable_linear_acceleration, acceleration[i],
//                    0.01);
//        i++;
//    }
//    i++;  // Skip the transition acceleration from positive to negative
//
//    // Loop through the negative acceleration portion of the bang-bang profile
//    while (acceleration[i] < -path_parameters.max_allowable_linear_acceleration)
//    {
//        EXPECT_NEAR(-path_parameters.max_allowable_linear_acceleration, acceleration[i],
//                    0.01);
//        i++;
//    }
//
//    EXPECT_NEAR(
//        trajectory.trajectory_elements[path_parameters.num_segments - 1].position.x,
//        shared_polynomial2d_getValueOrder3(path_parameters.path,
//        path_parameters.t_end).x, 0.01);
//    EXPECT_NEAR(
//        trajectory.trajectory_elements[path_parameters.num_segments - 1].position.y,
//        shared_polynomial2d_getValueOrder3(path_parameters.path,
//        path_parameters.t_end).y, 0.01);
//
//    EXPECT_NEAR(trajectory.trajectory_elements[0].position.x,
//                shared_polynomial2d_getValueOrder3(path, path_parameters.t_start).x,
//                0.01);
//    EXPECT_NEAR(trajectory.trajectory_elements[0].position.y,
//                shared_polynomial2d_getValueOrder3(path, path_parameters.t_start).y,
//                0.01);
//}
//
// TEST_F(TrajectoryPlannerTest,
//       dynamics_dont_exceed_maximums_straight_line_max_vel_not_reached)
//{
//    Polynomial2dOrder3_t path = {
//        .x = {.coefficients = {0, 0, 1, 0}},
//        .y = {.coefficients = {0, 0, 1, 0}},
//
//    };
//
//    FirmwareRobotPathParameters_t path_parameters;
//    path_parameters.path                              = path;
//    path_parameters.t_start                           = 0;
//    path_parameters.t_end                             = 3;
//    path_parameters.num_segments                      = 500;
//    path_parameters.max_allowable_linear_acceleration = 1;
//    path_parameters.max_allowable_linear_speed        = 30;
//    path_parameters.initial_linear_speed              = 0;
//    path_parameters.final_linear_speed                = 0;
//
//    PositionTrajectoryElement_t const_arc_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
//    float speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
//    PositionTrajectory_t trajectory = {.trajectory_elements  = const_arc_elements,
//                                       .path_parameters      = path_parameters,
//                                       .linear_speed_profile = speed_profile};
//
//
//    TrajectoryPlannerGenerationStatus_t status =
//        app_trajectory_planner_generateConstantArcLengthPositionTrajectory(&trajectory);
//    EXPECT_EQ(OK, status);
//
//    // Create the parmeterization to contain the desired number of segments
//    CREATE_STATIC_ARC_LENGTH_PARAMETRIZATION(arc_length_param,
//                                             TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS);
//
//    // Get all of the points for the arc length parameterization (Not constant arc
//    length
//    // segments)
//    shared_polynomial_getArcLengthParametrizationOrder3(
//        path, path_parameters.t_start, path_parameters.t_end, arc_length_param);
//
//    std::vector<double> velocity     = getSpeedsFromTrajectory(path_parameters);
//    std::vector<double> acceleration = getAccelerationsFromSpeed(velocity, &trajectory);
//
//    for (float vel : velocity)
//    {
//        EXPECT_TRUE(vel <= path_parameters.max_allowable_linear_speed &&
//                    vel >= path_parameters.initial_linear_speed);
//    }
//
//    EXPECT_NEAR(velocity.back(), path_parameters.final_linear_speed, 0.1);
//    EXPECT_FLOAT_EQ(velocity.front(), path_parameters.initial_linear_speed);
//
//    // Loop through the positive acceleration of the bang-bang profile
//    unsigned int i = 0;
//    while (acceleration[i] > path_parameters.max_allowable_linear_acceleration)
//    {
//        EXPECT_NEAR(path_parameters.max_allowable_linear_acceleration, acceleration[i],
//                    0.01);
//        i++;
//    }
//    i++;  // Skip the transition acceleration from positive to negative
//
//    // Loop through the negative acceleration portion of the bang-bang profile
//    while (acceleration[i] < -path_parameters.max_allowable_linear_acceleration)
//    {
//        EXPECT_NEAR(-path_parameters.max_allowable_linear_acceleration, acceleration[i],
//                    0.01);
//        i++;
//    }
//
//    EXPECT_NEAR(
//        trajectory.trajectory_elements[path_parameters.num_segments - 1].position.x,
//        shared_polynomial2d_getValueOrder3(path_parameters.path,
//        path_parameters.t_end).x, 0.01);
//    EXPECT_NEAR(
//        trajectory.trajectory_elements[path_parameters.num_segments - 1].position.y,
//        shared_polynomial2d_getValueOrder3(path_parameters.path,
//        path_parameters.t_end).y, 0.01);
//
//    EXPECT_NEAR(trajectory.trajectory_elements[0].position.x,
//                shared_polynomial2d_getValueOrder3(path, path_parameters.t_start).x,
//                0.01);
//    EXPECT_NEAR(trajectory.trajectory_elements[0].position.y,
//                shared_polynomial2d_getValueOrder3(path, path_parameters.t_start).y,
//                0.01);
//}
//
// TEST_F(TrajectoryPlannerTest, check_trajectory_path_length_reverse_parameterization)
//{
//    Polynomial2dOrder3_t path = {
//        .x = {.coefficients = {0, 0, 1, 0}},
//        .y = {.coefficients = {0, 0, 1, 0}},
//
//    };
//
//    FirmwareRobotPathParameters_t path_parameters;
//    path_parameters.path         = path;
//    path_parameters.t_start      = 1;
//    path_parameters.t_end        = 0;
//    path_parameters.num_segments = TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS - 1;
//    path_parameters.max_allowable_linear_acceleration = 3;
//    path_parameters.max_allowable_linear_speed        = 3;
//    path_parameters.initial_linear_speed              = 0;
//    path_parameters.final_linear_speed                = 0;
//
//    PositionTrajectoryElement_t const_arc_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
//    float speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
//    PositionTrajectory_t trajectory = {.trajectory_elements  = const_arc_elements,
//                                       .path_parameters      = path_parameters,
//                                       .linear_speed_profile = speed_profile};
//
//
//    TrajectoryPlannerGenerationStatus_t status =
//        app_trajectory_planner_generateConstantArcLengthPositionTrajectory(&trajectory);
//    EXPECT_EQ(OK, status);
//
//    // Create the parmeterization to contain the desired number of segments
//    CREATE_STATIC_ARC_LENGTH_PARAMETRIZATION(arc_length_param,
//                                             TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS);
//
//    // Get all of the points for the arc length parameterization (Not constant arc
//    length
//    // segments)
//    shared_polynomial_getArcLengthParametrizationOrder3(
//        path, path_parameters.t_end, path_parameters.t_start, arc_length_param);
//
//    const float arc_segment_length =
//        arc_length_param.arc_length_values[arc_length_param.num_values - 1];
//
//    float segment_length_sum = 0;
//
//    // Test by checking the length between points
//    // and the length of the total path
//    for (uint i = 0; i < trajectory.path_parameters.num_segments - 1; i++)
//    {
//        float length =
//            static_cast<float>(sqrt(pow(trajectory.trajectory_elements[i + 1].position.x
//            -
//                                            trajectory.trajectory_elements[i].position.x,
//                                        2) +
//                                    pow(trajectory.trajectory_elements[i + 1].position.y
//                                    -
//                                            trajectory.trajectory_elements[i].position.y,
//                                        2)));
//        segment_length_sum += length;
//    }
//    EXPECT_NEAR(segment_length_sum, arc_segment_length, 0.01);
//}
//
// TEST_F(TrajectoryPlannerTest,
//       check_trajectory_end_points_match_path_reverse_parameterization)
//{
//    Polynomial2dOrder3_t path = {
//        .x = {.coefficients = {0, 0, 1, 0}},
//        .y = {.coefficients = {0, 0, 1, 0}},
//
//    };
//
//    FirmwareRobotPathParameters_t path_parameters;
//    path_parameters.path         = path;
//    path_parameters.t_start      = 1;
//    path_parameters.t_end        = 0;
//    path_parameters.num_segments = TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS - 1;
//    path_parameters.max_allowable_linear_acceleration = 3;
//    path_parameters.max_allowable_linear_speed        = 3;
//    path_parameters.initial_linear_speed              = 0;
//    path_parameters.final_linear_speed                = 0;
//
//    PositionTrajectoryElement_t const_arc_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
//    float speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
//    PositionTrajectory_t trajectory = {.trajectory_elements  = const_arc_elements,
//                                       .path_parameters      = path_parameters,
//                                       .linear_speed_profile = speed_profile};
//
//
//    TrajectoryPlannerGenerationStatus_t status =
//        app_trajectory_planner_generateConstantArcLengthPositionTrajectory(&trajectory);
//    EXPECT_EQ(OK, status);
//
//    // Create the parmeterization to contain the desired number of segments
//    CREATE_STATIC_ARC_LENGTH_PARAMETRIZATION(arc_length_param,
//                                             TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS);
//
//    // Get all of the points for the arc length parameterization (Not constant arc
//    length
//    // segments)
//    shared_polynomial_getArcLengthParametrizationOrder3(
//        path, path_parameters.t_end, path_parameters.t_start, arc_length_param);
//
//    EXPECT_NEAR(
//        trajectory.trajectory_elements[trajectory.path_parameters.num_segments - 1]
//            .position.x,
//        shared_polynomial2d_getValueOrder3(path, path_parameters.t_end).x, 0.01);
//    EXPECT_NEAR(
//        trajectory.trajectory_elements[trajectory.path_parameters.num_segments - 1]
//            .position.y,
//        shared_polynomial2d_getValueOrder3(path, path_parameters.t_end).y, 0.01);
//
//    EXPECT_NEAR(trajectory.trajectory_elements[0].position.x,
//                shared_polynomial2d_getValueOrder3(path, path_parameters.t_start).x,
//                0.01);
//    EXPECT_NEAR(trajectory.trajectory_elements[0].position.y,
//                shared_polynomial2d_getValueOrder3(path, path_parameters.t_start).y,
//                0.01);
//}
//
// TEST_F(TrajectoryPlannerTest, check_trajectory_speed_profile_reverse_parameterization)
//{
//    Polynomial2dOrder3_t path = {
//        .x = {.coefficients = {0, 0, 1, 0}},
//        .y = {.coefficients = {0, 0, 1, 0}},
//
//    };
//
//    FirmwareRobotPathParameters_t path_parameters;
//    path_parameters.path         = path;
//    path_parameters.t_start      = 0;
//    path_parameters.t_end        = 1;
//    path_parameters.num_segments = TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS - 1;
//    path_parameters.max_allowable_linear_acceleration = 3;
//    path_parameters.max_allowable_linear_speed        = 3;
//    path_parameters.initial_linear_speed              = 0;
//    path_parameters.final_linear_speed                = 0;
//
//    PositionTrajectoryElement_t const_arc_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
//    float speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
//    PositionTrajectory_t trajectory = {.trajectory_elements  = const_arc_elements,
//                                       .path_parameters      = path_parameters,
//                                       .linear_speed_profile = speed_profile};
//
//    float forwards_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
//
//    TrajectoryPlannerGenerationStatus_t status =
//        app_trajectory_planner_generateConstantArcLengthPositionTrajectory(&trajectory);
//    EXPECT_EQ(OK, status);
//
//    // Copy the 'forwards' velocity profile (as if t1 < t2)
//    for (unsigned int i = 0; i < path_parameters.num_segments; i++)
//    {
//        forwards_speed_profile[i] = trajectory.linear_speed_profile[i];
//    }
//
//    // Reverse the parameterization
//    float temp                         = trajectory.path_parameters.t_start;
//    trajectory.path_parameters.t_start = trajectory.path_parameters.t_end;
//    trajectory.path_parameters.t_end   = temp;
//
//    status =
//        app_trajectory_planner_generateConstantArcLengthPositionTrajectory(&trajectory);
//    EXPECT_EQ(OK, status);
//
//    for (unsigned int i = 0; i < trajectory.path_parameters.num_segments; i++)
//    {
//        EXPECT_EQ(forwards_speed_profile[i],
//                  trajectory.linear_speed_profile[path_parameters.num_segments - 1 -
//                  i]);
//    }
//}
//
// TEST_F(TrajectoryPlannerTest, dynamics_dont_exceed_maximums_curved_path)
//{
//    Polynomial2dOrder3_t path = {
//        .x = {.coefficients = {0, 2, 1, 0}},
//        .y = {.coefficients = {3, 0, 1, 0}},
//
//    };
//    FirmwareRobotPathParameters_t path_parameters;
//    path_parameters.path         = path;
//    path_parameters.t_start      = -1;
//    path_parameters.t_end        = 1;
//    path_parameters.num_segments = 500;  // Use a low segment density for variety
//    path_parameters.max_allowable_linear_acceleration = 5;
//    path_parameters.max_allowable_linear_speed        = 5;
//    path_parameters.initial_linear_speed              = 1.87;
//    path_parameters.final_linear_speed                = 5;
//
//    PositionTrajectoryElement_t const_arc_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
//    float speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
//    PositionTrajectory_t trajectory = {.trajectory_elements  = const_arc_elements,
//                                       .path_parameters      = path_parameters,
//                                       .linear_speed_profile = speed_profile};
//
//    TrajectoryPlannerGenerationStatus_t status =
//        app_trajectory_planner_generateConstantArcLengthPositionTrajectory(&trajectory);
//    EXPECT_EQ(OK, status);
//
//    // Create the parmeterization to contain the desired number of segments
//    CREATE_STATIC_ARC_LENGTH_PARAMETRIZATION(arc_length_param,
//                                             TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS);
//
//    // Get all of the points for the arc length parameterization (Not constant arc
//    length
//    // segments)
//    shared_polynomial_getArcLengthParametrizationOrder3(
//        path, path_parameters.t_start, path_parameters.t_end, arc_length_param);
//
//    float velocities[trajectory.path_parameters.num_segments];
//
//    std::vector<double> velocity     = getSpeedsFromTrajectory(path_parameters);
//    std::vector<double> acceleration = getAccelerationsFromSpeed(velocity, &trajectory);
//
//    for (float vel : velocity)
//    {
//        EXPECT_TRUE(vel <= path_parameters.max_allowable_linear_speed);
//    }
//
//    EXPECT_NEAR(velocity.back(), path_parameters.final_linear_speed, 0.1);
//
//    EXPECT_FLOAT_EQ(velocity.front(), path_parameters.initial_linear_speed);
//
//    unsigned int i = 0;
//    // Acceleration phase coming up to a curve that must be slowed down for
//    while (fabs(acceleration[i] - path_parameters.max_allowable_linear_acceleration) <
//           0.1)
//    {
//        EXPECT_NEAR(acceleration[i], path_parameters.max_allowable_linear_acceleration,
//                    0.001);
//        i++;
//    }
//    i++;  // Skip the transition acceleration from positive to negative
//
//    // Slowing down for a curve
//    while (fabs(acceleration[i] + path_parameters.max_allowable_linear_acceleration) <
//           0.1)
//    {
//        EXPECT_NEAR(acceleration[i], -path_parameters.max_allowable_linear_acceleration,
//                    0.001);
//        i++;
//    }
//
//    // Skip the acceleration ramp-up phase where the acceleration is related to the
//    // curvature This part is skipped because it is dependent on path curvature and
//    cannot
//    // be compared to the maximum acceleration specified by the path parameters
//    while (fabs(fabs(acceleration[i]) -
//                path_parameters.max_allowable_linear_acceleration) >= 0.01)
//    {
//        i++;
//    }
//
//    // Check the final acceleration phase up to steady state
//    while (fabs(acceleration[i] - path_parameters.max_allowable_linear_acceleration) <
//           0.1)
//    {
//        EXPECT_NEAR(acceleration[i], path_parameters.max_allowable_linear_acceleration,
//                    0.001);
//        i++;
//    }
//    i++;  // Skip transition acceleration going to steady state (zero)
//
//    while (i < path_parameters.num_segments)
//    {
//        EXPECT_NEAR(acceleration[i], 0, 0.001);
//        i++;
//    }
//
//    EXPECT_NEAR(
//        trajectory.trajectory_elements[trajectory.path_parameters.num_segments - 1]
//            .position.x,
//        shared_polynomial2d_getValueOrder3(path, path_parameters.t_end).x, 0.025);
//    EXPECT_NEAR(
//        trajectory.trajectory_elements[trajectory.path_parameters.num_segments - 1]
//            .position.y,
//        shared_polynomial2d_getValueOrder3(path, path_parameters.t_end).y, 0.025);
//
//    EXPECT_NEAR(trajectory.trajectory_elements[0].position.x,
//                shared_polynomial2d_getValueOrder3(path, path_parameters.t_start).x,
//                0.025);
//    EXPECT_NEAR(trajectory.trajectory_elements[0].position.y,
//                shared_polynomial2d_getValueOrder3(path, path_parameters.t_start).y,
//                0.025);
//}
//
// TEST_F(TrajectoryPlannerTest, test_get_constant_time_interpolation_straight_line)
//{
//    Polynomial2dOrder3_t path = {
//        .x = {.coefficients = {0, 0, 1, 0}},
//        .y = {.coefficients = {0, 0, 1, 0}},
//
//    };
//
//    FirmwareRobotPathParameters_t path_parameters;
//    path_parameters.path                              = path;
//    path_parameters.t_start                           = 0;
//    path_parameters.t_end                             = 1;
//    path_parameters.num_segments                      = 500;
//    path_parameters.max_allowable_linear_acceleration = 3;
//    path_parameters.max_allowable_linear_speed        = 3;
//    path_parameters.initial_linear_speed              = 0;
//    path_parameters.final_linear_speed                = 0;
//
//    PositionTrajectoryElement_t const_arc_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
//    float speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
//
//    PositionTrajectory_t trajectory = {.trajectory_elements  = const_arc_elements,
//                                       .path_parameters      = path_parameters,
//                                       .linear_speed_profile = speed_profile};
//
//    TrajectoryPlannerGenerationStatus_t status =
//        app_trajectory_planner_generateConstantArcLengthPositionTrajectory(&trajectory);
//    EXPECT_EQ(OK, status);
//
//    // Create the parameterization to contain the desired number of segments
//    CREATE_STATIC_ARC_LENGTH_PARAMETRIZATION(arc_length_param,
//                                             TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS);
//
//    // Get all of the points for the arc length parameterization (Not constant arc
//    length
//    // segments)
//    shared_polynomial_getArcLengthParametrizationOrder3(
//        path, path_parameters.t_start, path_parameters.t_end, arc_length_param);
//
//    PositionTrajectory_t const_interp_trajectory;
//    PositionTrajectoryElement_t
//        const_interp_traj_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
//    float const_interp_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
//
//    const_interp_trajectory.linear_speed_profile = const_interp_speed_profile;
//    const_interp_trajectory.trajectory_elements  = const_interp_traj_elements;
//
//    // Calculate the constant-interpolation period equivalent of the trajectory
//    status = app_trajectory_planner_interpolateConstantPeriodPositionTrajectory(
//        &const_interp_trajectory, &trajectory, 0.001);
//    EXPECT_EQ(OK, status);
//
//    EXPECT_NEAR(
//        trajectory.trajectory_elements[trajectory.path_parameters.num_segments - 1]
//            .position.x,
//        const_interp_trajectory
//            .trajectory_elements[const_interp_trajectory.path_parameters.num_segments -
//            1] .position.x,
//        0.001);
//    EXPECT_NEAR(
//        trajectory.trajectory_elements[trajectory.path_parameters.num_segments - 1]
//            .position.y,
//        const_interp_trajectory
//            .trajectory_elements[const_interp_trajectory.path_parameters.num_segments -
//            1] .position.y,
//        0.001);
//    EXPECT_NEAR(
//        trajectory.trajectory_elements[trajectory.path_parameters.num_segments -
//        1].time, const_interp_trajectory
//            .trajectory_elements[const_interp_trajectory.path_parameters.num_segments -
//            1] .time,
//        0.001);
//
//    EXPECT_NEAR(trajectory.trajectory_elements[0].position.x,
//                const_interp_trajectory.trajectory_elements[0].position.x, 0.001);
//    EXPECT_NEAR(trajectory.trajectory_elements[0].position.y,
//                const_interp_trajectory.trajectory_elements[0].position.x, 0.001);
//    EXPECT_NEAR(trajectory.trajectory_elements[0].time,
//                const_interp_trajectory.trajectory_elements[0].time, 0.001);
//}
//
// TEST_F(TrajectoryPlannerTest, test_get_constant_time_interpolation_curved_line)
//{
//    Polynomial2dOrder3_t path = {
//        .x = {.coefficients = {2, 0, 1, 0}},
//        .y = {.coefficients = {1, 0, 1, 0}},
//
//    };
//
//    FirmwareRobotPathParameters_t path_parameters;
//    path_parameters.path                              = path;
//    path_parameters.t_start                           = 0;
//    path_parameters.t_end                             = 1;
//    path_parameters.num_segments                      = 500;
//    path_parameters.max_allowable_linear_acceleration = 3;
//    path_parameters.max_allowable_linear_speed        = 3;
//    path_parameters.initial_linear_speed              = 0;
//    path_parameters.final_linear_speed                = 0;
//
//    PositionTrajectoryElement_t const_arc_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
//    float speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
//    PositionTrajectory_t trajectory = {.trajectory_elements  = const_arc_elements,
//                                       .path_parameters      = path_parameters,
//                                       .linear_speed_profile = speed_profile};
//
//
//    TrajectoryPlannerGenerationStatus_t status =
//        app_trajectory_planner_generateConstantArcLengthPositionTrajectory(&trajectory);
//    EXPECT_EQ(OK, status);
//
//    // Create the parameterization to contain the desired number of segments
//    CREATE_STATIC_ARC_LENGTH_PARAMETRIZATION(arc_length_param,
//                                             TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS);
//
//    // Get all of the points for the arc length parameterization (Not constant arc
//    length
//    // segments)
//    shared_polynomial_getArcLengthParametrizationOrder3(
//        path, path_parameters.t_start, path_parameters.t_end, arc_length_param);
//
//    PositionTrajectory_t const_interp_trajectory;
//
//    PositionTrajectoryElement_t
//        const_interp_traj_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
//
//    float const_interp_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
//
//    const_interp_trajectory.linear_speed_profile = const_interp_speed_profile;
//    const_interp_trajectory.trajectory_elements  = const_interp_traj_elements;
//
//    // Calculate the constant-interpolation period equivalent of the trajectory
//    status = app_trajectory_planner_interpolateConstantPeriodPositionTrajectory(
//        &const_interp_trajectory, &trajectory, 0.001);
//
//    EXPECT_EQ(OK, status);
//
//    EXPECT_NEAR(
//        trajectory.trajectory_elements[trajectory.path_parameters.num_segments - 1]
//            .position.x,
//        const_interp_trajectory
//            .trajectory_elements[const_interp_trajectory.path_parameters.num_segments -
//            1] .position.x,
//        0.001);
//    EXPECT_NEAR(
//        trajectory.trajectory_elements[trajectory.path_parameters.num_segments - 1]
//            .position.y,
//        const_interp_trajectory
//            .trajectory_elements[const_interp_trajectory.path_parameters.num_segments -
//            1] .position.y,
//        0.001);
//    EXPECT_NEAR(
//        trajectory.trajectory_elements[trajectory.path_parameters.num_segments -
//        1].time, const_interp_trajectory
//            .trajectory_elements[const_interp_trajectory.path_parameters.num_segments -
//            1] .time,
//        0.001);
//
//    EXPECT_NEAR(trajectory.trajectory_elements[0].position.x,
//                const_interp_trajectory.trajectory_elements[0].position.x, 0.001);
//    EXPECT_NEAR(trajectory.trajectory_elements[0].position.y,
//                const_interp_trajectory.trajectory_elements[0].position.x, 0.001);
//    EXPECT_NEAR(trajectory.trajectory_elements[0].time,
//                const_interp_trajectory.trajectory_elements[0].time, 0.001);
//}
//
//
//// This test generates a scenario where there is not enough elements in the
//// TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS specified array as there path is quite long and
///the / maximum speed of the path is low. This means that most (likely all) arc-length
///segments / require multiple interpolation periods to traverse, and therefore requires
///more space / than the constant arc_length trajectory that is alread 1 element away from
//// TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS
// TEST_F(TrajectoryPlannerTest, test_get_constant_time_interpolation_too_many_elements)
//{
//    Polynomial2dOrder3_t path = {
//        .x = {.coefficients = {2, 0, 1, 0}},
//        .y = {.coefficients = {1, 0, 1, 0}},
//
//    };
//
//    FirmwareRobotPathParameters_t path_parameters;
//    path_parameters.path         = path;
//    path_parameters.t_start      = 0;
//    path_parameters.t_end        = 2;
//    path_parameters.num_segments = TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS - 1;
//    path_parameters.max_allowable_linear_acceleration = 10;
//    path_parameters.max_allowable_linear_speed        = 0.1;
//    path_parameters.initial_linear_speed              = 0;
//    path_parameters.final_linear_speed                = 0;
//
//    PositionTrajectoryElement_t const_arc_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
//    float speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
//    PositionTrajectory_t trajectory = {.trajectory_elements  = const_arc_elements,
//                                       .path_parameters      = path_parameters,
//                                       .linear_speed_profile = speed_profile};
//
//
//    TrajectoryPlannerGenerationStatus_t status =
//        app_trajectory_planner_generateConstantArcLengthPositionTrajectory(&trajectory);
//    EXPECT_EQ(OK, status);
//
//    // Create the parameterization to contain the desired number of segments
//    CREATE_STATIC_ARC_LENGTH_PARAMETRIZATION(arc_length_param,
//                                             TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS);
//
//    // Get all of the points for the arc length parameterization (Not constant arc
//    length
//    // segments)
//    shared_polynomial_getArcLengthParametrizationOrder3(
//        path, path_parameters.t_start, path_parameters.t_end, arc_length_param);
//
//    PositionTrajectory_t const_interp_trajectory;
//
//    PositionTrajectoryElement_t
//        const_interp_traj_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
//
//    float const_interp_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
//
//    const_interp_trajectory.linear_speed_profile = const_interp_speed_profile;
//    const_interp_trajectory.trajectory_elements  = const_interp_traj_elements;
//
//    // Calculate the constant-interpolation period equivalent of the trajectory
//    status = app_trajectory_planner_interpolateConstantPeriodPositionTrajectory(
//        &const_interp_trajectory, &trajectory, 0.001);
//
//    EXPECT_EQ(INTERPOLATION_ELEMENT_MAXED_OUT, status);
//}
//
// TEST_F(TrajectoryPlannerTest, test_assert_cannot_reach_final_velocity)
//{
//    Polynomial2dOrder3_t path = {
//        .x = {.coefficients = {0, 0, 1, 0}},
//        .y = {.coefficients = {0, 0, 1, 0}},
//
//    };
//
//    FirmwareRobotPathParameters_t path_parameters;
//    path_parameters.path         = path;
//    path_parameters.t_start      = 0;
//    path_parameters.t_end        = 1;
//    path_parameters.num_segments = TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS - 1;
//    path_parameters.max_allowable_linear_acceleration = 3;
//    path_parameters.max_allowable_linear_speed        = 2000;
//    path_parameters.initial_linear_speed              = 0;
//    path_parameters.final_linear_speed                = 3000;
//
//    PositionTrajectoryElement_t const_arc_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
//    float speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
//    PositionTrajectory_t trajectory = {.trajectory_elements  = const_arc_elements,
//                                       .path_parameters      = path_parameters,
//                                       .linear_speed_profile = speed_profile};
//
//
//    TrajectoryPlannerGenerationStatus_t status =
//        app_trajectory_planner_generateConstantArcLengthPositionTrajectory(&trajectory);
//
//    EXPECT_EQ(FINAL_VELOCITY_TOO_HIGH, status);
//}
//
//// Set the initial velocity to a speed that is so high the robot cannot possible slow
///down / in time to follow the path
// TEST_F(TrajectoryPlannerTest, test_assert_initial_velocity_too_high)
//{
//    Polynomial2dOrder3_t path = {
//        .x = {.coefficients = {0, 0, 1, 0}},
//        .y = {.coefficients = {0, 0, 1, 0}},
//
//    };
//
//    FirmwareRobotPathParameters_t path_parameters;
//    path_parameters.path         = path;
//    path_parameters.t_start      = 0;
//    path_parameters.t_end        = 3;
//    path_parameters.num_segments = TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS;
//    path_parameters.max_allowable_linear_acceleration = 3;
//    path_parameters.max_allowable_linear_speed        = 50;
//    path_parameters.initial_linear_speed              = 20;
//    path_parameters.final_linear_speed                = 0;
//
//    PositionTrajectoryElement_t const_arc_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
//    float speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
//    PositionTrajectory_t trajectory = {.trajectory_elements  = const_arc_elements,
//                                       .path_parameters      = path_parameters,
//                                       .linear_speed_profile = speed_profile};
//
//
//    TrajectoryPlannerGenerationStatus_t status =
//        app_trajectory_planner_generateConstantArcLengthPositionTrajectory(&trajectory);
//
//    EXPECT_EQ(INITIAL_VELOCITY_TOO_HIGH, status);
//}
//
// TEST_F(TrajectoryPlannerTest, velocity_trajectory_straight_line_high_acceleration)
//
//{
//    Polynomial2dOrder3_t path = {
//        .x = {.coefficients = {0, 0, 1, 0}},
//        .y = {.coefficients = {0, 0, 1, 0}},
//
//    };
//
//    FirmwareRobotPathParameters_t path_parameters;
//    path_parameters.path                              = path;
//    path_parameters.t_start                           = 0;
//    path_parameters.t_end                             = 3;
//    path_parameters.num_segments                      = 1000;
//    path_parameters.max_allowable_linear_acceleration = 10;
//    path_parameters.max_allowable_linear_speed        = 9;
//    path_parameters.initial_linear_speed              = 2;
//    path_parameters.final_linear_speed                = 5;
//
//    PositionTrajectoryElement_t const_arc_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
//    float speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
//    PositionTrajectory_t trajectory = {.trajectory_elements  = const_arc_elements,
//                                       .path_parameters      = path_parameters,
//                                       .linear_speed_profile = speed_profile};
//
//
//    TrajectoryPlannerGenerationStatus_t status =
//        app_trajectory_planner_generateConstantArcLengthPositionTrajectory(&trajectory);
//
//    EXPECT_EQ(OK, status);
//
//
//    VelocityTrajectory_t velocity_trajectory;
//    VelocityTrajectoryElement_t velocity_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
//    velocity_trajectory.trajectory_elements = velocity_elements;
//
//    app_trajectory_planner_generateVelocityTrajectory(&trajectory,
//    &velocity_trajectory);
//
//    // Run test class function to get all the unit vectors for the velocity at each
//    point
//    // on the trajectory
//    std::vector<Vector> direction_unit_vectors =
//        getDirectionVectorsFromPositionTrajectory(&trajectory);
//
//    // Check that every velocity has the correct magnitude
//    for (unsigned int i = 0; i < velocity_trajectory.path_parameters.num_segments; i++)
//    {
//        const float element_speed =
//            sqrt(pow(velocity_trajectory.trajectory_elements[i].linear_velocity.x, 2) +
//                 pow(velocity_trajectory.trajectory_elements[i].linear_velocity.y, 2));
//        EXPECT_NEAR(trajectory.linear_speed_profile[i], element_speed, 0.001);
//
//        EXPECT_NEAR(direction_unit_vectors[i].x() * trajectory.linear_speed_profile[i],
//                    velocity_trajectory.trajectory_elements[i].linear_velocity.x,
//                    0.001);
//        EXPECT_NEAR(direction_unit_vectors[i].y() * trajectory.linear_speed_profile[i],
//                    velocity_trajectory.trajectory_elements[i].linear_velocity.y,
//                    0.001);
//    }
//}
//
// TEST_F(TrajectoryPlannerTest, velocity_trajectory_parabola_path_high_acceleration)
//{
//    Polynomial2dOrder3_t path = {
//        .x = {.coefficients = {0, 0, 1, 0}},
//        .y = {.coefficients = {0, 2, 0, 0}},
//
//    };
//
//    FirmwareRobotPathParameters_t path_parameters;
//    path_parameters.path                              = path;
//    path_parameters.t_start                           = -1;
//    path_parameters.t_end                             = 1;
//    path_parameters.num_segments                      = 1000;
//    path_parameters.max_allowable_linear_acceleration = 10;
//    path_parameters.max_allowable_linear_speed        = 5;
//    path_parameters.initial_linear_speed              = 0;
//    path_parameters.final_linear_speed                = 0;
//
//    PositionTrajectoryElement_t const_arc_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
//    float speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
//    PositionTrajectory_t trajectory = {.trajectory_elements  = const_arc_elements,
//                                       .path_parameters      = path_parameters,
//                                       .linear_speed_profile = speed_profile};
//
//
//    TrajectoryPlannerGenerationStatus_t status =
//        app_trajectory_planner_generateConstantArcLengthPositionTrajectory(&trajectory);
//
//    EXPECT_EQ(OK, status);
//
//
//    VelocityTrajectory_t velocity_trajectory;
//    VelocityTrajectoryElement_t velocity_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
//    velocity_trajectory.trajectory_elements = velocity_elements;
//
//    app_trajectory_planner_generateVelocityTrajectory(&trajectory,
//    &velocity_trajectory);
//
//    // Run test class function to get all the unit vectors for the velocity at each
//    point
//    // on the trajectory
//    std::vector<Vector> direction_unit_vectors =
//        getDirectionVectorsFromPositionTrajectory(&trajectory);
//
//    // Check that every velocity has the correct magnitude
//    for (unsigned int i = 0; i < velocity_trajectory.path_parameters.num_segments; i++)
//    {
//        const float element_speed =
//            sqrt(pow(velocity_trajectory.trajectory_elements[i].linear_velocity.x, 2) +
//                 pow(velocity_trajectory.trajectory_elements[i].linear_velocity.y, 2));
//        EXPECT_FLOAT_EQ(trajectory.linear_speed_profile[i], element_speed);
//
//        EXPECT_FLOAT_EQ(
//            direction_unit_vectors[i].x() * trajectory.linear_speed_profile[i],
//            velocity_trajectory.trajectory_elements[i].linear_velocity.x);
//        EXPECT_FLOAT_EQ(
//            direction_unit_vectors[i].y() * trajectory.linear_speed_profile[i],
//            velocity_trajectory.trajectory_elements[i].linear_velocity.y);
//    }
//}
//
// TEST_F(TrajectoryPlannerTest, velocity_trajectory_curved_path_low_acceleration)
//{
//    Polynomial2dOrder3_t path = {
//        .x = {.coefficients = {3, 0, 1, 0}},
//        .y = {.coefficients = {0, 2, 0, 0}},
//
//    };
//
//    FirmwareRobotPathParameters_t path_parameters;
//    path_parameters.path                              = path;
//    path_parameters.t_start                           = -1;
//    path_parameters.t_end                             = 1;
//    path_parameters.num_segments                      = 1000;
//    path_parameters.max_allowable_linear_acceleration = 3;
//    path_parameters.max_allowable_linear_speed        = 5;
//    path_parameters.initial_linear_speed              = 2;
//    path_parameters.final_linear_speed                = 1;
//
//    PositionTrajectoryElement_t const_arc_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
//    float speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
//    PositionTrajectory_t trajectory = {.trajectory_elements  = const_arc_elements,
//                                       .path_parameters      = path_parameters,
//                                       .linear_speed_profile = speed_profile};
//
//
//    TrajectoryPlannerGenerationStatus_t status =
//        app_trajectory_planner_generateConstantArcLengthPositionTrajectory(&trajectory);
//
//    EXPECT_EQ(OK, status);
//
//
//    VelocityTrajectory_t velocity_trajectory;
//    VelocityTrajectoryElement_t velocity_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
//    velocity_trajectory.trajectory_elements = velocity_elements;
//
//    app_trajectory_planner_generateVelocityTrajectory(&trajectory,
//    &velocity_trajectory);
//
//    // Run test class function to get all the unit vectors for the velocity at each
//    point
//    // on the trajectory
//    std::vector<Vector> direction_unit_vectors =
//        getDirectionVectorsFromPositionTrajectory(&trajectory);
//
//    // Check that every velocity has the correct magnitude
//    for (unsigned int i = 0; i < velocity_trajectory.path_parameters.num_segments; i++)
//    {
//        const float element_speed =
//            sqrt(pow(velocity_trajectory.trajectory_elements[i].linear_velocity.x, 2) +
//                 pow(velocity_trajectory.trajectory_elements[i].linear_velocity.y, 2));
//        EXPECT_FLOAT_EQ(trajectory.linear_speed_profile[i], element_speed);
//
//        EXPECT_FLOAT_EQ(
//            direction_unit_vectors[i].x() * trajectory.linear_speed_profile[i],
//            velocity_trajectory.trajectory_elements[i].linear_velocity.x);
//        EXPECT_FLOAT_EQ(
//            direction_unit_vectors[i].y() * trajectory.linear_speed_profile[i],
//            velocity_trajectory.trajectory_elements[i].linear_velocity.y);
//    }
//}

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

    const float delta_t =
        (path_parameters.t_end - path_parameters.t_start) / path_parameters.num_segments;

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
        app_trajectory_planner_generateBackwardsContinuousLinearSpeedProfile(
            &trajectory, segment_lengths);
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
        app_trajectory_planner_generateBackwardsContinuousLinearSpeedProfile(
            &trajectory, segment_lengths);
    EXPECT_EQ(status, OK);

    EXPECT_NEAR(trajectory.trajectory_elements[0].linear_speed, sqrt(7), 0.0001);
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
        app_trajectory_planner_generateBackwardsContinuousLinearSpeedProfile(
            &trajectory, segment_lengths);
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
        app_trajectory_planner_generateBackwardsContinuousAngularSpeedProfile(
            &trajectory, segment_lengths);
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
        app_trajectory_planner_generateBackwardsContinuousAngularSpeedProfile(
            &trajectory, segment_lengths);
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
    EXPECT_NEAR(trajectory.trajectory_elements[1].linear_speed, 0.5318, 0.0001);
    EXPECT_NEAR(trajectory.trajectory_elements[2].linear_speed, 0.7521, 0.0001);
    EXPECT_NEAR(trajectory.trajectory_elements[3].linear_speed, 0.9211, 0.0001);
    EXPECT_NEAR(trajectory.trajectory_elements[4].linear_speed, 1.0, 0.0001);

    EXPECT_NEAR(trajectory.trajectory_elements[0].angular_speed, 0, 0.0001);
    EXPECT_NEAR(trajectory.trajectory_elements[1].angular_speed, 0.4472, 0.0001);
    EXPECT_NEAR(trajectory.trajectory_elements[2].angular_speed, 0.6324, 0.0001);
    EXPECT_NEAR(trajectory.trajectory_elements[3].angular_speed, 0.7745, 0.0001);
    EXPECT_NEAR(trajectory.trajectory_elements[4].angular_speed, 0.89442, 0.0001);

    // Check that the path speed is limited by the maximum value set in the path
    // parameters
    for (unsigned int i = 5; i < trajectory.path_parameters.num_segments - 2; i++)
    {
        EXPECT_FLOAT_EQ(trajectory.trajectory_elements[i].linear_speed,
                        path_parameters.max_allowable_linear_speed);
        EXPECT_FLOAT_EQ(trajectory.trajectory_elements[i].angular_speed,
                        path_parameters.max_allowable_angular_speed);
    }

    status = app_trajectory_planner_generateBackwardsContinuousLinearSpeedProfile(
        &trajectory, segment_lengths);
    EXPECT_EQ(status, OK);
    status = app_trajectory_planner_generateBackwardsContinuousAngularSpeedProfile(
        &trajectory, segment_lengths);
    EXPECT_EQ(status, OK);

    // Check that the decceleration period is correct
    // Check that the values of the acceleration period are correct
    EXPECT_NEAR(
        trajectory.trajectory_elements[path_parameters.num_segments - 5].linear_speed,
        1 - 0, 0.0001);
    EXPECT_NEAR(
        trajectory.trajectory_elements[path_parameters.num_segments - 4].linear_speed,
        0.9211, 0.0001);
    EXPECT_NEAR(
        trajectory.trajectory_elements[path_parameters.num_segments - 3].linear_speed,
        0.7521, 0.0001);
    EXPECT_NEAR(
        trajectory.trajectory_elements[path_parameters.num_segments - 2].linear_speed,
        0.5318, 0.0001);
    EXPECT_NEAR(
        trajectory.trajectory_elements[path_parameters.num_segments - 1].linear_speed,
        path_parameters.final_linear_speed, 0.0001);


    EXPECT_NEAR(
        trajectory.trajectory_elements[path_parameters.num_segments - 5].angular_speed,
        0.8944, 0.0001);
    EXPECT_NEAR(
        trajectory.trajectory_elements[path_parameters.num_segments - 4].angular_speed,
        0.7745, 0.0001);
    EXPECT_NEAR(
        trajectory.trajectory_elements[path_parameters.num_segments - 3].angular_speed,
        0.6324, 0.0001);
    EXPECT_NEAR(
        trajectory.trajectory_elements[path_parameters.num_segments - 2].angular_speed,
        0.4472, 0.0001);
    EXPECT_NEAR(
        trajectory.trajectory_elements[path_parameters.num_segments - 1].angular_speed, 0,
        0.0001);
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
    path_parameters.initial_linear_speed               = 0;
    path_parameters.final_linear_speed                 = 0;
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

    status = app_trajectory_planner_generateBackwardsContinuousLinearSpeedProfile(
        &trajectory, segment_lengths);
    EXPECT_EQ(status, OK);
    status = app_trajectory_planner_generateBackwardsContinuousAngularSpeedProfile(
        &trajectory, segment_lengths);
    EXPECT_EQ(status, OK);

    app_trajectory_planner_generatePositionTrajectoryTimeProfile_2(&trajectory,
                                                                   segment_lengths);

    std::cout << "hello world";
}