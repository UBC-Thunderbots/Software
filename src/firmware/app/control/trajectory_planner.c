#include "firmware/app/control/trajectory_planner.h"

#include <assert.h>
#include <stdbool.h>

#include "assert.h"
#include "firmware/shared/math/polynomial_1d.h"
#include "firmware/shared/math/polynomial_2d.h"
#include "math.h"

TrajectoryPlannerGenerationStatus
app_trajectory_planner_generateConstantArcLengthSegmentation(
    FirmwareRobotPathParameters_t path_parameters,
    Trajectory_t trajectory[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS])
{
    float max_allowable_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    float velocity_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];


    // Set the internal path parameter variables
    TrajectoryElement_t* traj_elements = trajectory->trajectory_elements;

    float final_speed                      = path_parameters.final_speed;
    float initial_speed                    = path_parameters.initial_speed;
    const float max_allowable_acceleration = path_parameters.max_allowable_acceleration;
    const float max_allowable_speed        = path_parameters.max_allowable_speed;
    const unsigned int num_segments        = path_parameters.num_segments;
    const Polynomial2dOrder3_t path        = path_parameters.path;
    float t_end                            = path_parameters.t_end;
    float t_start                          = path_parameters.t_start;

    TrajectoryPlannerGenerationStatus generation_status = OK;


    trajectory->num_elements = path_parameters.num_segments;

    // Variable used to flag if the path is moving "backwards" along the input path
    bool reverse_parameterization = false;

    // Check that the pre conditions are met
    assert(num_segments > 2);
    assert(max_allowable_acceleration > 0);
    assert(max_allowable_speed > 0);
    assert(initial_speed >= 0);
    assert(final_speed >= 0);

    // Check for the parameterization direction
    // If the path is traversed in reverse, then flip all components to forwards (to be
    // reversed again in the end)
    if (t_end < t_start)
    {
        reverse_parameterization = true;

        // Reverse the direction (Polynomial library can only handle forwards direction)
        float temp = t_start;
        t_start    = t_end;
        t_end      = temp;

        // Reverse the final and initial speeds
        temp          = initial_speed;
        initial_speed = final_speed;
        final_speed   = temp;
    }

    // Create the parmeterization to contain the desired number of segments
    CREATE_STATIC_ARC_LENGTH_PARAMETRIZATION(arc_length_parameterization,
                                             TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS);

    // Get all of the points for the arc length parameterization (Not constant arc length
    // segments)
    shared_polynomial_getArcLengthParametrizationOrder3(path, t_start, t_end,
                                                        arc_length_parameterization);

    const float arc_segment_length =
        app_trajectory_planner_getTotalArcLength(arc_length_parameterization) /
        num_segments;

    // Now use numerical interpolation to get constant arc length segments for the
    // parameterization
    app_trajectory_planner_generateConstArclengthTrajectoryPositions(
        traj_elements, path, num_segments, arc_length_parameterization,
        arc_segment_length);

    // Generate the profile for the maximum physically valid velocity at each point on the
    // profile
    app_trajectory_planner_getMaxAllowableSpeedProfile(
        max_allowable_speed_profile, path, num_segments, arc_length_parameterization,
        arc_segment_length, max_allowable_acceleration);

    // First point on the velocity profile is always the current speed
    velocity_profile[0] = initial_speed;

    // Loop through the path forwards to ensure that the maximum velocity limit defined by
    // the curvature is not breached by constant max acceleration of the robot (if it is,
    // pull down the acceleration)
    app_trajectory_planner_generateForwardsContinuousVelocityProfile(
        num_segments, velocity_profile, max_allowable_speed_profile, arc_segment_length,
        max_allowable_acceleration, max_allowable_speed);

    // Check that it was physically possible for the robot to reach the final velocity
    // requested
    if (velocity_profile[num_segments - 1] >= final_speed)
    {
        velocity_profile[num_segments - 1] = final_speed;
    }
    else
    {
        generation_status = finalVelocityTooHigh;
    }

    // Now check backwards continuity. This is done to guarantee the robot can deccelerate
    // in time to not breach the maximum allowable speed defined by the path curvature
    app_trajectory_planner_generateBackwardsContinuousVelocityProfile(
        num_segments, velocity_profile, arc_segment_length, max_allowable_acceleration);

    // Check that it was physically possible for the robot to stay on the path given the
    // initial velocity
    if (velocity_profile[0] >= initial_speed)
    {
        velocity_profile[0] = initial_speed;
    }
    else
    {
        generation_status = initialVelocityTooHigh;
    }


    // Now that the velocity profile has been defined we can calculate the time profile of
    // the trajectory
    app_trajectory_planner_generateTimeProfile(traj_elements, num_segments,
                                               arc_segment_length, velocity_profile);

    // If the parameterization ended up being reversed, we need to flip all the values
    // back.
    // TODO: Remove the 'reverse parameterization' hack #1322
    if (reverse_parameterization == true)
    {
        app_trajectory_planner_reverseTrajectoryDirection(traj_elements, num_segments);
    }

    return generation_status;
}


void app_trajectory_planner_interpolateConstantTimeTrajectorySegmentation(
    Trajectory_t constant_period_trajectory[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS],
    Trajectory_t variable_time_trajectory[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS],
    const float interpolation_period)
{
    // The first point is the same for each trajectory
    constant_period_trajectory->trajectory_elements[0].time =
        variable_time_trajectory->trajectory_elements[0].time;
    constant_period_trajectory->trajectory_elements[0].position =
        variable_time_trajectory->trajectory_elements[0].position;

    // Keep track of the current time we are searching for in the constant arclength
    // trajectory
    unsigned int time_periods = 1;
    float trajectory_time     = interpolation_period * time_periods;

    // Loop until we find a value JUST larger than the expected
    // Check the element prior and perform linear interpolation
    for (unsigned int i = 1; i < variable_time_trajectory->num_elements; i++)
    {
        while (variable_time_trajectory->trajectory_elements[i].time >= trajectory_time)
        {
            // Perform a linear interpolation
            const float delta_time =
                variable_time_trajectory->trajectory_elements[i].time -
                variable_time_trajectory->trajectory_elements[i - 1].time;
            const float delta_x =
                variable_time_trajectory->trajectory_elements[i].position.x -
                variable_time_trajectory->trajectory_elements[i - 1].position.x;
            const float delta_y =
                variable_time_trajectory->trajectory_elements[i].position.y -
                variable_time_trajectory->trajectory_elements[i - 1].position.y;

            const float slope_x = delta_x / delta_time;
            const float slope_y = delta_y / delta_time;

            const float interpolated_x =
                slope_x * (trajectory_time -
                           variable_time_trajectory->trajectory_elements[i - 1].time) +
                variable_time_trajectory->trajectory_elements[i - 1].position.x;
            const float interpolated_y =
                slope_y * (trajectory_time -
                           variable_time_trajectory->trajectory_elements[i - 1].time) +
                variable_time_trajectory->trajectory_elements[i - 1].position.y;

            constant_period_trajectory->trajectory_elements[time_periods].position.x =
                interpolated_x;
            constant_period_trajectory->trajectory_elements[time_periods].position.y =
                interpolated_y;
            constant_period_trajectory->trajectory_elements[time_periods].time =
                trajectory_time;

            time_periods++;
            trajectory_time = interpolation_period * time_periods;
        }
    }

    constant_period_trajectory->num_elements = time_periods;
}


float app_trajectory_planner_getTotalArcLength(
    ArcLengthParametrization_t arc_length_paramameterization)
{
    return arc_length_paramameterization
        .arc_length_values[arc_length_paramameterization.num_values - 1];
}

static void app_trajectory_planner_generateConstArclengthTrajectoryPositions(
    TrajectoryElement_t traj_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS],
    Polynomial2dOrder3_t path, const unsigned int num_elements,
    ArcLengthParametrization_t arc_length_parameterization,
    const float arc_segment_length)
{
    for (unsigned int i = 0; i < num_elements; i++)
    {
        // Get the 't' value corresponding to the current arc length (to be used for
        // further computing)
        float t = shared_polynomial2d_getTValueAtArcLengthOrder3(
            path, i * arc_segment_length, arc_length_parameterization);

        // Get the X and Y position at the 't' value defined by the arc length
        traj_elements[i].position = shared_polynomial2d_getValueOrder3(path, t);
    }
}

void app_trajectory_planner_getMaxAllowableSpeedProfile(
    float max_allowable_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS],
    Polynomial2dOrder3_t path, unsigned int num_elements,
    ArcLengthParametrization_t arc_length_parameterization, float arc_segment_length,
    const float max_allowable_acceleration)
{
    Polynomial2dOrder2_t first_deriv = shared_polynomial2d_differentiateOrder3(path);
    Polynomial2dOrder1_t second_deriv =
        shared_polynomial2d_differentiateOrder2(first_deriv);

    for (unsigned int i = 0; i < num_elements; i++)
    {
        // Get the 't' value corresponding to the current arc length (to be used for
        // further computing)
        float t = shared_polynomial2d_getTValueAtArcLengthOrder3(
            path, i * arc_segment_length, arc_length_parameterization);

        // Create the polynomial representing path curvature
        //                                              1
        //                              ---------------------------------
        //                                     abs(x'y'' - y'x'')
        //        radius of curvature =      ----------------------
        //                                     (x'^2 + y'^2)^(3/2)
        //
        const float numerator =
            fabs(shared_polynomial1d_getValueOrder2(first_deriv.x, t) *
                     shared_polynomial1d_getValueOrder1(second_deriv.y, t) -
                 shared_polynomial1d_getValueOrder2(first_deriv.y, t) *
                     shared_polynomial1d_getValueOrder1(second_deriv.x, t));
        const float denominator =
            pow(pow(shared_polynomial1d_getValueOrder2(first_deriv.x, t), 2) +
                    pow(shared_polynomial1d_getValueOrder2(first_deriv.y, t), 2),
                3.0 / 2.0);

        const float radius_of_curvature = 1 / (numerator / denominator);

        max_allowable_speed_profile[i] =
            sqrt(max_allowable_acceleration * radius_of_curvature);
    }
}

void app_trajectory_planner_generateForwardsContinuousVelocityProfile(
    unsigned int num_segments,
    float velocity_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS],
    float max_allowable_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS],
    const float arc_segment_length, const float max_allowable_acceleration,
    const float max_allowable_speed)
{
    for (unsigned int i = 1; i < num_segments; i++)
    {
        // Vf = sqrt( Vi^2 + 2*constant_segment_length*max_acceleration)
        const float temp_vel =
            (float)sqrt(pow(velocity_profile[i - 1], 2) +
                        2 * arc_segment_length * max_allowable_acceleration);

        // If the new speed is greater than the max allowable, reduce it to the max
        if (temp_vel > max_allowable_speed_profile[i])
        {
            velocity_profile[i] = max_allowable_speed_profile[i];
        }
        // If the new speed is greater than the maximum limit of the robot, reduce it
        // to the limit
        else if (temp_vel >= max_allowable_speed)
        {
            velocity_profile[i] = max_allowable_speed;
        }
        else
        {
            velocity_profile[i] = temp_vel;
        }
    }
}

void app_trajectory_planner_generateBackwardsContinuousVelocityProfile(
    unsigned int num_segments,
    float forwards_continuous_velocity_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS],
    const float arc_segment_length, const float max_allowable_acceleration)
{
    for (unsigned int i = num_segments - 1; i > 0; i--)
    {
        // Vf = sqrt( Vi^2 + 2*constant_segment_length*max_acceleration)
        const float temp_vel =
            (float)sqrt(pow(forwards_continuous_velocity_profile[i], 2) +
                        2 * arc_segment_length * max_allowable_acceleration);

        if (forwards_continuous_velocity_profile[i - 1] > temp_vel)
        {
            forwards_continuous_velocity_profile[i - 1] = temp_vel;
        }
    }
}

void app_trajectory_planner_generateTimeProfile(
    TrajectoryElement_t traj_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS],
    const float num_segments, const float arc_segment_length,
    float velocity_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS])
{
    traj_elements[0].time = 0.0;
    for (unsigned int i = 0; i < num_segments - 1; i++)
    {
        traj_elements[i + 1].time =
            traj_elements[i].time +
            (2 * arc_segment_length) / (velocity_profile[i] + velocity_profile[i + 1]);
    }
}

void app_trajectory_planner_reverseTrajectoryDirection(
    TrajectoryElement_t forwards[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS],
    unsigned int num_segments)
{
    TrajectoryElement_t reverse[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    const float path_duration = forwards[num_segments - 1].time;

    for (unsigned int i = 0; i < num_segments; i++)
    {
        // Reverse the positions
        reverse[(num_segments - 1) - i].position = forwards[i].position;
        reverse[(num_segments - 1) - i].time     = path_duration - forwards[i].time;
    }
    // Copy reverse element array back
    for (unsigned int i = 0; i < num_segments; i++)
    {
        forwards[i].position = reverse[i].position;
        forwards[i].time     = reverse[i].time;
    }
}