#include "firmware/app/control/trajectory_planner.h"

#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdbool.h>

#include "assert.h"
#include "firmware/shared/math/polynomial_1d.h"
#include "firmware/shared/math/polynomial_2d.h"
#include "firmware/shared/math/tbots_math.h"
#include "firmware/shared/physics.h"

TrajectoryPlannerGenerationStatus_t
app_trajectory_planner_interpolateConstantPeriodPositionTrajectory(
    PositionTrajectory_t* constant_period_trajectory,
    PositionTrajectory_t* variable_time_trajectory, const float interpolation_period)
{
    constant_period_trajectory->path_parameters =
        variable_time_trajectory->path_parameters;

    // The first point is the same for each trajectory
    app_trajectory_planner_copyPositionTrajectoryElement(
        &constant_period_trajectory->trajectory_elements[0],
        &variable_time_trajectory->trajectory_elements[0]);

    // Keep track of the current time we are searching for in the constant
    // parameterization trajectory
    unsigned int time_periods = 1;
    float trajectory_time     = interpolation_period * time_periods;

    // Loop until we find a value JUST larger than the expected
    // Check the element prior and perform linear interpolation
    for (unsigned int i = 1; i < variable_time_trajectory->path_parameters.num_segments;
         i++)
    {
        while (variable_time_trajectory->trajectory_elements[i].time > trajectory_time &&
               time_periods < TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS)
        {
            PositionTrajectoryElement_t* current_element =
                &variable_time_trajectory->trajectory_elements[i];
            PositionTrajectoryElement_t* previous_element =
                &variable_time_trajectory->trajectory_elements[i - 1];

            // Perform a linear interpolation
            const float delta_time = current_element->time - previous_element->time;
            const float delta_x =
                current_element->position.x - previous_element->position.x;
            const float delta_y =
                current_element->position.y - previous_element->position.y;
            const float delta_theta =
                current_element->orientation - previous_element->orientation;
            const float delta_omega =
                current_element->angular_speed - previous_element->angular_speed;
            const float delta_linear_speed =
                current_element->linear_speed - previous_element->linear_speed;

            const float interpolated_x = shared_tbots_math_linearInterpolation(
                previous_element->time, previous_element->position.x,
                current_element->time, current_element->position.x, trajectory_time);
            const float interpolated_y = shared_tbots_math_linearInterpolation(
                previous_element->time, previous_element->position.y,
                current_element->time, current_element->position.y, trajectory_time);
            const float interpolated_theta = shared_tbots_math_linearInterpolation(
                previous_element->time, previous_element->orientation,
                current_element->time, current_element->orientation, trajectory_time);
            const float interpolated_angular_speed =
                shared_tbots_math_linearInterpolation(
                    previous_element->time, previous_element->angular_speed,
                    current_element->time, current_element->angular_speed,
                    trajectory_time);
            const float interpolated_linear_speed = shared_tbots_math_linearInterpolation(
                previous_element->time, previous_element->linear_speed,
                current_element->time, current_element->linear_speed, trajectory_time);

            constant_period_trajectory->trajectory_elements[time_periods].position.x =
                interpolated_x;
            constant_period_trajectory->trajectory_elements[time_periods].position.y =
                interpolated_y;
            constant_period_trajectory->trajectory_elements[time_periods].orientation =
                interpolated_theta;
            constant_period_trajectory->trajectory_elements[time_periods].linear_speed =
                interpolated_linear_speed;
            constant_period_trajectory->trajectory_elements[time_periods].angular_speed =
                interpolated_angular_speed;
            constant_period_trajectory->trajectory_elements[time_periods].time =
                trajectory_time;

            // Step forwards one interpolation period
            time_periods++;
            trajectory_time = interpolation_period * time_periods;
        }
    }

    if (time_periods == TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS)
    {
        return INTERPOLATION_ELEMENT_MAXED_OUT;
    }

    const unsigned int last_element_index =
        variable_time_trajectory->path_parameters.num_segments - 1;

    // The last element of both trajectories are also identical
    app_trajectory_planner_copyPositionTrajectoryElement(
        &constant_period_trajectory->trajectory_elements[time_periods],
        &variable_time_trajectory->trajectory_elements[last_element_index]);

    // Set the new number of time periods
    constant_period_trajectory->path_parameters.num_segments = ++time_periods;
    return OK;
}

TrajectoryPlannerGenerationStatus_t
app_trajectory_planner_createForwardsContinuousLinearSpeedProfile(
    PositionTrajectory_t* trajectory, float* max_allowable_speed_profile,
    TrajectorySegment_t* segment_lengths)
{
    const unsigned int num_segments = trajectory->path_parameters.num_segments;
    const float max_allowable_speed =
        trajectory->path_parameters.max_allowable_linear_speed;
    const float max_allowable_acceleration =
        trajectory->path_parameters.max_allowable_linear_acceleration;

    trajectory->trajectory_elements[0].linear_speed =
        trajectory->path_parameters.initial_linear_speed;

    for (unsigned int i = 1; i < num_segments; i++)
    {
        const float speed        = trajectory->trajectory_elements[i - 1].linear_speed;
        const float displacement = segment_lengths[i - 1].linear_segment_length;

        // Vf = sqrt( Vi^2 + 2*constant_segment_length*max_acceleration)
        float temp_vel =
            shared_physics_calculateFinalSpeedFromDisplacementInitialSpeedAndAcceleration(
                speed, displacement, max_allowable_acceleration);

        // Pick  the lowest of the maximum the available speeds
        const float lowest_speed =
            fmin(max_allowable_speed_profile[i], max_allowable_speed);
        const float speed_to_set = fmin(lowest_speed, temp_vel);

        trajectory->trajectory_elements[i].linear_speed = speed_to_set;
    }

    if (trajectory->trajectory_elements[num_segments - 1].linear_speed <
        trajectory->path_parameters.final_linear_speed)
    {
        return FINAL_VELOCITY_TOO_HIGH;
    }

    trajectory->trajectory_elements[num_segments - 1].linear_speed =
        trajectory->path_parameters.final_linear_speed;
    return OK;
}

TrajectoryPlannerGenerationStatus_t
app_trajectory_planner_createForwardsContinuousSpeedProfile(
    float speeds[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS],
    float segment_lengths[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS],
    float max_allowable_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS],
    const float max_allowable_acceleration, const float initial_speed,
    const float final_speed, const unsigned int num_segments)
{
    // Set the initial speed
    speeds[0] = initial_speed;

    for (unsigned int i = 1; i < num_segments; i++)
    {
        // 'i' represents the next speed, where [i-1] is the current speed
        const float speed        = speeds[i - 1];
        const float displacement = segment_lengths[i - 1];

        // Vf = sqrt( Vi^2 + 2*constant_segment_length*max_acceleration)
        float temp_vel =
            shared_physics_calculateFinalSpeedFromDisplacementInitialSpeedAndAcceleration(
                speed, displacement, max_allowable_acceleration);

        // Pick  the lowest of the maximum the available speeds
        const float lowest_speed = fmin(max_allowable_speed_profile[i], temp_vel);

        speeds[i] = lowest_speed;
    }

    if (speeds[num_segments - 1] < final_speed)
    {
        return FINAL_VELOCITY_TOO_HIGH;
    }

    speeds[num_segments - 1] = final_speed;
    return OK;
}

TrajectoryPlannerGenerationStatus_t
app_trajectory_planner_modifySpeedsToBackwardsContinuous(
    float speeds[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS],
    float segment_lengths[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS],
    const float max_allowable_acceleration, const float initial_speed,
    const unsigned int num_segments)
{
    for (unsigned int i = num_segments - 1; i > 0; i--)
    {
        const float current_speed  = speeds[i];
        const float previous_speed = speeds[i - 1];
        const float segment_length = segment_lengths[i - 1];

        // Vf = sqrt( Vi^2 + 2*constant_segment_length*max_acceleration)
        float temp_speed =
            shared_physics_calculateFinalSpeedFromDisplacementInitialSpeedAndAcceleration(
                current_speed, segment_length, max_allowable_acceleration);

        // If the velocity at [i-1] is larger than it physically possible to decelerate
        // from, pull the speed at [i-1] lower
        if (previous_speed > temp_speed)
        {
            speeds[i - 1] = temp_speed;
        }
    }

    // Check that we are able to decelerate fast enough that the initial velocity allows
    // for the path to be followed
    // Note: The initial speed of an angular profile is always assumed to be zero
    TrajectoryPlannerGenerationStatus_t status = OK;

    if (speeds[0] < initial_speed)
    {
        speeds[0] = initial_speed;
        status    = INITIAL_VELOCITY_TOO_HIGH;
    }

    return status;
}

void app_trajectory_planner_createForwardsContinuousAngularSpeedProfile(
    PositionTrajectory_t* trajectory, TrajectorySegment_t* segment_lengths)
{
    const unsigned int num_segments = trajectory->path_parameters.num_segments;
    const float max_allowable_speed =
        trajectory->path_parameters.max_allowable_angular_speed;
    const float max_allowable_acceleration =
        trajectory->path_parameters.max_allowable_angular_acceleration;

    trajectory->trajectory_elements[0].angular_speed = 0;

    for (unsigned int i = 1; i < num_segments; i++)
    {
        const float speed        = trajectory->trajectory_elements[i - 1].angular_speed;
        const float displacement = segment_lengths[i - 1].angular_segment_length;

        // Vf = sqrt( Vi^2 + 2*constant_segment_length*max_acceleration)
        float temp_vel =
            shared_physics_calculateFinalSpeedFromDisplacementInitialSpeedAndAcceleration(
                speed, displacement, max_allowable_acceleration);

        // Pick  the lowest of the maximum the available speeds
        const float speed_to_set = fmin(max_allowable_speed, temp_vel);

        trajectory->trajectory_elements[i].angular_speed = speed_to_set;
    }

    // The final velocity of an angular profile is assumed zero
    trajectory->trajectory_elements[num_segments - 1].angular_speed = 0;
}

TrajectoryPlannerGenerationStatus_t
app_trajectory_planner_modifyTrajectoryToBackwardsContinuous(
    PositionTrajectory_t* trajectory, TrajectorySegment_t* segment_lengths)
{
    const unsigned int num_segments = trajectory->path_parameters.num_segments;

    const float max_allowable_acceleration_linear =
        trajectory->path_parameters.max_allowable_linear_acceleration;
    const float max_allowable_acceleration_angular =
        trajectory->path_parameters.max_allowable_angular_acceleration;

    for (unsigned int i = num_segments - 1; i > 0; i--)
    {
        const float current_speed_linear =
            trajectory->trajectory_elements[i].linear_speed;
        const float current_speed_angular =
            trajectory->trajectory_elements[i].angular_speed;
        const float previous_speed_linear =
            trajectory->trajectory_elements[i - 1].linear_speed;
        const float previous_speed_angular =
            trajectory->trajectory_elements[i - 1].angular_speed;
        const float segment_length_linear = segment_lengths[i - 1].linear_segment_length;
        const float segment_length_angular =
            segment_lengths[i - 1].angular_segment_length;


        // Vf = sqrt( Vi^2 + 2*constant_segment_length*max_acceleration)
        float temp_vel_linear =
            shared_physics_calculateFinalSpeedFromDisplacementInitialSpeedAndAcceleration(
                current_speed_linear, segment_length_linear,
                max_allowable_acceleration_linear);
        float temp_vel_angular =
            shared_physics_calculateFinalSpeedFromDisplacementInitialSpeedAndAcceleration(
                current_speed_angular, segment_length_angular,
                max_allowable_acceleration_angular);

        // If the velocity at [i-1] is larger than it physically possible to decelerate
        // from, pull the speed at [i-1] lower
        if (previous_speed_linear > temp_vel_linear)
        {
            trajectory->trajectory_elements[i - 1].linear_speed = temp_vel_linear;
        }
        if (previous_speed_angular > temp_vel_angular)
        {
            trajectory->trajectory_elements[i - 1].angular_speed = temp_vel_angular;
        }
    }

    // Check that we are able to decelerate fast enough that the initial velocity allows
    // for the path to be followed
    // Note: The initial speed of an angular profile is always assumed to be zero
    TrajectoryPlannerGenerationStatus_t status = OK;

    trajectory->trajectory_elements[0].angular_speed = 0.0;

    if (trajectory->trajectory_elements[0].linear_speed <
        trajectory->path_parameters.initial_linear_speed)
    {
        trajectory->trajectory_elements[0].linear_speed =
            trajectory->path_parameters.initial_linear_speed;
        status = INITIAL_VELOCITY_TOO_HIGH;
    }

    return status;
}

TrajectoryPlannerGenerationStatus_t
app_trajectory_planner_generateConstantParameterizationPositionTrajectory_2(
    PositionTrajectory_t* position_trajectory,
    FirmwareRobotPathParameters_t path_parameters)
{
    // Assign all of the path parameter data to local variables
    const unsigned int num_elements = path_parameters.num_segments;
    const float t_end               = path_parameters.t_end;
    const float t_start             = path_parameters.t_start;
    float* x_profile                = position_trajectory->x_position;
    float* y_profile                = position_trajectory->y_position;
    float* orientation_profile      = position_trajectory->orientation;
    float* angular_speed            = position_trajectory->angular_speed;
    float* linear_speed             = position_trajectory->linear_speed;
    const float max_linear_acceleration =
        path_parameters.max_allowable_linear_acceleration;
    const float max_angular_acceleration =
        path_parameters.max_allowable_angular_acceleration;
    const float max_linear_speed     = path_parameters.max_allowable_linear_speed;
    const float max_angular_speed    = path_parameters.max_allowable_angular_speed;
    const float initial_linear_speed = path_parameters.initial_linear_speed;
    const float final_linear_speed   = path_parameters.final_linear_speed;

    Polynomial1dOrder3_t theta_poly = path_parameters.orientation_profile;

    float linear_segment_lengths[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    float angular_segment_lengths[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    // Generate the states and segment lengths for each dimension
    app_trajectory_planner_generateLinearSegmentNodesAndLengths(
        t_start, t_end, path_parameters.path, linear_segment_lengths, x_profile,
        y_profile, num_elements);

    app_trajectory_planner_generateSegmentNodesAndLengths(
        t_start, t_end, theta_poly, angular_segment_lengths, orientation_profile,
        num_elements);

    // Generate the max allowable speed profile for linear and angular profile
    float max_allowable_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    app_trajectory_planner_getAbsoluteMaximumSpeedProfile(
        path_parameters.path, num_elements, t_start, t_end, max_linear_acceleration,
        max_linear_speed, max_allowable_speed_profile);

    TrajectoryPlannerGenerationStatus_t status =
        app_trajectory_planner_createForwardsContinuousSpeedProfile(
            linear_speed, linear_segment_lengths, max_allowable_speed_profile,
            max_linear_acceleration, initial_linear_speed, final_linear_speed,
            num_elements);
    if (status != OK)
    {
        return status;
    }

    // Create a 2d polynomial out of the theta profile and a polynomial of all zeros
    Polynomial2dOrder3_t theta_poly_2d = {.x = theta_poly, .y = {0, 0, 0, 0}};
    app_trajectory_planner_getAbsoluteMaximumSpeedProfile(
        theta_poly_2d, num_elements, t_start, t_end, max_angular_acceleration,
        max_angular_speed, max_allowable_speed_profile);

    // The initial and final angular velocity of a path is assumed to be zero
    const float initial_angular_speed = 0;
    const float final_angular_speed   = 0;
    // Generate the forwards continuous angular speed profile
    status = app_trajectory_planner_createForwardsContinuousSpeedProfile(
        angular_speed, angular_segment_lengths, max_allowable_speed_profile,
        max_angular_acceleration, initial_angular_speed, final_angular_speed,
        num_elements);
    if (status != OK)
    {
        return status;
    }

    // Make the speed profiles backwards continuous
    status = app_trajectory_planner_modifySpeedsToBackwardsContinuous(
        linear_speed, linear_segment_lengths, max_linear_acceleration,
        initial_linear_speed, num_elements);
    if (status != OK)
    {
        return status;
    }

    status = app_trajectory_planner_modifySpeedsToBackwardsContinuous(
        angular_speed, angular_segment_lengths, max_angular_acceleration,
        initial_angular_speed, num_elements);
    if (status != OK)
    {
        return status;
    }

    float linear_time_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    float angular_time_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    // Generate the segment-based duration of each trajectory
    app_trajectory_planner_generatePositionTrajectoryTimeProfile_2(
        linear_segment_lengths, linear_speed, linear_time_profile, num_elements);
    app_trajectory_planner_generatePositionTrajectoryTimeProfile_2(
        angular_segment_lengths, angular_speed, angular_time_profile, num_elements);


    // Calculate the time duration of the trajectory at each segment node
    app_trajectory_planner_modifySpeedsToMatchDuration(
        linear_speed, angular_speed, linear_time_profile, angular_time_profile,
        linear_segment_lengths, angular_segment_lengths,
        position_trajectory->time_profile, num_elements);

    return OK;
}

TrajectoryPlannerGenerationStatus_t
app_trajectory_planner_generateConstantParameterizationPositionTrajectory(
    PositionTrajectory_t* position_trajectory)
{
    // Generate the segments and states
    TrajectorySegment_t segment_lengths[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    app_trajectory_planner_generateStatesAndReturnSegmentLengths(position_trajectory,
                                                                 segment_lengths);

    float max_allowable_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    app_trajectory_planner_getMaxAllowableSpeedProfile(
        position_trajectory->path_parameters.path,
        position_trajectory->path_parameters.num_segments,
        position_trajectory->path_parameters.t_start,
        position_trajectory->path_parameters.t_end,
        position_trajectory->path_parameters.max_allowable_linear_acceleration,
        max_allowable_speed_profile);

    // Generate the forwards continuous angular and linear profiles
    TrajectoryPlannerGenerationStatus_t status = OK;
    status = app_trajectory_planner_createForwardsContinuousLinearSpeedProfile(
        position_trajectory, max_allowable_speed_profile, segment_lengths);
    if (status != OK)
    {
        return status;
    }

    app_trajectory_planner_createForwardsContinuousAngularSpeedProfile(
        position_trajectory, segment_lengths);

    // Modify the forwards continuous profiles to be backwards continuous
    status = app_trajectory_planner_modifyTrajectoryToBackwardsContinuous(
        position_trajectory, segment_lengths);
    if (status != OK)
    {
        return status;
    }

    // Calculate the time duration of the trajectory at each segment node
    app_trajectory_planner_generatePositionTrajectoryTimeProfile(position_trajectory,
                                                                 segment_lengths);

    return OK;
}

TrajectoryPlannerGenerationStatus_t
app_trajectory_planner_generateConstantInterpolationPeriodPositionTrajectory(
    FirmwareRobotPathParameters_t path_parameters,
    PositionTrajectory_t* constant_period_trajectory, const float interpolation_period)
{
    PositionTrajectory_t variable_time_trajectory;
    PositionTrajectoryElement_t
        variable_time_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    variable_time_trajectory.path_parameters     = path_parameters;
    variable_time_trajectory.trajectory_elements = variable_time_elements;

    // Generate the position trajectory
    TrajectoryPlannerGenerationStatus_t status =
        app_trajectory_planner_generateConstantParameterizationPositionTrajectory(
            &variable_time_trajectory);

    if (status != OK)
    {
        return status;
    }

    status = app_trajectory_planner_interpolateConstantPeriodPositionTrajectory(
        constant_period_trajectory, &variable_time_trajectory, interpolation_period);

    if (status != OK)
    {
        return status;
    }

    return OK;
}

void app_trajectory_planner_generatePositionTrajectoryTimeProfile(
    PositionTrajectory_t* trajectory, TrajectorySegment_t* segment_lengths)
{
    PositionTrajectoryElement_t* trajectory_elements = trajectory->trajectory_elements;

    // The timing of a trajectory is relative and starts with zero
    trajectory_elements[0].time = 0.0;

    // Calculate the time required for linear and rotational segment
    // then use the largest of the calculated times as the total time for the segment
    // Use the relative time difference to scale the speed (linear or angular) that is
    // 'faster' than the other to make them arrive at the same point
    for (unsigned int i = 1; i < trajectory->path_parameters.num_segments; i++)
    {
        // Initialize the variables to hold the length of time for each segment
        float angular_segment_delta_time = 0;
        float linear_segment_delta_time  = 0;

        // Check that we are not dividing by zero
        if (trajectory->trajectory_elements[i].linear_speed == 0 &&
            trajectory->trajectory_elements[i - 1].linear_speed == 0)
        {
            linear_segment_delta_time = 0;
        }
        else
        {
            linear_segment_delta_time =
                (2 * segment_lengths[i - 1].linear_segment_length) /
                (trajectory->trajectory_elements[i].linear_speed +
                 trajectory->trajectory_elements[i - 1].linear_speed);
        }

        // Check that we are not dividing by zero
        if (trajectory->trajectory_elements[i].angular_speed == 0 &&
            trajectory->trajectory_elements[i - 1].angular_speed == 0)
        {
            angular_segment_delta_time = 0;
        }
        else
        {
            angular_segment_delta_time =
                (2 * segment_lengths[i - 1].angular_segment_length) /
                (trajectory->trajectory_elements[i].angular_speed +
                 trajectory->trajectory_elements[i - 1].angular_speed);
        }

        // Now we know the time requirements for following the linear and angular path
        // profiles Pick the largest and scale the previous speed by a proportional amount
        if (angular_segment_delta_time > linear_segment_delta_time)
        {
            // In this case the linear velocity needs to be scaled down
            app_trajectory_planner_rebalanceAngularAndLinearTrajectorySegmentsForEquivalentDuration(
                trajectory, segment_lengths, angular_segment_delta_time, i, false);
            trajectory->trajectory_elements[i].time =
                trajectory->trajectory_elements[i - 1].time + angular_segment_delta_time;
        }
        else if (linear_segment_delta_time > angular_segment_delta_time)
        {
            // In this case the angular velocity needs to be scaled down
            app_trajectory_planner_rebalanceAngularAndLinearTrajectorySegmentsForEquivalentDuration(
                trajectory, segment_lengths, linear_segment_delta_time, i, true);
            trajectory->trajectory_elements[i].time =
                trajectory->trajectory_elements[i - 1].time + linear_segment_delta_time;
        }
        else
        {
            // Catch-all case that there is no change in position for both the linear and
            // angular paths
            trajectory->trajectory_elements[i].time =
                trajectory->trajectory_elements[i - 1].time + 0.0;
        }
    }
}

void app_trajectory_planner_generatePositionTrajectoryTimeProfile_2(
    float segment_lengths[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS],
    float speeds[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS],
    float trajectory_duration[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS],
    const unsigned int num_elements)
{
    // Calculate the time required to move between the first and last nodes of a
    // trajectory segment
    for (unsigned int i = 0; i < num_elements; i++)
    {
        // Delta-time over the length of the segment
        float delta_time = 0;

        // Check that we are not dividing by zero
        if (speeds[i + 1] == 0 && speeds[i] == 0)
        {
            delta_time = 0;
        }
        else
        {
            delta_time = (2 * segment_lengths[i]) / (speeds[i + 1] + speeds[i]);
        }

        // Catch-all case that there is no change in position for both the linear and
        // angular paths
        trajectory_duration[i] = delta_time;
    }
}

TrajectoryPlannerGenerationStatus_t
app_trajectory_planner_rebalanceAngularAndLinearTrajectorySegmentsForEquivalentDuration(
    PositionTrajectory_t* trajectory, TrajectorySegment_t* segment_lengths,
    const float desired_segment_duration, unsigned int trajectory_index,
    bool rebalance_angular)
{
    // Initialize variables and values for the type of trajectory being modified
    float current_speed  = 0;
    float segment_length = 0;

    if (rebalance_angular)
    {
        current_speed =
            trajectory->trajectory_elements[trajectory_index - 1].angular_speed;
        segment_length = segment_lengths[trajectory_index - 1].angular_segment_length;
    }
    else
    {
        current_speed =
            trajectory->trajectory_elements[trajectory_index - 1].linear_speed;
        segment_length = segment_lengths[trajectory_index - 1].linear_segment_length;
    }

    // Calculate the new final velocity based on the initial velocity, segment length, and
    // the desired segment duration
    const float next_speed =
        (2 * segment_length / desired_segment_duration) + current_speed;

    if (rebalance_angular)
    {
        trajectory->trajectory_elements[trajectory_index].angular_speed = next_speed;
    }
    else
    {
        trajectory->trajectory_elements[trajectory_index].linear_speed = next_speed;
    }
}

void app_trajectory_planner_getMaxAllowableSpeedProfile(
    Polynomial2dOrder3_t path, const unsigned int num_elements, const float t_start,
    const float t_end, const float max_allowable_acceleration,
    float* max_allowable_speed_profile)
{
    Polynomial2dOrder2_t first_deriv = shared_polynomial2d_differentiateOrder3(path);
    Polynomial2dOrder1_t second_deriv =
        shared_polynomial2d_differentiateOrder2(first_deriv);
    const float delta_t = (t_end - t_start) / num_elements;

    for (unsigned int i = 0; i < num_elements; i++)
    {
        float t = t_start + i * delta_t;

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

void app_trajectory_planner_getAbsoluteMaximumSpeedProfile(
    Polynomial2dOrder3_t path, const unsigned int num_elements, const float t_start,
    const float t_end, const float max_allowable_acceleration, const float speed_cap,
    float* max_allowable_speed_profile)
{
    const float t_increment = (t_end - t_start) / (num_elements - 1);

    for (unsigned int i = 0; i < num_elements; i++)
    {
        const float current_t = t_start + i * t_increment;

        const float radius_of_curvature =
            shared_polynomial2d_getCurvatureAtPositionOrder3(path, current_t);

        const float max_speed = sqrt(max_allowable_acceleration * radius_of_curvature);

        max_allowable_speed_profile[i] = fmin(max_speed, speed_cap);
    }
}

void app_trajectory_planner_generateVelocityTrajectory(
    PositionTrajectory_t* position_trajectory, VelocityTrajectory_t* velocity_trajectory)
{
    PositionTrajectoryElement_t* position_elements =
        position_trajectory->trajectory_elements;
    VelocityTrajectoryElement_t* velocity_elements =
        velocity_trajectory->trajectory_elements;

    velocity_trajectory->path_parameters = position_trajectory->path_parameters;

    for (unsigned int i = 0; i < position_trajectory->path_parameters.num_segments - 1;
         i++)
    {
        const float delta_x =
            position_elements[i + 1].position.x - position_elements[i].position.x;
        const float delta_y =
            position_elements[i + 1].position.y - position_elements[i].position.y;
        const float delta_theta =
            (position_elements[i + 1].orientation - position_elements[i].orientation);

        // The unit vector of the direction is 1/magnitide(vector) *vector
        const float vector_magnitude_inverse =
            (1 / sqrt(pow(delta_x, 2) + pow(delta_y, 2)));

        const float x_velocity_component =
            delta_x == 0
                ? 0
                : vector_magnitude_inverse * delta_x * position_elements[i].linear_speed;
        const float y_velocity_component =
            delta_y == 0
                ? 0
                : vector_magnitude_inverse * delta_y * position_elements[i].linear_speed;


        // Use the sign of the change in orientation to calculate the direction of the
        // angular velocity
        const float angular_velocity = delta_theta > 0.0
                                           ? position_elements[i].angular_speed
                                           : -1 * position_elements[i].angular_speed;

        // Copy data into the velocity element array
        velocity_elements[i].linear_velocity.x = x_velocity_component;
        velocity_elements[i].linear_velocity.y = y_velocity_component;
        velocity_elements[i].angular_velocity  = angular_velocity;
        velocity_elements[i].time              = position_elements[i].time;
    }

    const unsigned int last_element_index =
        position_trajectory->path_parameters.num_segments - 1;
    // Assume that the velocity of the final segment is in the direction of the previous
    const float delta_x = position_elements[last_element_index].position.x -
                          position_elements[last_element_index - 1].position.x;
    const float delta_y = position_elements[last_element_index].position.y -
                          position_elements[last_element_index - 1].position.y;

    float x_velocity_component = 0;
    float y_velocity_component = 0;

    // Get the inverse magnitude of the direction vector for normalization
    const float vector_magnitude_inverse = (1 / sqrt(pow(delta_x, 2) + pow(delta_y, 2)));
    if (delta_x != 0)
    {
        x_velocity_component = vector_magnitude_inverse * delta_x *
                               position_elements[last_element_index].linear_speed;
    }
    if (delta_y != 0)
    {
        y_velocity_component = vector_magnitude_inverse * delta_y *
                               position_elements[last_element_index].linear_speed;
    }

    // Copy data into the velocity element array
    velocity_elements[last_element_index].linear_velocity.x = x_velocity_component;
    velocity_elements[last_element_index].linear_velocity.y = y_velocity_component;
    velocity_elements[last_element_index].angular_velocity =
        0.0;  // Final angular velocity is always assumed to be zero
    velocity_elements[last_element_index].time =
        position_elements[last_element_index].time;
}

TrajectoryPlannerGenerationStatus_t
app_trajectory_planner_generateConstantInterpolationVelocityTrajectory(
    FirmwareRobotPathParameters_t path_parameters, const float interpolation_period,
    VelocityTrajectory_t* velocity_trajectory)
{
    PositionTrajectoryElement_t position_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    PositionTrajectory_t position_trajectory;
    position_trajectory.trajectory_elements = position_elements;

    TrajectoryPlannerGenerationStatus_t status =
        app_trajectory_planner_generateConstantInterpolationPeriodPositionTrajectory(
            path_parameters, &position_trajectory, interpolation_period);
    if (status != OK)
    {
        return status;
    }

    // Now calculate the velocity trajectory from the position trajectory
    app_trajectory_planner_generateVelocityTrajectory(&position_trajectory,
                                                      velocity_trajectory);
    return status;
}


void app_trajectory_planner_generateStatesAndReturnSegmentLengths(
    PositionTrajectory_t* trajectory,
    TrajectorySegment_t trajectory_segments[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS])
{
    // Check that the pre conditions are met
    assert(trajectory->path_parameters.num_segments > 2);
    assert(trajectory->path_parameters.max_allowable_linear_acceleration >= 0);
    assert(trajectory->path_parameters.max_allowable_angular_acceleration >= 0);
    assert(trajectory->path_parameters.max_allowable_linear_speed >= 0);
    assert(trajectory->path_parameters.max_allowable_angular_speed >= 0);
    assert(trajectory->path_parameters.initial_linear_speed >= 0);
    assert(trajectory->path_parameters.final_linear_speed >= 0);
    assert(trajectory->path_parameters.max_allowable_angular_acceleration >= 0);
    assert(trajectory->path_parameters.max_allowable_angular_speed >= 0);
    assert(trajectory->path_parameters.num_segments <=
           TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS);

    const float t_start = trajectory->path_parameters.t_start;

    PositionTrajectoryElement_t* trajectory_elements = trajectory->trajectory_elements;

    trajectory_elements[0].orientation = shared_polynomial1d_getValueOrder3(
        trajectory->path_parameters.orientation_profile, t_start);

    trajectory_elements[0].position =
        shared_polynomial2d_getValueOrder3(trajectory->path_parameters.path, t_start);

    const float t_segment_size =
        (trajectory->path_parameters.t_end - trajectory->path_parameters.t_start) /
        (trajectory->path_parameters.num_segments - 1);

    for (unsigned int i = 1; i < trajectory->path_parameters.num_segments; i++)
    {
        const float current_t = t_start + i * t_segment_size;
        // Grab the states at each 't' value
        trajectory_elements[i].orientation = shared_polynomial1d_getValueOrder3(
            trajectory->path_parameters.orientation_profile, current_t);
        trajectory_elements[i].position = shared_polynomial2d_getValueOrder3(
            trajectory->path_parameters.path, current_t);

        // Calculate the length of each segment and store it
        const float delta_x =
            trajectory_elements[i].position.x - trajectory_elements[i - 1].position.x;
        const float delta_y =
            trajectory_elements[i].position.y - trajectory_elements[i - 1].position.y;

        const float linear_segment_length = sqrt(pow(delta_x, 2) + pow(delta_y, 2));
        const float angular_segment_length =
            trajectory_elements[i].orientation - trajectory_elements[i - 1].orientation;

        trajectory_segments[i - 1].linear_segment_length  = linear_segment_length;
        trajectory_segments[i - 1].angular_segment_length = angular_segment_length;
    }
}

void app_trajectory_planner_generateSegmentNodesAndLengths(
    const float t_start, const float t_end, Polynomial1dOrder3_t poly,
    float segment_lengths[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS],
    float node_values[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS],
    const unsigned int num_elements)
{
    // Check that the pre conditions are met
    assert(num_elements > 2);
    assert(num_elements <= TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS);
    assert(t_start != t_end);

    node_values[0] = shared_polynomial1d_getValueOrder3(poly, t_start);

    const float t_segment_size = (t_end - t_start) / (num_elements - 1);

    for (unsigned int i = 1; i < num_elements; i++)
    {
        const float current_t = t_start + i * t_segment_size;

        // Grab the states at each 't' value
        node_values[i] = shared_polynomial1d_getValueOrder3(poly, current_t);

        // Calculate the length of each segment and store it
        const float segment_length = node_values[i] - node_values[i - 1];

        segment_lengths[i - 1] = segment_length;
    }
}

void app_trajectory_planner_generateLinearSegmentNodesAndLengths(
    const float t_start, const float t_end, Polynomial2dOrder3_t poly,
    float segment_lengths[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS],
    float x_values[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS],
    float y_values[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS], const unsigned int num_elements)
{
    // Hold into the x and y segment lengths to calculate the combined segment length
    float x_lengths[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    float y_lengths[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    app_trajectory_planner_generateSegmentNodesAndLengths(
        t_start, t_end, poly.x, x_lengths, x_values, num_elements);
    app_trajectory_planner_generateSegmentNodesAndLengths(
        t_start, t_end, poly.y, y_lengths, y_values, num_elements);

    // total length is the root sum-squared of the individual values
    for (unsigned int i = 0; i < num_elements; i++)
    {
        segment_lengths[i] = (float)sqrt(pow(x_lengths[i], 2) + pow(y_lengths[i], 2));
    }
}

void app_trajectory_planner_copyPositionTrajectoryElement(
    PositionTrajectoryElement_t* to_trajectory_element,
    PositionTrajectoryElement_t* from_trajectory_element)
{
    to_trajectory_element->time          = from_trajectory_element->time;
    to_trajectory_element->position      = from_trajectory_element->position;
    to_trajectory_element->orientation   = from_trajectory_element->orientation;
    to_trajectory_element->angular_speed = from_trajectory_element->angular_speed;
    to_trajectory_element->linear_speed  = from_trajectory_element->linear_speed;
}

TrajectoryPlannerGenerationStatus_t app_trajectory_planner_modifySpeedsToMatchDuration(
    float speeds1[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS],
    float speeds2[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS],
    float durations1[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS],
    float durations2[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS],
    float displacement1[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS],
    float displacement2[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS],
    float complete_time_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS],
    const float num_elements)
{
    // The time profile is relative to the first element, thus is starts at zero
    complete_time_profile[0] = 0.0f;

    for (unsigned int i = 0; i < num_elements - 1; i++)
    {
        // Check for the case that each segment duration isn't equal, so that they can be
        // modifed to have the same duration
        if (durations1[i] > durations2[i] && displacement1[i] != 0)
        {
            const float current_speed    = speeds2[i];
            const float displacement     = displacement2[i];
            const float desired_duration = durations1[i];
            float* final_speed_to_change = &speeds2[i + 1];

            app_trajectory_planner_modifySpeedToMatchDuration(
                current_speed, final_speed_to_change, desired_duration, displacement);
            complete_time_profile[i + 1] = complete_time_profile[i] + desired_duration;
        }
        else if (durations2[i] > durations1[i] && displacement2[i] != 0)
        {
            const float current_speed    = speeds1[i];
            const float displacement     = displacement1[i];
            const float desired_duration = durations2[i];
            float* final_speed_to_change = &speeds1[i + 1];

            app_trajectory_planner_modifySpeedToMatchDuration(
                current_speed, final_speed_to_change, desired_duration, displacement);
            complete_time_profile[i + 1] = complete_time_profile[i] + desired_duration;
        }
        else
        {
            // This means that the durations are equal and nothing needs to be changed
            complete_time_profile[i + 1] = complete_time_profile[i] + durations1[i];
            continue;
        }
    }
}

TrajectoryPlannerGenerationStatus_t
app_trajectory_planner_generateConstantInterpolationPeriodPositionTrajectory_2(
    PositionTrajectory_t* constant_period_trajectory,
    FirmwareRobotPathParameters_t* path_parameters, const float interpolation_period)
{
    PositionTrajectory_t variable_time_trajectory;
    PositionTrajectoryElement_t
        variable_time_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    variable_time_trajectory.path_parameters     = *path_parameters;
    variable_time_trajectory.trajectory_elements = variable_time_elements;

    // Generate the position trajectory
    TrajectoryPlannerGenerationStatus_t status =
        app_trajectory_planner_generateConstantParameterizationPositionTrajectory_2(
            &variable_time_trajectory, *path_parameters);

    if (status != OK)
    {
        return status;
    }

    status = app_trajectory_planner_interpolateConstantPeriodPositionTrajectory_2(
        constant_period_trajectory, &variable_time_trajectory, path_parameters,
        interpolation_period);

    if (status != OK)
    {
        return status;
    }

    return OK;
}

TrajectoryPlannerGenerationStatus_t
app_trajectory_planner_interpolateConstantPeriodPositionTrajectory_2(
    PositionTrajectory_t* constant_period_trajectory,
    PositionTrajectory_t* variable_period_trajectory,
    FirmwareRobotPathParameters_t* path_parameters, const float interpolation_period)
{
    // The first point is the same for each trajectory
    constant_period_trajectory->x_position[0] = variable_period_trajectory->x_position[0];
    constant_period_trajectory->y_position[0] = variable_period_trajectory->y_position[0];
    constant_period_trajectory->orientation[0] =
        variable_period_trajectory->orientation[0];
    constant_period_trajectory->linear_speed[0] =
        variable_period_trajectory->linear_speed[0];
    constant_period_trajectory->angular_speed[0] =
        variable_period_trajectory->angular_speed[0];
    constant_period_trajectory->time_profile[0] =
        variable_period_trajectory->time_profile[0];

    // Keep track of the current time we are searching for in the constant
    // parameterization trajectory
    unsigned int time_periods = 1;
    float trajectory_time     = interpolation_period * time_periods;

    // Loop until we find a value JUST larger than the expected
    // Check the element prior and perform linear interpolation
    for (unsigned int i = 1; i < path_parameters->num_segments; i++)
    {
        while (variable_period_trajectory->time_profile[i] > trajectory_time &&
               time_periods < TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS)
        {
            // Perform a linear interpolation
            const float delta_time = variable_period_trajectory->time_profile[i] -
                                     variable_period_trajectory->time_profile[i - 1];
            const float delta_x = variable_period_trajectory->x_position[i] -
                                  variable_period_trajectory->x_position[i - 1];
            const float delta_y = variable_period_trajectory->y_position[i] -
                                  variable_period_trajectory->y_position[i - 1];
            const float delta_theta = variable_period_trajectory->orientation[i] -
                                      variable_period_trajectory->orientation[i - 1];
            const float delta_omega = variable_period_trajectory->angular_speed[i] -
                                      variable_period_trajectory->angular_speed[i - 1];
            const float delta_linear_speed =
                variable_period_trajectory->linear_speed[i] -
                variable_period_trajectory->linear_speed[i - 1];

            const float interpolated_x = shared_tbots_math_linearInterpolation(
                variable_period_trajectory->time_profile[i - 1],
                variable_period_trajectory->x_position[i - 1],
                variable_period_trajectory->time_profile[i],
                variable_period_trajectory->x_position[i], trajectory_time);
            const float interpolated_y = shared_tbots_math_linearInterpolation(
                variable_period_trajectory->time_profile[i - 1],
                variable_period_trajectory->y_position[i - 1],
                variable_period_trajectory->time_profile[i],
                variable_period_trajectory->y_position[i], trajectory_time);
            const float interpolated_theta = shared_tbots_math_linearInterpolation(
                variable_period_trajectory->time_profile[i - 1],
                variable_period_trajectory->orientation[i - 1],
                variable_period_trajectory->time_profile[i],
                variable_period_trajectory->orientation[i], trajectory_time);
            const float interpolated_angular_speed =
                shared_tbots_math_linearInterpolation(
                    variable_period_trajectory->time_profile[i - 1],
                    variable_period_trajectory->angular_speed[i - 1],
                    variable_period_trajectory->time_profile[i],
                    variable_period_trajectory->angular_speed[i], trajectory_time);
            const float interpolated_linear_speed = shared_tbots_math_linearInterpolation(
                variable_period_trajectory->time_profile[i - 1],
                variable_period_trajectory->linear_speed[i - 1],
                variable_period_trajectory->time_profile[i],
                variable_period_trajectory->linear_speed[i], trajectory_time);

            constant_period_trajectory->x_position[time_periods]  = interpolated_x;
            constant_period_trajectory->y_position[time_periods]  = interpolated_y;
            constant_period_trajectory->orientation[time_periods] = interpolated_theta;
            constant_period_trajectory->linear_speed[time_periods] =
                interpolated_linear_speed;
            constant_period_trajectory->angular_speed[time_periods] =
                interpolated_angular_speed;
            constant_period_trajectory->angular_speed[time_periods] = trajectory_time;

            // Step forwards one interpolation period
            time_periods++;
            trajectory_time = interpolation_period * time_periods;
        }
    }

    if (time_periods == TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS)
    {
        return INTERPOLATION_ELEMENT_MAXED_OUT;
    }

    const unsigned int last_element_index = path_parameters->num_segments - 1;

    // The last element of both trajectories are also identical
    constant_period_trajectory->x_position[time_periods] =
        variable_period_trajectory->x_position[last_element_index];
    constant_period_trajectory->y_position[time_periods] =
        variable_period_trajectory->y_position[last_element_index];
    constant_period_trajectory->orientation[time_periods] =
        variable_period_trajectory->orientation[last_element_index];
    constant_period_trajectory->linear_speed[time_periods] =
        variable_period_trajectory->linear_speed[last_element_index];
    constant_period_trajectory->angular_speed[time_periods] =
        variable_period_trajectory->angular_speed[last_element_index];
    constant_period_trajectory->time_profile[time_periods] =
        variable_period_trajectory->time_profile[last_element_index];

    // Set the new number of time periods
    path_parameters->num_segments = ++time_periods;
    return OK;
}

void app_trajectory_planner_generateVelocityTrajectory_2(
    PositionTrajectory_t* position_trajectory, VelocityTrajectory_t* velocity_trajectory,
    FirmwareRobotPathParameters_t path_parameters)
{
    // Assign local variables to make code more legible
    float* x_positions           = position_trajectory->x_position;
    float* y_positions           = position_trajectory->y_position;
    float* orientations          = position_trajectory->orientation;
    float* linear_speeds         = position_trajectory->linear_speed;
    float* angular_speeds        = position_trajectory->angular_speed;
    float* position_time_profile = position_trajectory->time_profile;

    float* x_velocity            = velocity_trajectory->x_velocity;
    float* y_velocity            = velocity_trajectory->y_velocity;
    float* angular_speeds_copy   = velocity_trajectory->angular_velocity;
    float* velocity_time_profile = velocity_trajectory->time;

    for (unsigned int i = 0; i < path_parameters.num_segments - 1; i++)
    {
        const float delta_x     = x_positions[i + 1] - x_positions[i];
        const float delta_y     = y_positions[i + 1] - y_positions[i];
        const float delta_theta = (orientations[i + 1] - orientations[i]);

        // The unit vector of the direction is 1/magnitide(vector) *vector
        const float vector_magnitude_inverse =
            (float)(1 / sqrt(pow(delta_x, 2) + pow(delta_y, 2)));

        const float x_velocity_component =
            delta_x == 0 ? 0 : vector_magnitude_inverse * delta_x * linear_speeds[i];
        const float y_velocity_component =
            delta_y == 0 ? 0 : vector_magnitude_inverse * delta_y * linear_speeds[i];


        // Use the sign of the change in orientation to calculate the direction of the
        // angular velocity
        const float angular_velocity =
            delta_theta > 0.0 ? angular_speeds[i] : -1 * angular_speeds[i];

        // Copy data into the velocity element array
        x_velocity[i]            = x_velocity_component;
        y_velocity[i]            = y_velocity_component;
        angular_speeds_copy[i]   = angular_velocity;
        velocity_time_profile[i] = position_time_profile[i];
    }

    const unsigned int last_element_index = path_parameters.num_segments - 1;
    // Assume that the velocity of the final segment is in the direction of the previous
    const float delta_x =
        x_positions[last_element_index] - x_positions[last_element_index - 1];
    const float delta_y =
        y_positions[last_element_index] - y_positions[last_element_index - 1];

    float x_velocity_component = 0;
    float y_velocity_component = 0;

    // Get the inverse magnitude of the direction vector for normalization
    const float vector_magnitude_inverse =
        (float)(1 / sqrt(pow(delta_x, 2) + pow(delta_y, 2)));
    if (delta_x != 0)
    {
        x_velocity_component =
            vector_magnitude_inverse * delta_x * linear_speeds[last_element_index];
    }
    if (delta_y != 0)
    {
        y_velocity_component =
            vector_magnitude_inverse * delta_y * linear_speeds[last_element_index];
    }

    // Copy data into the velocity element array
    x_velocity[last_element_index] = x_velocity_component;
    y_velocity[last_element_index] = y_velocity_component;
    angular_speeds_copy[last_element_index] =
        0.0;  // Final angular velocity is always assumed to be zero
    velocity_time_profile[last_element_index] = position_time_profile[last_element_index];
}

TrajectoryPlannerGenerationStatus_t app_trajectory_planner_modifySpeedToMatchDuration(
    float initial_speed, float* final_speed, float duration, float displacement)
{
    // Calculate the new final speed based on the initial speed, displacement, and
    // the desired duration in time
    const float new_speed = (2 * displacement / duration) + initial_speed;
    *final_speed          = new_speed;
}