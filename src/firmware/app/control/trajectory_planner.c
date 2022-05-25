#include "firmware/app/control/trajectory_planner.h"

#include <assert.h>
#include <math.h>

#include "assert.h"
#include "firmware/app/control/trajectory_planner_private.h"
#include "firmware/shared/math/polynomial_1d.h"
#include "firmware/shared/math/polynomial_2d.h"
#include "firmware/shared/math/tbots_math.h"

TrajectoryPlannerGenerationStatus_t
app_trajectory_planner_generateConstantParameterizationPositionTrajectory(
    FirmwareRobotPathParameters_t path_parameters,
    PositionTrajectory_t* position_trajectory)
{
    // Assign all of the path parameter data to local variables
    const unsigned int num_elements = path_parameters.num_elements;
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
    app_trajectory_planner_impl_generate2dSegmentNodesAndLengths(
        t_start, t_end, path_parameters.path, num_elements, x_profile, y_profile,
        linear_segment_lengths);

    app_trajectory_planner_impl_generate1dSegmentNodesAndLengths(
        t_start, t_end, theta_poly, num_elements, orientation_profile,
        angular_segment_lengths);

    // Generate the max allowable speed profile for linear and angular profile
    float max_allowable_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    app_trajectory_planner_impl_getMaximumSpeedProfile(
        path_parameters.path, num_elements, t_start, t_end, max_linear_acceleration,
        max_linear_speed, max_allowable_speed_profile);

    TrajectoryPlannerGenerationStatus_t status =
        app_trajectory_planner_impl_createForwardsContinuousSpeedProfile(
            final_linear_speed, linear_segment_lengths, max_allowable_speed_profile,
            max_linear_acceleration, initial_linear_speed, num_elements, linear_speed);
    if (status != OK)
    {
        return status;
    }

    // Create a 2d polynomial out of the theta profile and a polynomial of all zeros
    Polynomial2dOrder3_t theta_poly_2d = {.x = theta_poly, .y = {{0, 0, 0, 0}}};
    app_trajectory_planner_impl_getMaximumSpeedProfile(
        theta_poly_2d, num_elements, t_start, t_end, max_angular_acceleration,
        max_angular_speed, max_allowable_speed_profile);

    // The initial and final angular velocity of a path is assumed to be zero
    const float initial_angular_speed = 0;
    const float final_angular_speed   = 0;
    // Generate the forwards continuous angular speed profile
    status = app_trajectory_planner_impl_createForwardsContinuousSpeedProfile(
        final_angular_speed, angular_segment_lengths, max_allowable_speed_profile,
        max_angular_acceleration, initial_angular_speed, num_elements, angular_speed);
    if (status != OK)
    {
        return status;
    }

    // Make the speed profiles backwards continuous
    status = app_trajectory_planner_impl_modifySpeedsToBeBackwardsContinuous(
        initial_linear_speed, linear_segment_lengths, max_linear_acceleration,
        num_elements, linear_speed);
    if (status != OK)
    {
        return status;
    }

    status = app_trajectory_planner_impl_modifySpeedsToBeBackwardsContinuous(
        initial_angular_speed, angular_segment_lengths, max_angular_acceleration,
        num_elements, angular_speed);
    if (status != OK)
    {
        return status;
    }

    float linear_time_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    float angular_time_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    // Generate the segment-based duration of each trajectory
    app_trajectory_planner_impl_generatePositionTrajectoryTimeProfile(
        linear_segment_lengths, linear_speed, num_elements, linear_time_profile);
    app_trajectory_planner_impl_generatePositionTrajectoryTimeProfile(
        angular_segment_lengths, angular_speed, num_elements, angular_time_profile);

    // Calculate the time duration of the trajectory at each segment node
    app_trajectory_planner_impl_modifySpeedsToMatchLongestSegmentDuration(
        linear_segment_lengths, angular_segment_lengths, linear_time_profile,
        angular_time_profile, (float)num_elements, linear_speed, angular_speed,
        position_trajectory->time_profile);

    return OK;
}

TrajectoryPlannerGenerationStatus_t
app_trajectory_planner_generateConstantPeriodPositionTrajectory(
    float interpolation_period, FirmwareRobotPathParameters_t* path_parameters,
    PositionTrajectory_t* constant_period_trajectory)
{
    PositionTrajectory_t variable_time_trajectory;

    // Generate the position trajectory
    TrajectoryPlannerGenerationStatus_t status =
        app_trajectory_planner_generateConstantParameterizationPositionTrajectory(
            *path_parameters, &variable_time_trajectory);

    if (status != OK)
    {
        return status;
    }

    status = app_trajectory_planner_interpolateConstantPeriodPositionTrajectory(
        &variable_time_trajectory, interpolation_period, &path_parameters->num_elements,
        constant_period_trajectory);

    if (status != OK)
    {
        return status;
    }

    return OK;
}

TrajectoryPlannerGenerationStatus_t
app_trajectory_planner_interpolateConstantPeriodPositionTrajectory(
    PositionTrajectory_t* variable_period_trajectory, float interpolation_period,
    unsigned int* num_elements, PositionTrajectory_t* constant_period_trajectory)
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
    float trajectory_time     = interpolation_period * (float)time_periods;

    // Loop until we find a value JUST larger than the expected
    // Check the element prior and perform linear interpolation
    for (unsigned int i = 1; i < *num_elements; i++)
    {
        while (variable_period_trajectory->time_profile[i] > trajectory_time &&
               time_periods < TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS)
        {
            // Perform a linear interpolation
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
            constant_period_trajectory->time_profile[time_periods] =
                interpolation_period * (float)time_periods;
            time_periods++;
            trajectory_time = interpolation_period * (float)time_periods;
        }
    }

    if (time_periods == TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS)
    {
        return INTERPOLATION_ELEMENT_MAXED_OUT;
    }

    const unsigned int last_element_index = *num_elements - 1;

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
    *num_elements = ++time_periods;
    return OK;
}

void app_trajectory_planner_generateVelocityTrajectory(
    PositionTrajectory_t* position_trajectory, unsigned int num_elements,
    VelocityTrajectory_t* velocity_trajectory)
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
    float* velocity_time_profile = velocity_trajectory->time_profile;

    for (unsigned int i = 0; i < num_elements - 1; i++)
    {
        const float delta_x     = x_positions[i + 1] - x_positions[i];
        const float delta_y     = y_positions[i + 1] - y_positions[i];
        const float delta_theta = (orientations[i + 1] - orientations[i]);

        // The unit vector of the direction is 1/magnitide(vector) *vector
        const float vector_magnitude_inverse =
            (1 / sqrtf(powf(delta_x, 2.0f) + powf(delta_y, 2.0f)));

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

    const unsigned int last_element_index = num_elements - 1;
    // Assume that the velocity of the final segment is in the direction of the previous
    const float delta_x =
        x_positions[last_element_index] - x_positions[last_element_index - 1];
    const float delta_y =
        y_positions[last_element_index] - y_positions[last_element_index - 1];

    float x_velocity_component = 0;
    float y_velocity_component = 0;

    // Get the inverse magnitude of the direction vector for normalization
    const float vector_magnitude_inverse =
        (float)(1 / sqrtf(powf(delta_x, 2) + powf(delta_y, 2)));
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
