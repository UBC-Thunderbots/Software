#include "firmware/app/control/trajectory_planner_private.h"

#include <assert.h>
#include <float.h>
#include <math.h>

#include "firmware/app/control/trajectory_planner.h"
#include "firmware/shared/math/polynomial_1d.h"
#include "firmware/shared/math/polynomial_2d.h"
#include "firmware/shared/physics.h"

void app_trajectory_planner_impl_getMaximumSpeedProfile(
    Polynomial2dOrder3_t path, unsigned int num_elements, float t_start, float t_end,
    float max_allowable_acceleration, float speed_cap,
    float max_allowable_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS])
{
    const float t_increment = (t_end - t_start) / (float)(num_elements - 1);

    for (unsigned int i = 0; i < num_elements; i++)
    {
        const float current_t = (float)t_start + (float)i * t_increment;

        const float radius_of_curvature =
            shared_polynomial2d_getCurvatureAtPositionOrder3(path, current_t);

        float max_speed = FLT_MAX;
        // Avoid overflow
        if (max_allowable_acceleration <= 1)
        {
            // no overflow possible when max_allowable_acceleration
            max_speed = sqrtf(max_allowable_acceleration * radius_of_curvature);
        }
        else if (radius_of_curvature < FLT_MAX / max_allowable_acceleration)
        {
            // if max_allowable_acceleration is greater than 1 then check for overflow
            max_speed = sqrtf(max_allowable_acceleration * radius_of_curvature);
        }

        max_allowable_speed_profile[i] = fminf(max_speed, speed_cap);
    }
}

void app_trajectory_planner_impl_generate1dSegmentNodesAndLengths(
    float t_start, float t_end, Polynomial1dOrder3_t path_1d, unsigned int num_elements,
    float node_values[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS],
    float segment_lengths[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS])
{
    // Check that the pre conditions are met
    assert(num_elements > 2);
    assert(num_elements <= TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS);
    assert(t_start != t_end);

    node_values[0] = shared_polynomial1d_getValueOrder3(path_1d, t_start);

    const float t_segment_size = (t_end - t_start) / (float)(num_elements - 1);

    for (unsigned int i = 1; i < num_elements; i++)
    {
        const float current_t = (float)t_start + (float)i * t_segment_size;

        // Grab the states at each 't' value
        node_values[i] = shared_polynomial1d_getValueOrder3(path_1d, current_t);

        // Calculate the length of each segment and store it
        const float segment_length = node_values[i] - node_values[i - 1];

        segment_lengths[i - 1] = segment_length;
    }
}

void app_trajectory_planner_impl_generate2dSegmentNodesAndLengths(
    float t_start, float t_end, Polynomial2dOrder3_t path_2d, unsigned int num_elements,
    float x_values[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS],
    float y_values[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS],
    float segment_lengths[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS])
{
    // Hold into the x and y segment lengths to calculate the combined segment length
    float x_lengths[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    float y_lengths[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    app_trajectory_planner_impl_generate1dSegmentNodesAndLengths(
        t_start, t_end, path_2d.x, num_elements, x_values, x_lengths);
    app_trajectory_planner_impl_generate1dSegmentNodesAndLengths(
        t_start, t_end, path_2d.y, num_elements, y_values, y_lengths);

    // total length is the root sum-squared of the individual values
    for (unsigned int i = 0; i < num_elements - 1; i++)
    {
        segment_lengths[i] = sqrtf(powf(x_lengths[i], 2) + powf(y_lengths[i], 2));
    }
}

void app_trajectory_planner_impl_generatePositionTrajectoryTimeProfile(
    float segment_lengths[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS],
    float speeds[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS], unsigned int num_elements,
    float trajectory_durations[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS])
{
    // Calculate the time required to move between the first and last nodes of a
    // trajectory segment
    for (unsigned int i = 0; i < num_elements - 1; i++)
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
        trajectory_durations[i] = delta_time;
    }
}

float app_trajectory_planner_impl_calculateSpeedToMatchDuration(float initial_speed,
                                                                float duration,
                                                                float displacement)
{
    // Calculate the new final speed based on the initial speed, displacement, and
    // the desired duration in time
    float next_speed = (2.0f * displacement - initial_speed * duration) / duration;

    // Since speed is absolute here, we want to prevent returning negative speed that
    // would make us go backwards along the path. The case where next_speed < 0 shouldn't
    // happen under normal circumstances, but this check exists just in case.
    if (next_speed < 0.0f)
    {
        next_speed = 0.0f;
    }

    return next_speed;
}


void app_trajectory_planner_impl_modifySpeedsToMatchLongestSegmentDuration(
    float displacement1[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS],
    float displacement2[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS],
    float durations1[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS],
    float durations2[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS], float num_elements,
    float speeds1[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS],
    float speeds2[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS],
    float complete_time_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS])
{
    // The time profile is relative to the first element, thus is starts at zero
    complete_time_profile[0] = 0.0f;

    for (unsigned int i = 0; i < num_elements - 1; i++)
    {
        // Check for the case that each segment duration isn't equal, so that they can be
        // modified to have the same duration
        if (durations1[i] > durations2[i] && displacement1[i] != 0)
        {
            const float current_speed    = speeds2[i];
            const float desired_duration = durations1[i];
            float *next_speed            = &speeds2[i + 1];
            const float displacement     = displacement2[i];

            *next_speed = app_trajectory_planner_impl_calculateSpeedToMatchDuration(
                current_speed, desired_duration, displacement);

            complete_time_profile[i + 1] = complete_time_profile[i] + desired_duration;
        }
        else if (durations2[i] > durations1[i] && displacement2[i] != 0)
        {
            const float current_speed    = speeds1[i];
            const float desired_duration = durations2[i];
            float *next_speed            = &speeds1[i + 1];
            const float displacement     = displacement1[i];

            *next_speed = app_trajectory_planner_impl_calculateSpeedToMatchDuration(
                current_speed, desired_duration, displacement);

            complete_time_profile[i + 1] = complete_time_profile[i] + desired_duration;
        }
        else
        {
            // This means that the durations are equal and nothing needs to be changed
            complete_time_profile[i + 1] = complete_time_profile[i] + durations1[i];
        }
    }
}

TrajectoryPlannerGenerationStatus_t
app_trajectory_planner_impl_createForwardsContinuousSpeedProfile(
    float final_speed, float segment_lengths[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS],
    float max_allowable_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS],
    float max_allowable_acceleration, float initial_speed, unsigned int num_elements,
    float speeds[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS])
{
    // Set the initial speed
    speeds[0] = initial_speed;

    for (unsigned int i = 1; i < num_elements; i++)
    {
        // 'i' represents the next speed, where [i-1] is the current speed
        const float speed        = speeds[i - 1];
        const float displacement = fabsf(segment_lengths[i - 1]);

        // Vf = sqrtf( Vi^2 + 2*constant_segment_length*max_acceleration)
        float temp_vel =
            shared_physics_getFinalSpeed(speed, displacement, max_allowable_acceleration);

        // Pick  the lowest of the maximum the available speeds
        const float lowest_speed = fminf(max_allowable_speed_profile[i], temp_vel);

        speeds[i] = lowest_speed;
    }

    if (speeds[num_elements - 1] < final_speed)
    {
        return FINAL_VELOCITY_TOO_HIGH;
    }

    speeds[num_elements - 1] = final_speed;
    return OK;
}

TrajectoryPlannerGenerationStatus_t
app_trajectory_planner_impl_modifySpeedsToBeBackwardsContinuous(
    float initial_speed, float segment_lengths[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS],
    float max_allowable_acceleration, unsigned int num_segments,
    float speeds[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS])
{
    for (unsigned int i = num_segments - 1; i > 0; i--)
    {
        const float current_speed  = speeds[i];
        const float previous_speed = speeds[i - 1];
        const float segment_length = fabsf(segment_lengths[i - 1]);

        float temp_speed = shared_physics_getFinalSpeed(current_speed, segment_length,
                                                        max_allowable_acceleration);

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
