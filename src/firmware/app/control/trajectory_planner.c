#include "firmware/app/control/trajectory_planner.h"

#include <stdbool.h>

#include "firmware/shared/math/polynomial_1d.h"
#include "firmware/shared/math/polynomial_2d.h"
#include "firmware/shared/math/vector_2d.h"
#include "math.h"
#include "stdio.h"

void generate_constant_arc_length_segmentation(
    FirmwareRobotPathParameters_t path_parameters, Trajectory_t* trajectory)
{
    TrajectoryElement_t reverse_trajectory[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    double max_allowable_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    double velocity_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];


    // Set the internal path parameter variables
    TrajectoryElement_t* traj_elements = trajectory->trajectory_elements;

    double final_speed                      = path_parameters.final_speed;
    double initial_speed                    = path_parameters.initial_speed;
    const double max_allowable_acceleration = path_parameters.max_allowable_acceleration;
    const double max_allowable_speed        = path_parameters.max_allowable_speed;
    const unsigned int num_segments         = path_parameters.num_segments;
    const Polynomial2dOrder3_t path         = path_parameters.path;
    double t_end                            = path_parameters.t_end;
    double t_start                          = path_parameters.t_start;

    trajectory->num_elements = path_parameters.num_segments;

    // Variable used to flag if the path is moving "backwards" along the input path
    bool reverse_parameterization = false;


    // Check for the parameterization direction
    // If the path is traversed in reverse, then flip all components to forwards (to be
    // reversed again in the end)
    if (t_end < t_start)
    {
        reverse_parameterization = true;

        // Reverse the direction (Polynomial library can only handle forwards direction)
        double temp = t_start;
        t_start     = t_end;
        t_end       = temp;

        // Reverse the final and initial speeds
        temp          = initial_speed;
        initial_speed = final_speed;
        final_speed   = temp;
    }

    // Create the parmeterization to contain the desired number of segments
    CREATE_STATIC_ARC_LENGTH_PARAMETRIZATION(arc_length_param,
                                             TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS);

    // Get all of the points for the arc length parameterization (Not constant arc length
    // segments)
    shared_polynomial_getArcLengthParametrizationOrder3(path, t_start, t_end,
                                                        arc_length_param);

    const double arc_segment_length = getTotalArcLength(arc_length_param) / num_segments;

    // Now use numerical interpolation to get constant arc length segments for the
    // parameterization
    getMaxAllowableSpeedProfile(max_allowable_speed_profile, traj_elements, path,
                                num_segments, arc_length_param, arc_segment_length,
                                max_allowable_acceleration);

    // First point on the velocity profile is always the current speed
    velocity_profile[0] = initial_speed;

    // Loop through the path forwards to ensure that the maximum velocity limit defined by
    // the curvature is not breached by constant max acceleration of the robot (if it is,
    // pull down the acceleration)
    for (unsigned int i = 1; i < num_segments; i++)
    {
        // Vf = sqrt( Vi^2 + 2*constant_segment_length*max_acceleration)
        const double temp_vel = sqrt(pow(velocity_profile[i - 1], 2) +
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

    // Now check backwards continuity. This is done to guarantee the robot can deccelerate
    // in time to not breach the maximum allowable speed defined by the path curvature
    velocity_profile[num_segments - 1] = final_speed;

    for (unsigned int i = num_segments - 1; i > 0; i--)
    {
        // Vf = sqrt( Vi^2 + 2*constant_segment_length*max_acceleration)
        const double temp_vel = sqrt(pow(velocity_profile[i], 2) +
                                     2 * arc_segment_length * max_allowable_acceleration);

        if (velocity_profile[i - 1] > temp_vel)
        {
            velocity_profile[i - 1] = temp_vel;
        }
    }

    // Now that the velocity profile has been defined we can calcuate the time profile of
    // the trajectory
    traj_elements[0].time = 0.0;
    for (unsigned int i = 0; i < num_segments - 1; i++)
    {
        traj_elements[i + 1].time =
            traj_elements[i].time +
            (2 * arc_segment_length) / (velocity_profile[i] + velocity_profile[i + 1]);
    }

    // If the parameterization ended up being reversed, we need to flip all the values
    // back.
    // TODO: Remove the 'reverse parameterization' hack #1322
    if (reverse_parameterization == true)
    {
        const double path_duration = traj_elements[num_segments - 1].time;

        for (unsigned int i = 0; i < num_segments; i++)
        {
            // Reverse the positions
            reverse_trajectory[(num_segments - 1) - i].position =
                traj_elements[i].position;
            reverse_trajectory[(num_segments - 1) - i].time =
                fabs(path_duration - traj_elements[i].time);
        }
        // Copy reverse element array back
        for (unsigned int i = 0; i < num_segments; i++)
        {
            traj_elements[i].position = reverse_trajectory[i].position;
            traj_elements[i].time     = reverse_trajectory[i].time;
        }
    }
}


Trajectory_t interpolate_constant_time_trajectory_segmentation(
    Trajectory_t* constant_arclength_trajectory, const double interpolation_period)
{
    static TrajectoryElement_t
        constant_period_trajectory[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];

    // The first point is the same for each trajectory
    constant_period_trajectory[0].time =
        constant_arclength_trajectory->trajectory_elements[0].time;
    constant_period_trajectory[0].position =
        constant_arclength_trajectory->trajectory_elements[0].position;

    // Keep track of the current time we are searching for in the constant arclength
    // trajectory
    unsigned int time_periods = 1;
    double trajectory_time    = interpolation_period * time_periods;

    // Loop until we find a value JUST larger than the expected
    // Check the element prior and perform linear interpolation
    for (unsigned int i = 1; i < constant_arclength_trajectory->num_elements; i++)
    {
        while (constant_arclength_trajectory->trajectory_elements[i].time >=
               trajectory_time)
        {
            // Perform a linear interpolation
            const double delta_time =
                constant_arclength_trajectory->trajectory_elements[i].time -
                constant_arclength_trajectory->trajectory_elements[i - 1].time;
            const double delta_x =
                constant_arclength_trajectory->trajectory_elements[i].position.x -
                constant_arclength_trajectory->trajectory_elements[i - 1].position.x;
            const double delta_y =
                constant_arclength_trajectory->trajectory_elements[i].position.y -
                constant_arclength_trajectory->trajectory_elements[i - 1].position.y;

            const double slope_x = delta_x / delta_time;
            const double slope_y = delta_y / delta_time;

            const double interpolated_x =
                slope_x *
                    (trajectory_time -
                     constant_arclength_trajectory->trajectory_elements[i - 1].time) +
                constant_arclength_trajectory->trajectory_elements[i - 1].position.x;
            const double interpolated_y =
                slope_y *
                    (trajectory_time -
                     constant_arclength_trajectory->trajectory_elements[i - 1].time) +
                constant_arclength_trajectory->trajectory_elements[i - 1].position.y;

            constant_period_trajectory[time_periods].position.x = interpolated_x;
            constant_period_trajectory[time_periods].position.y = interpolated_y;
            constant_period_trajectory[time_periods].time       = trajectory_time;

            time_periods++;
            trajectory_time = interpolation_period * time_periods;
        }
    }

    static Trajectory_t ret;

    ret.trajectory_elements = constant_period_trajectory;
    ret.num_elements        = time_periods;

    return ret;
}


double getTotalArcLength(ArcLengthParametrization_t arc_length_param)
{
    return arc_length_param.arc_length_values[arc_length_param.num_values - 1];
}

void getMaxAllowableSpeedProfile(double max_allowable_speed_profile[],
                                 TrajectoryElement_t traj_elements[],
                                 Polynomial2dOrder3_t path, unsigned int num_elements,
                                 ArcLengthParametrization_t arc_length_param,
                                 double arc_segment_length,
                                 const double max_allowable_acceleration)
{
    Polynomial2dOrder2_t first_deriv = shared_polynomial2d_differentiateOrder3(path);
    Polynomial2dOrder1_t second_deriv =
        shared_polynomial2d_differentiateOrder2(first_deriv);

    for (unsigned int i = 0; i < num_elements; i++)
    {
        // Get the 't' value corresponding to the current arc length (to be used for
        // further computing)
        double t = shared_polynomial2d_getTValueAtArcLengthOrder3(
            path, i * arc_segment_length, arc_length_param);

        // Get the X and Y position at the 't' value defined by the arc length
        traj_elements[i].position = shared_polynomial2d_getValueOrder3(path, t);


        // Create the polynomial representing path curvature
        //                                              1
        //                              ---------------------------------
        //                                     abs(x'y'' - y'x'')
        //        radius of curvature =      ----------------------
        //                                     (x'^2 + y'^2)^(3/2)
        //
        const double numerator =
            fabs(shared_polynomial1d_getValueOrder2(first_deriv.x, t) *
                     shared_polynomial1d_getValueOrder1(second_deriv.y, t) -
                 shared_polynomial1d_getValueOrder2(first_deriv.y, t) *
                     shared_polynomial1d_getValueOrder1(second_deriv.x, t));
        const double denominator =
            pow(pow(shared_polynomial1d_getValueOrder2(first_deriv.x, t), 2) +
                    pow(shared_polynomial1d_getValueOrder2(first_deriv.y, t), 2),
                3.0 / 2.0);

        const double radius_of_curvature = 1 / (numerator / denominator);

        max_allowable_speed_profile[i] =
            sqrt(max_allowable_acceleration * radius_of_curvature);
    }
}