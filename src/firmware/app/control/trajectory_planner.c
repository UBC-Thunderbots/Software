#include "firmware/app/control/trajectory_planner.h"

#include "firmware/shared/math/polynomial_1d.h"
#include "firmware/shared/math/polynomial_2d.h"
#include "firmware/shared/math/vector_2d.h"
#include "math.h"
#include "stdio.h"

TrajectoryElement_t* generate_constant_arc_length_segmentation(
    Polynomial2dOrder3_t path, double t_start, double t_end, unsigned int num_segments,
    double max_allowable_acceleration, double max_allowable_speed, double initial_speed,
    double final_speed)
{
    static TrajectoryElement_t trajectory[__TRAJECTORY_PLANNER_MAX_NUM_SEGMENTS__];
    static TrajectoryElement_t
        reverse_trajectory[__TRAJECTORY_PLANNER_MAX_NUM_SEGMENTS__];
    static double max_allowable_speed_profile[__TRAJECTORY_PLANNER_MAX_NUM_SEGMENTS__];
    static double velocity_profile[__TRAJECTORY_PLANNER_MAX_NUM_SEGMENTS__];

    // Variable used to flag if the path is moving "backwards" along the input path
    unsigned int reverse_parameterization = 0;


    // Check for the parameterization direction
    // If the path is traversed in reverse, then flip all components to forwards (to be
    // reversed again in the end)
    if (t_end < t_start)
    {
        reverse_parameterization = 1;

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
                                             __TRAJECTORY_PLANNER_MAX_NUM_SEGMENTS__);

    // Get all of the points for the arc length parameterization (Not constant arc length
    // segments)
    shared_polynomial_getArcLengthParametrizationOrder3(path, t_start, t_end,
                                                        arc_length_param);

    const double arc_segment_length =
        arc_length_param.arc_length_values[arc_length_param.num_values - 1] /
        num_segments;

    Polynomial2dOrder2_t first_deriv = shared_polynomial2d_differentiateOrder3(path);
    Polynomial2dOrder1_t second_deriv =
        shared_polynomial2d_differentiateOrder2(first_deriv);

    // Now use numerial interpolation to get constant arc length segments for the
    // parameterization
    for (unsigned int i = 0; i < num_segments; i++)
    {
        // Get the 't' value corresponding to the current arc length (to be used for
        // further computing)
        double t = shared_polynomial2d_getTValueAtArcLengthOrder3(
            path, i * arc_segment_length, arc_length_param);

        // Get the X and Y position at the 't' value defined by the arc length
        trajectory[i].position = shared_polynomial2d_getValueOrder3(path, t);


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


    double temp_vel = 0;
    // First point on the velocity profile is always the current speed
    velocity_profile[0] = initial_speed;

    // Loop through the path forwards to ensure that the maximum velocity limit defined by
    // the curvature is not breached by constant max acceleration of the robot (if it is,
    // pull down the acceleration)
    for (unsigned int j = 1; j < num_segments; j++)
    {
        // Vf = sqrt( Vi^2 + 2*constant_segment_length*max_acceleration)
        temp_vel = sqrt(pow(velocity_profile[j - 1], 2) +
                        2 * arc_segment_length * max_allowable_acceleration);

        // If the new speed is greater than the max allowable, reduce it to the max
        if (temp_vel > max_allowable_speed_profile[j])
        {
            velocity_profile[j] = max_allowable_speed_profile[j];
        }

        else
        {
            // If the new speed is greater than the maximum limit of the robot, reduce it
            // to the limit
            if (temp_vel >= max_allowable_speed)
            {
                velocity_profile[j] = max_allowable_speed;
            }
            else
            {
                velocity_profile[j] = temp_vel;
            }
        }
    }

    // Now check backwards continuity. This is done to guarantee the robot can deccelerate
    // in time to not breach the maximum allowable speed defined by the path curvature
    velocity_profile[num_segments - 1] = final_speed;

    for (unsigned int j = num_segments - 1; j > 0; j--)
    {
        // Vf = sqrt( Vi^2 + 2*constant_segment_length*max_acceleration)
        temp_vel = sqrt(pow(velocity_profile[j], 2) +
                        2 * arc_segment_length * max_allowable_acceleration);

        if (velocity_profile[j - 1] > temp_vel)
        {
            velocity_profile[j - 1] = temp_vel;
        }
    }

    // Now that the velocity profile has been defined we can calcuate the time profile of
    // the trajectory
    trajectory[0].time = 0.0;
    for (unsigned int j = 0; j < num_segments - 1; j++)
    {
        trajectory[j + 1].time =
            trajectory[j].time +
            (2 * arc_segment_length) / (velocity_profile[j] + velocity_profile[j + 1]);
    }

    // If the parameterization ended up being reversed, we need to flip all the values
    // back
    if (reverse_parameterization == 1)
    {
        const double path_duration = trajectory[num_segments - 1].time;

        for (unsigned int j = 0; j < num_segments; j++)
        {
            // Reverse the positions
            reverse_trajectory[(num_segments - 1) - j].position = trajectory[j].position;
            reverse_trajectory[(num_segments - 1) - j].time =
                fabs(path_duration - trajectory[j].time);
        }

        return reverse_trajectory;
    }

    return trajectory;
}


Trajectory_t interpolate_constant_time_trajectory_segmentation(
    TrajectoryElement_t* constant_arclength_trajectory, unsigned int num_segments,
    const double interpolation_period)
{
    static TrajectoryElement_t
        constant_period_trajectory[__TRAJECTORY_PLANNER_MAX_NUM_SEGMENTS__];

    // The first point is the same for each trajectory
    constant_period_trajectory[0].time     = constant_arclength_trajectory[0].time;
    constant_period_trajectory[0].position = constant_arclength_trajectory[0].position;

    // Keep track of the current time we are searching for in the constant arclength
    // trajectory
    unsigned int time_periods = 1;
    double trajectory_time    = interpolation_period * time_periods;

    // We want to loop through the entire trajectory

    // Loop until we find a value JUST larger than the expected
    // Check the element prior and perform linear interpolation

    for (unsigned int i = 1; i < num_segments; i++)
    {
        while (constant_arclength_trajectory[i].time >= trajectory_time)
        {
            // Perform a linear interpolation
            const double delta_time = constant_arclength_trajectory[i].time -
                                      constant_arclength_trajectory[i - 1].time;
            const double delta_x = constant_arclength_trajectory[i].position.x -
                                   constant_arclength_trajectory[i - 1].position.x;
            const double delta_y = constant_arclength_trajectory[i].position.y -
                                   constant_arclength_trajectory[i - 1].position.y;

            const double slope_x = delta_x / delta_time;
            const double slope_y = delta_y / delta_time;

            const double interpolated_x =
                slope_x * (trajectory_time - constant_arclength_trajectory[i - 1].time) +
                constant_arclength_trajectory[i - 1].position.x;
            const double interpolated_y =
                slope_y * (trajectory_time - constant_arclength_trajectory[i - 1].time) +
                constant_arclength_trajectory[i - 1].position.y;

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