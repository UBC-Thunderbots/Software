#include "trajectory_planner.h"

#include "firmware/main/math/polynomial_1d.h"
#include "firmware/main/math/polynomial_2d.h"
#include "firmware/main/math/vector_2d.h"
#include "math.h"
#include "stdio.h"

TrajectoryElement_t* generate_constant_arc_length_segmentation(
    Polynomial2dOrder3_t path, double t_start, double t_end, unsigned int num_segments,
    double max_allowable_acceleration, double max_allowable_speed, double initial_speed,
    double final_speed)
{
    static TrajectoryElement_t trajectory[__TRAJECTORY_PLANNER_MAX_NUM_SEGMENTS__];
    static double max_allowable_speed_profile[__TRAJECTORY_PLANNER_MAX_NUM_SEGMENTS__];
    static double velocity_profile[__TRAJECTORY_PLANNER_MAX_NUM_SEGMENTS__];

    // Create the parmeterization to contain the desired number of segments
    CREATE_STATIC_ARC_LENGTH_PARAMETRIZATION(arc_length_param, __TRAJECTORY_PLANNER_MAX_NUM_SEGMENTS__);

    // Get all of the points for the arc length parameterization (Not constant arc length
    // segments)
    shared_polynomial_getArcLengthParametrizationOrder3(path, t_start, t_end,
                                                        arc_length_param);

    const double arc_segment_length =
        arc_length_param.arc_length_values[arc_length_param.num_values-1] /
            num_segments;

    Polynomial2dOrder2_t first_deriv = shared_polynomial2d_differentiateOrder3(path);
    Polynomial2dOrder1_t second_deriv =
        shared_polynomial2d_differentiateOrder2(first_deriv);

    // Now use numerial interpolation to get constant arc length segments for the
    // parameterization
    for (unsigned int i = 0; i < num_segments; i++)
    {
        // Get the 't' value corresponding to the current arc length (to be used for further computing)
        double t = shared_polynomial2d_getTValueAtArcLengthOrder3(path, i*arc_segment_length, arc_length_param);

        // Get the X and Y position at the 't' value defined by the arc length
        trajectory[i].position = shared_polynomial2d_getValueOrder3(
            path, t);


        // Create the polynomial representing path curvature
        //                                              1
        //                              ---------------------------------
        //                                     abs(x'y'' - y'x'')
        //        radius of curvature =      ----------------------
        //                                     (x'^2 + y'^2)^(3/2)
        //        
        const double numerator = fabs(
            shared_polynomial1d_getValueOrder2(first_deriv.x, t) *
                shared_polynomial1d_getValueOrder1(second_deriv.y,
                                                   t) -
            shared_polynomial1d_getValueOrder2(first_deriv.y, t) *
                shared_polynomial1d_getValueOrder1(second_deriv.x,
                                                   t));
        const double denominator =
            pow(pow(shared_polynomial1d_getValueOrder2(first_deriv.x,
                                                       t),
                    2) +
                    pow(shared_polynomial1d_getValueOrder2(first_deriv.y,
                                                           t),
                        2),
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
    for (unsigned int j = 0; j < num_segments; j++)
    {
        // Vf = sqrt( Vi^2 + 2*constant_segment_length*max_acceleration)
        temp_vel = sqrt(pow(velocity_profile[j], 2) +
                        2 * arc_segment_length * max_allowable_acceleration);

        // If the new speed is greater than the max allowable, reduce it to the max
        if (temp_vel > max_allowable_speed_profile[j + 1])
        {
            velocity_profile[j + 1] = max_allowable_speed_profile[j + 1];
        }

        else
        {
            // If the new speed is greater than the maximum limit of the robot, reduce it
            // to the limit
            if (temp_vel > max_allowable_speed)
            {
                velocity_profile[j + 1] = max_allowable_speed;
            }
            else
            {
                velocity_profile[j + 1] = temp_vel;
            }
        }
    }

    // Now check backwards continuity. This is done to guarantee the robot can deccelerate
    // in time to not breach the maximum allowable speed defined by the path curvature
    velocity_profile[num_segments] = final_speed;

    for (unsigned int j = num_segments; j > 0; j--)
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

    return trajectory;
}
