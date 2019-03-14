#pragma once
#include "util/parameter/parameter.h"
namespace Util
{
    /**
     * This namespace contains all of our dynamically adjustable Parameters, providing
     * a centralized way to access them. See the comment in the parameter.h file for
     * a list of valid Parameter types.
     */
    namespace DynamicParameters
    {
        /**
         * Updates all known parameters with the latest values from the ROS Parameter
         * Server
         */
        void updateAllParametersFromROSParameterServer();

        /**
         * Updates all known parameters with the latest values from config lists
         * in the dynamic_reconfigure::Config msgs
         *
         */
        void updateAllParametersFromConfigMsg(
            const dynamic_reconfigure::Config::ConstPtr&);
        // How long in milliseconds a Robot must not appear in vision before it is removed
        // from the AI
        extern Parameter<int32_t> robot_expiry_buffer_milliseconds;

        namespace Navigator
        {
            // Default avoid distance around robots.
            extern Parameter<double> default_avoid_dist;

            // Scaling factor for collision avoidance.
            // TODO this is arbitrary for now; could be determined as part of
            // #23: https://github.com/UBC-Thunderbots/Software/issues/23
            extern Parameter<double> collision_avoid_velocity_scale;
        }  // namespace Navigator

        namespace Evaluation
        {
            namespace Indirect_Chip
            {
                // Adjusts how far between ball and target the robot will chip
                extern Parameter<double> chip_target_fraction;

                // Maximum fraction of distance between chipper and target the first
                // bounce should be, so ball is rolling when it reaches the target
                extern Parameter<double> chip_power_bounce_threshold;

                // Maximum power the robot can chip the ball at without malfunctions
                extern Parameter<double> max_chip_power;

                // Closest distance to edge of field that the robot could chip and chase
                // to
                extern Parameter<double> chip_target_area_inset;

                // Minimum area of chip target triangle
                extern Parameter<double> min_chip_tri_area;

                // Minimum edge length of chip target triangle
                extern Parameter<double> min_chip_tri_edge_len;

                // Minimum angle in degrees between chip triangle edges
                extern Parameter<double> min_chip_tri_edge_angle;

                // Percentage of distance to center of triangle to return as target
                extern Parameter<double> chip_cherry_power_downscale;
            }  // namespace Indirect_Chip

        }  // namespace Evaluation

        namespace AI
        {
            namespace Passing
            {
                // The offset from the sides of the field to place the rectangular
                // sigmoid we use to determine what areas to pass to
                extern Parameter<double> static_field_position_quality_x_offset;
                extern Parameter<double> static_field_position_quality_y_offset;

                // The weight that being close to the goal will have on the static
                // position quality. Lower, more negative weights result in the distance
                // to the goal having less of an effect.
                extern Parameter<double>
                    static_field_position_quality_friendly_goal_distance_weight;

                // The number of passes to try to optimize at any given time
                extern Parameter<int32_t> num_passes_to_optimize;

                // The number of passes to keep after pruning
                extern Parameter<int32_t> num_passes_to_keep_after_pruning;

                // The number of steps of gradient descent to perform in each iteration
                extern Parameter<int32_t> number_of_gradient_descent_steps_per_iter;

                // The maximum allowed difference between the reciever and passer points
                // of two passes for which they are considered equal
                extern Parameter<double> pass_equality_max_position_difference_meters;

                // The maximum allowed difference between the start times of two passes
                // for which they are considered equal
                extern Parameter<double> pass_equality_max_start_time_difference_seconds;

                // The maximum allowed difference between the speeds of two passes for
                // which they are considered equal
                extern Parameter<double>
                    pass_equality_max_speed_difference_meters_per_second;

            }  // namespace Passing
        }      // namespace AI

        namespace XBoxControllerDemo
        {
            extern Parameter<int32_t> robot_id;
            extern Parameter<double> kick_speed_meters_per_second;
            extern Parameter<double> chip_distance_meters;
            extern Parameter<double> dribbler_rpm;
            extern Parameter<double> linear_sensitivity;
            extern Parameter<double> angular_sensitivity;
        }  // namespace XBoxControllerDemo
    }      // namespace DynamicParameters
}  // namespace Util
