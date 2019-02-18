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
            extern Parameter<double> default_avoid_dist;
            extern Parameter<double> collision_avoid_velocity_scale;
        }  // namespace Navigator

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
