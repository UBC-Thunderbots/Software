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

            }  // namespace Passing
        }      // namespace AI
    }      // namespace DynamicParameters
}  // namespace Util
