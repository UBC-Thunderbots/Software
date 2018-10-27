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

        // How long in milliseconds a Robot must not appear in vision before it is removed
        // from the AI
        extern Parameter<unsigned int> robot_expiry_buffer_milliseconds;

        namespace Navigator
        {
            extern Parameter<double> default_avoid_dist;
            extern Parameter<double> collision_avoid_velocity_scale;
        }  // namespace Navigator
    }      // namespace DynamicParameters
}  // namespace Util
