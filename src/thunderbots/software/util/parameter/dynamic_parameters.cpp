#include "util/parameter/dynamic_parameters.h"

namespace Util
{
    namespace DynamicParameters
    {
        void updateAllParametersFromROSParameterServer()
        {
            Parameter<bool>::updateAllParametersFromROSParameterServer();
            Parameter<int32_t>::updateAllParametersFromROSParameterServer();
            Parameter<double>::updateAllParametersFromROSParameterServer();
            Parameter<std::string>::updateAllParametersFromROSParameterServer();
            Parameter<std::vector<bool>>::updateAllParametersFromROSParameterServer();
            Parameter<std::vector<int32_t>>::updateAllParametersFromROSParameterServer();
            Parameter<std::vector<double>>::updateAllParametersFromROSParameterServer();
            Parameter<
                std::vector<std::string>>::updateAllParametersFromROSParameterServer();
        }

        Parameter<unsigned int> robot_expiry_buffer_milliseconds(
            "thunderbots/parameters/robot_expiry_buffer_milliseconds", 1000);

        namespace Navigator
        {
            // Default avoid distance around robots.
            Parameter<double> default_avoid_dist(
                "/thunderbots/parameters/default_avoid_dist", 0.15);

            // Scaling factor for collision avoidance.
            // TODO this is arbitrary for now; could be determined as part of
            // #23: https://github.com/UBC-Thunderbots/Software/issues/23
            Parameter<double> collision_avoid_velocity_scale(
                "/thunderbots/parameters/collision_avoid_velocity_scale", 2.0);
        }  // namespace Navigator
    }      // namespace DynamicParameters
}  // namespace Util