#include "util/parameter/dynamic_parameters.h"

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
        Parameter<std::vector<std::string>>::updateAllParametersFromROSParameterServer();
    }

    const extern Parameter<int> param1("/thunderbots/parameters/param1", 7);

    namespace Navigator
    {
        // Default avoid distance around robots.
        const Parameter<double> default_avoid_dist(
            "/thunderbots/parameters/default_avoid_dist", 0.15);

        // Scaling factor for collision avoidance.
        // TODO this is arbitrary for now; could be determined as part of
        // #23: https://github.com/UBC-Thunderbots/Software/issues/23
        const Parameter<double> collision_avoid_velocity_scale(
            "/thunderbots/parameters/collision_avoid_velocity_scale", 2.0);
    }

    namespace HL
    {
        namespace STP
        {
            const Parameter<std::vector<std::string>> param3(
                "/thunderbots/parameters/param3", {"MovePlay", "IdlePlay"});
        }
    }
}