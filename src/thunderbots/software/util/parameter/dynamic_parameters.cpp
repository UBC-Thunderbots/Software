#include "util/parameter/dynamic_parameters.h"
#include "string.h"

namespace DynamicParameters
{
    std::string kParamNs = "/thunderbots/parameters/"; 

    void updateAllParametersFromROSParameterServer()
    {
        Parameter<bool>::updateAllParametersFromROSParameterServer();
        Parameter<int32_t>::updateAllParametersFromROSParameterServer();
        Parameter<double>::updateAllParametersFromROSParameterServer();
        Parameter<std::string>::updateAllParametersFromROSParameterServer();

        //Parameter<std::vector<bool>>::updateAllParametersFromROSParameterServer();
        //Parameter<std::vector<int32_t>>::updateAllParametersFromROSParameterServer();
        //Parameter<std::vector<double>>::updateAllParametersFromROSParameterServer();
        //Parameter<std::vector<std::string>>::updateAllParametersFromROSParameterServer();
    }

    void updateAllParametersFromROSParameterServer()
    {
        Parameter<bool>::updateAllParametersFromROSParameterServer();
        Parameter<int32_t>::updateAllParametersFromROSParameterServer();
        Parameter<double>::updateAllParametersFromROSParameterServer();
        Parameter<std::string>::updateAllParametersFromROSParameterServer();

        //Parameter<std::vector<bool>>::updateAllParametersFromROSParameterServer();
        //Parameter<std::vector<int32_t>>::updateAllParametersFromROSParameterServer();
        //Parameter<std::vector<double>>::updateAllParametersFromROSParameterServer();
        //Parameter<std::vector<std::string>>::updateAllParametersFromROSParameterServer();
    }

    namespace Navigator
    {
        // Default avoid distance around robots.
        Parameter<double> default_avoid_dist(kParamNs + "default_avoid_dist",
                0.15);

        // Scaling factor for collision avoidance.
        // TODO this is arbitrary for now; could be determined as part of
        // #23: https://github.com/UBC-Thunderbots/Software/issues/23
        Parameter<double> collision_avoid_velocity_scale(kParamNs + "collision_avoid_velocity_scale", 
                2.0);
    }

    namespace Example
    {
        //Example parameters of all types

        //bool
        Parameter<bool> bl(kParamNs + "bool", false);
        //string
        std::string test = "example";
        //Parameter<std::string> strng(kParamNs + "string", test.c_str());
        Parameter<int> it(kParamNs + "int", -1);
        //double
        Parameter<double> dbl(kParamNs + "double",12.0);

        // Scaling factor for collision avoidance.
        // TODO this is arbitrary for now; could be determined as part of
        // #23: https://github.com/UBC-Thunderbots/Software/issues/23
        Parameter<double> collision_avoid_velocity_scale(kParamNs + "collision_avoid_velocity_scale", 
                2.0);
    }
}


