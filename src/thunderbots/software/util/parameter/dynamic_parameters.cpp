#include "util/parameter/dynamic_parameters.h"

namespace DynamicParameters
{
    std::string kParamNs = "/thunderbots/parameters/"; 

    void updateAllParametersFromROSParameterServer()
    {
	//currently supported
        Parameter<bool>::updateAllParametersFromROSParameterServer();
        Parameter<int32_t>::updateAllParametersFromROSParameterServer();
	Parameter<double>::updateAllParametersFromROSParameterServer();
        Parameter<std::string>::updateAllParametersFromROSParameterServer();

	//currently not supported
        Parameter<std::vector<bool>>::updateAllParametersFromROSParameterServer();
        Parameter<std::vector<int32_t>>::updateAllParametersFromROSParameterServer();
        Parameter<std::vector<double>>::updateAllParametersFromROSParameterServer();
        Parameter<std::vector<std::string>>::updateAllParametersFromROSParameterServer();
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
        
        Parameter<bool> bl(kParamNs + "bool_example", false); //bool
        Parameter<std::string> strng(kParamNs + "string_example", "example"); //string
        Parameter<int> it(kParamNs + "int_example", -1); //int
        Parameter<double> dbl(kParamNs + "double_example",12.0); //double

        // Scaling factor for collision avoidance.
        // TODO this is arbitrary for now; could be determined as part of
        // #23: https://github.com/UBC-Thunderbots/Software/issues/23
        Parameter<double> collision_avoid_velocity_scale(kParamNs + "collision_avoid_velocity_scale", 
							2.0);
    }
}


