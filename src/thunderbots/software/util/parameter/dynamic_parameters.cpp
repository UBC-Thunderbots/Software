#include "util/parameter/dynamic_parameters.h"

namespace DynamicParameters
{
    void updateAllParametersFromROSParameterServer()
    {
        Parameter<bool>::updateAllParametersFromROSParameterServer();
        Parameter<int32_t>::updateAllParametersFromROSParameterServer();
	Parameter<double>::updateAllParametersFromROSParameterServer();
        Parameter<std::string>::updateAllParametersFromROSParameterServer();

	//currently not supported (possible to implement through custom dynamic reconf)
        //Parameter<std::vector<bool>>::updateAllParametersFromROSParameterServer();
        //Parameter<std::vector<int32_t>>::updateAllParametersFromROSParameterServer();
        //Parameter<std::vector<double>>::updateAllParametersFromROSParameterServer();
        //Parameter<std::vector<std::string>>::updateAllParametersFromROSParameterServer();
    }

    Parameter<int32_t> robot_expiry_buffer_milliseconds(
        "robot_expiry_buffer_milliseconds", 1000);

    namespace Navigator
    {
        // Default avoid distance around robots.
        Parameter<double> default_avoid_dist("default_avoid_dist", 0.15);

        // Scaling factor for collision avoidance.
        // TODO this is arbitrary for now; could be determined as part of
        // #23: https://github.com/UBC-Thunderbots/Software/issues/23
        Parameter<double> collision_avoid_velocity_scale(
            "collision_avoid_velocity_scale", 2.0);
    }  // namespace Navigator

    namespace Example
    {
        Parameter<bool> bl("bool_example", false); //bool
        Parameter<std::string> strng("string_example", "example"); //string
        Parameter<int32_t> it("int_example", -1); //int
        Parameter<double> dbl("double_example",12.0); //double
    } // namespace Example
}  // namespace DynamicParameters
