#pragma once
#include "util/parameter/parameter.h"

/**
 * This namespace contains all of our dynamically adjustable Parameters, providing
 * a centralized way to access them. See the comment in the parameter.h file for
 * a list of valid Parameter types.
 */
namespace DynamicParameters
{
    /**
     * Updates all known parameters with the latest values from the ROS Parameter Server
     */
    void updateAllParametersFromROSParameterServer();

    namespace Navigator
    {
        extern Parameter<double> default_avoid_dist;
        extern Parameter<double> collision_avoid_velocity_scale; 
    }

    namespace Example
    {
        extern Parameter<bool> bl;
        extern Parameter<std::string> strng;
        extern Parameter<int32_t> it;
        extern Parameter<double> dbl;
        extern Parameter<double> collision_avoid_velocity_scale;    
    }

}
