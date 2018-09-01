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

    // These are currently placeholders to provide an example of how parameters are
    // created an initialized. Parameters can be organized in (nested) namespaces.
    const extern Parameter<int> param1;

    namespace Navigator
    {
        const extern Parameter<double> param2;
    }

    namespace HL
    {
        namespace STP
        {
            const extern Parameter<std::vector<std::string>> param3;
        }
    }
}