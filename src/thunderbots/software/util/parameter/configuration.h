#pragma once

#include "util/parameter/parameter.h"

/**
 * This namespace contains all of our dynamically adjustable Parameters, providing
 * a centralized way to access them. See the comment in the parameter.h file for
 * a list of valid Parameter types.
 */
namespace Configuration
{
/**
 * Updates all known parameters with the latest values from the ROS Parameter Server
 */
void updateParametersFromParameterServer();

// These are currently placeholders to provide an example of how parameters are
// created an initialized. Parameters can be organized in (nested) namespaces.
const Parameter<int> param1("/thunderbots/parameters/param1", 7);

namespace Navigator
{
const Parameter<double> param2("/thunderbots/parameters/param2", 55.0);
}

namespace HL
{
namespace STP
{
const Parameter<std::vector<std::string>> param3("/thunderbots/parameters/param3",
                                                 {"MovePlay", "IdlePlay"});
}
}
}