#pragma once

#include "util/parameter/boolean/boolean_parameter.h"
#include "util/parameter/double/double_parameter.h"
#include "util/parameter/integer/integer_parameter.h"
#include "util/parameter/string/string_parameter.h"

namespace Configuration
{
/**
 * Updates all known parameters with the latest values from the ROS Parameter Server
 */
void updateParametersFromParameterServer();

// Parameters
extern BooleanParameter param1;
extern IntegerParameter param2;
extern DoubleParameter param3;
extern StringParameter param4;
}