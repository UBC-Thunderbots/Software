#include "configuration.h"

namespace Configuration
{
// Update functions
void updateParametersFromParameterServer()
{
    for (const auto& parameter : BooleanParameter::getRegistry())
    {
        parameter->updateValueFromParameterServer();
    }

    for (const auto& parameter : IntegerParameter::getRegistry())
    {
        parameter->updateValueFromParameterServer();
    }

    for (const auto& parameter : DoubleParameter::getRegistry())
    {
        parameter->updateValueFromParameterServer();
    }

    for (const auto& parameter : StringParameter::getRegistry())
    {
        parameter->updateValueFromParameterServer();
    }
}

// Parameters
// These are currently placeholders to provide an example of how parameters are
// created an initialized
BooleanParameter param1("/thunderbots/paramters/param1", true);
IntegerParameter param2("/thunderbots/paramters/param2", 5);
DoubleParameter param3("/thunderbots/paramters/param3", -6.1818);
StringParameter param4("/thunderbots/paramters/param4", "Hello, World!");
}