#include "util/parameter/configuration.h"

namespace Configuration
{
void updateParametersFromParameterServer()
{
    for (const auto& p : Parameter<bool>::getRegistry())
    {
        p->updateValueFromParameterServer();
    }

    for (const auto& p : Parameter<int32_t>::getRegistry())
    {
        p->updateValueFromParameterServer();
    }

    for (const auto& p : Parameter<double>::getRegistry())
    {
        p->updateValueFromParameterServer();
    }

    for (const auto& p : Parameter<std::string>::getRegistry())
    {
        p->updateValueFromParameterServer();
    }

    for (const auto& p : Parameter<std::vector<bool>>::getRegistry())
    {
        p->updateValueFromParameterServer();
    }

    for (const auto& p : Parameter<std::vector<int32_t>>::getRegistry())
    {
        p->updateValueFromParameterServer();
    }

    for (const auto& p : Parameter<std::vector<double>>::getRegistry())
    {
        p->updateValueFromParameterServer();
    }

    for (const auto& p : Parameter<std::vector<std::string>>::getRegistry())
    {
        p->updateValueFromParameterServer();
    }
}
}