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

const extern Parameter<int> param1("/thunderbots/parameters/param1", 7);

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