#include "util/parameter/configuration.h"

namespace DynamicParameters
{
void updateAllParametersFromROSParameterServer()
{
    Parameter<bool>::updateAllParametersFromROSParameterServer();
    Parameter<int32_t>::updateAllParametersFromROSParameterServer();
    Parameter<double>::updateAllParametersFromROSParameterServer();
    Parameter<std::string>::updateAllParametersFromROSParameterServer();
    Parameter<std::vector<bool>>::updateAllParametersFromROSParameterServer();
    Parameter<std::vector<int32_t>>::updateAllParametersFromROSParameterServer();
    Parameter<std::vector<double>>::updateAllParametersFromROSParameterServer();
    Parameter<std::vector<std::string>>::updateAllParametersFromROSParameterServer();
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