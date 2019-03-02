#include "util/parameter/dynamic_parameter_utils.h"

namespace Util::DynamicParameters
{
    ros::Subscriber initParamUpdateSubscription(ros::NodeHandle& node_handle)
    {
        // return the subscriber that updates the parameters on change
        return node_handle.subscribe(node_handle.getNamespace()+"/parameter_updates", 1,
                                     Util::DynamicParameters::parameterUpdateCallback);
    }

    void parameterUpdateCallback(const dynamic_reconfigure::Config::ConstPtr& updates)
    {
        Parameter<bool>::updateAllParametersFromConfigMsg(updates);
        Parameter<int32_t>::updateAllParametersFromConfigMsg(updates);
        Parameter<double>::updateAllParametersFromConfigMsg(updates);
        Parameter<std::string>::updateAllParametersFromConfigMsg(updates);
    }

    void updateAllParametersFromROSParameterServer()
    {
        Parameter<bool>::updateAllParametersFromROSParameterServer();
        Parameter<int32_t>::updateAllParametersFromROSParameterServer();
        Parameter<double>::updateAllParametersFromROSParameterServer();
        Parameter<std::string>::updateAllParametersFromROSParameterServer();
    }

}  // namespace Util::DynamicParameters
