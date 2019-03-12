#include "util/parameter/dynamic_parameter_utils.h"

#include "util/parameter/dynamic_parameters.h"

namespace Util::DynamicParameters
{
    std::vector<ros::Subscriber> initUpdateSubscriptions(ros::NodeHandle nh)
    {
        std::vector<ros::Subscriber> update_subscriptions;
        for (const std::string& cfg : Util::DynamicParameters::cfg_strs)
        {
            update_subscriptions.emplace_back(
                nh.subscribe("/" + cfg + "/parameter_updates", 1,
                             Util::DynamicParameters::parameterUpdateCallback));
        }
        return update_subscriptions;
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
