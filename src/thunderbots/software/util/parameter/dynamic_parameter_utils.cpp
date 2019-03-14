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
    
    namespace Evaluation
    {
        namespace Indirect_Chip
        {
            Parameter<double> chip_target_fraction("chip_target_fraction", 5.0 / 10.0);
            Parameter<double> chip_power_bounce_threshold("chip_power_bounce_threshold",
                                                          7.5 / 10.0);
            Parameter<double> max_chip_power("max_chip_power", 8.0);
            Parameter<double> chip_target_area_inset("chip_target_area_inset", 0.3);
            Parameter<double> min_chip_tri_area("min_chip_tri_area", 0.5);
            Parameter<double> min_chip_tri_edge_len("min_chip_tri_edge_len", 0.8);
            Parameter<double> min_chip_tri_edge_angle("min_chip_tri_edge_angle", 20);
            Parameter<double> chip_cherry_power_downscale("chip_cherry_power_downscale",
                                                          0.85);
        }  // namespace Indirect_Chip
    }      // namespace Evaluation
}  // namespace Util::DynamicParameters
