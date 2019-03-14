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
            // Adjusts how far between ball and target the robot will chip
            extern Parameter<double> chip_target_fraction;

            // Maximum fraction of distance between chipper and target the first
            // bounce should be, so ball is rolling when it reaches the target
            extern Parameter<double> chip_power_bounce_threshold;

            // Maximum power the robot can chip the ball at without malfunctions
            extern Parameter<double> max_chip_power;

            // Closest distance to edge of field that the robot could chip and chase
            // to
            extern Parameter<double> chip_target_area_inset;

            // Minimum area of chip target triangle
            extern Parameter<double> min_chip_tri_area;

            // Minimum edge length of chip target triangle
            extern Parameter<double> min_chip_tri_edge_len;

            // Minimum angle in degrees between chip triangle edges
            extern Parameter<double> min_chip_tri_edge_angle;

            // Percentage of distance to center of triangle to return as target
            extern Parameter<double> chip_cherry_power_downscale;
        }  // namespace Indirect_Chip
    }  // namespace Evaluation
}  // namespace Util::DynamicParameters
