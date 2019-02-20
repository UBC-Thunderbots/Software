#include "util/parameter/dynamic_parameters.h"

namespace Util
{
    namespace DynamicParameters
    {
        void updateAllParametersFromROSParameterServer()
        {
            Parameter<bool>::updateAllParametersFromROSParameterServer();
            Parameter<int32_t>::updateAllParametersFromROSParameterServer();
            Parameter<double>::updateAllParametersFromROSParameterServer();
            Parameter<std::string>::updateAllParametersFromROSParameterServer();
        }

        void updateAllParametersFromConfigMsg(
            const dynamic_reconfigure::Config::ConstPtr& updates)
        {
            Parameter<bool>::updateAllParametersFromConfigMsg(updates);
            Parameter<int32_t>::updateAllParametersFromConfigMsg(updates);
            Parameter<double>::updateAllParametersFromConfigMsg(updates);
            Parameter<std::string>::updateAllParametersFromConfigMsg(updates);
        }

        Parameter<int32_t> robot_expiry_buffer_milliseconds(
            "robot_expiry_buffer_milliseconds", 1000);

        namespace Navigator
        {
            // Default avoid distance around robots.
            Parameter<double> default_avoid_dist("default_avoid_dist", 0.15);

            // Scaling factor for collision avoidance.
            // TODO this is arbitrary for now; could be determined as part of
            // #23: https://github.com/UBC-Thunderbots/Software/issues/23
            Parameter<double> collision_avoid_velocity_scale(
                "collision_avoid_velocity_scale", 2.0);
        }  // namespace Navigator

        namespace Indirect_Chip_Evaluation
        {
            // Adjusts how far between ball and target the robot will chip
            Parameter<double> chip_target_fraction("",);

            // Maximum fraction of distance between chipper and target the first bounce
            // should be, so ball is rolling when it reaches the target
            Parameter<double> chip_power_bounce_threshold;

            // Maximum power the robot can chip the ball at without malfunctions
            Parameter<double> max_chip_power;

            // Closest distance to edge of field that the robot could chip and chase to
            Parameter<double> chip_target_area_inset;

            // Minimum area of chip target triangle
            Parameter<double> min_chip_tri_area;

            // Minimum edge length of chip target triangle
            Parameter<double> min_chip_tri_edge_len;

            // Minimum angle in degrees between chip triangle edges
            Parameter<double> min_chip_tri_edge_angle;

            // Closest distance to edge of field that the robot could chip and chase to
            Parameter<double> chip_cherry_power_downscale;
        }  // namespace Indirect_Chip_Evaluation
    }      // namespace DynamicParameters
}  // namespace Util
