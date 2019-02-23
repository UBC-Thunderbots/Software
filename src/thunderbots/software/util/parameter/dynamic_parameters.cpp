#include "util/parameter/dynamic_parameters.h"

namespace Util::DynamicParameters
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
        Parameter<double> collision_avoid_velocity_scale("collision_avoid_velocity_scale",
                                                         2.0);
    }  // namespace Navigator

    namespace Indirect_Chip_Evaluation
    {
        // Adjusts how far between ball and target the robot will chip
        Parameter<double> chip_target_fraction("chip_target_fraction", 5.0 / 10.0);

        // Maximum fraction of distance between chipper and target the first bounce
        // should be, so ball is rolling when it reaches the target
        Parameter<double> chip_power_bounce_threshold("chip_power_bounce_threshold",
                                                      7.5 / 10.0);

        // Maximum power the robot can chip the ball at without malfunctions
        Parameter<double> max_chip_power("max_chip_power", 8.0);

        // Closest distance to edge of field that the robot could chip and chase to
        Parameter<double> chip_target_area_inset("chip_target_area_inset", 0.3);

        // Minimum area of chip target triangle
        Parameter<double> min_chip_tri_area("min_chip_tri_area", 0.5);

        // Minimum edge length of chip target triangle
        Parameter<double> min_chip_tri_edge_len("min_chip_tri_edge_len", 0.8);

        // Minimum angle in degrees between chip triangle edges
        Parameter<double> min_chip_tri_edge_angle("min_chip_tri_edge_angle", 20);

        // Percentage of distance to center of triangle to return as target
        Parameter<double> chip_cherry_power_downscale("chip_cherry_power_downscale",
                                                      0.85);
    }  // namespace Indirect_Chip_Evaluation

    namespace AI
    {
        namespace Passing
        {
            Parameter<double> static_field_position_quality_x_offset(
                "static_field_position_quality_x_offset", 0.3);
            Parameter<double> static_field_position_quality_y_offset(
                "static_field_position_quality_y_offset", 0.3);
            Parameter<double> static_field_position_quality_friendly_goal_distance_weight(
                "static_field_position_quality_friendly_goal_distance_weight", 0.3);
        }  // namespace Passing
    }      // namespace AI
    
}  // namespace Util::DynamicParameters
