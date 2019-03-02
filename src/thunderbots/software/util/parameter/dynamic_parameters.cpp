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
            Parameter<double> enemy_proximity_importance("enemy_proximity_importance", 0.5);
            Parameter<double> ideal_min_shoot_angle_degrees("ideal_min_shoot_angle_degrees", 40.0);
            Parameter<double> ideal_min_rotation_to_shoot_degrees("ideal_min_rotation_to_shoot_degrees", 5.0);
            Parameter<int32_t> num_passes_to_optimize("num_passes_to_optimize", 50);
            Parameter<int32_t> num_passes_to_keep_after_pruning(
                "num_passes_to_keep_after_pruning", 10);
            Parameter<int32_t> number_of_gradient_descent_steps_per_iter(
                "number_of_gradient_descent_steps_per_iter", 20);
            Parameter<double> pass_equality_max_position_difference_meters(
                "pass_equality_max_position_difference_meters", 0.1);
            Parameter<double> pass_equality_max_start_time_difference_seconds(
                "pass_equality_max_start_time_difference_seconds", 0.5);
            Parameter<double> pass_equality_max_speed_difference_meters_per_second(
                "pass_equality_max_speed_difference_meters_per_second", 0.3);
        }  // namespace Passing
    }      // namespace AI


}  // namespace Util::DynamicParameters
