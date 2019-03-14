#include <dynamic_reconfigure/Reconfigure.h>
#include <ros/ros.h>

#include "util/parameter/parameter.h"

namespace Util::DynamicParameters
{
    /*
     * cfg_strs contain the names of all the cfg files which corresponds
     * to the namespace the /parameter_updates topic will be published on
     * to subscribe to
     */
    extern const std::vector<std::string> cfg_strs;

    /**
     * This creates the subscriber to each seperate reconfigure server
     * to the parameter updates
     */
    std::vector<ros::Subscriber> initUpdateSubscriptions(ros::NodeHandle);

    /**
     * This callback is attatched to the /parameter/parameter_updates topic
     * The new values arrive on this topic and the parameter objects are updated
     * from the Config msg
     */
    void parameterUpdateCallback(const dynamic_reconfigure::Config::ConstPtr& updates);

    /**
     * Updates all known parameters with the latest values from the ROS Parameter
     * Server
     */
    void updateAllParametersFromROSParameterServer();

    /**
     * Updates all known parameters with the latest values from config lists
     * in the dynamic_reconfigure::Config msgs
     *
     */
    void updateAllParametersFromConfigMsg(const dynamic_reconfigure::Config::ConstPtr&);

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
