#include <ros/ros.h>

#include <boost/exception/diagnostic_information.hpp>

#include "geom/point.h"
#include "network_input/backend.h"
#include "network_input/networking/network_client.h"
#include "network_input/networking/ssl_gamecontroller_client.h"
#include "network_input/networking/ssl_vision_client.h"
#include "thunderbots_msgs/Ball.h"
#include "thunderbots_msgs/Field.h"
#include "thunderbots_msgs/Team.h"
#include "thunderbots_msgs/World.h"
#include "util/constants.h"
#include "util/logger/init.h"
#include "util/parameter/dynamic_parameter_utils.h"
#include "util/parameter/dynamic_parameters.h"
#include "util/ros_messages.h"
#include "util/time/timestamp.h"

int main(int argc, char** argv)
{
    // Init ROS node
    ros::init(argc, argv, "network_input");
    ros::NodeHandle node_handle;

    // Initialize the logger
    Util::Logger::LoggerSingleton::initializeLogger(node_handle);

    // Create and start our NetworkClient
    NetworkClient client = NetworkClient(node_handle);

    // Initialize Dynamic Parameters
    auto update_subscribers =
        Util::DynamicParameters::initUpdateSubscriptions(node_handle);

    // Services any ROS calls in a separate thread "behind the scenes". Does not return
    // until the node is shutdown
    // http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning
    ros::spin();

    return 0;
}
