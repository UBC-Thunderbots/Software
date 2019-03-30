#pragma once
#include "thunderbots_msgs/RobotStatus.h"
#include "thunderbots_msgs/MRFMessages.h"
#include <ros/ros.h>

/**
 * This class publishes messages received from the dongle.
 */
class Annunciator
{
    public:
        /**
         * Constructor.
         * @param node_handle The node handle used to initialize the publisher.
         */
        explicit Annunciator(ros::NodeHandle& node_handle);

        /**
         * Handles diagnostics for each robot.
         * 
         * @param index Robot number.
         * @param data The data of the status packet.
         * @param len The length of the packet.
         * @param lqi Link quality.
         * @param rssi Received signal strength indicator.
         */
        thunderbots_msgs::MRFMessages handle_robot_message(int index, const void *data,
                                                        std::size_t len, uint8_t lqi,
                                                        uint8_t rssi);

        /**
         * Handles general dongle messages.
         * 
         * @param status The integer encoding all the status data.
         */
        thunderbots_msgs::MRFMessages handle_status(uint8_t status);

    private:
        ros::Publisher robot_status_publisher; 
        thunderbots_msgs::MRFMessages mrf_message;
};