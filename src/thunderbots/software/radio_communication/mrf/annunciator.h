#pragma once
#include <ros/ros.h>

#include "thunderbots_msgs/RobotStatus.h"

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
     * Updates detected robots from vision, used to determine dead bots.
     */
    void update_vision_detections();

    /**
     * Decodes diagnostics and messages for each robot, and publishes them.
     * Returns a boolean if there were new messages since the last status update.
     *
     * @param index Robot number.
     * @param data The data of the status packet.
     * @param len The length of the packet.
     * @param lqi Link quality.
     * @param rssi Received signal strength indicator.
     *
     * @return true if new messages since last status update
     */
    bool handle_robot_message(int index, const void* data, std::size_t len, uint8_t lqi,
                              uint8_t rssi);

    /**
     * Handles general dongle messages.
     *
     * @param status The uint8 encoding all the status data.
     * @return vector of dongle messages sent from the dongle
     */
    std::vector<std::string> handle_dongle_messages(uint8_t status);

   private:
    ros::Publisher robot_status_publisher;
    std::vector<std::string> dongle_messages;
};