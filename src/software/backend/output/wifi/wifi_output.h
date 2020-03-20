#pragma once

#include <limits>

#include "software/backend/output/radio/mrf/dongle.h"
#include "software/backend/robot_status.h"
#include "software/world/ball.h"
#include "software/world/team.h"

using RobotPrimitiveCommunicator = RobotCommunicator<primitive_msg, status_msg>;
using RobotVisionCommunicator = RobotCommunicator<vision_msg, status_msg>;

class WifiOutput
{
   public:
    /**
     * Creates a new WifiOutput.
     *
     * Automatically joins the multicast group for both primitive and vision
     * communication using the provided RobotCommunicators
     *
     * @param primitive_coms The RobotCommunicator used to send primitives
     * @param vision_coms The RobotCommunicator used to send vision data
     *
     */
    explicit WifiOutput(RobotPrimitiveCommunicator primitive_comms,
                            RobotVisionCommunicator vision_comms);

    /**
     * Sends the given primitives to the backend to control the robots
     *
     * @param primitives the list of primitives to send
     */
    void sendPrimitives(const std::vector<std::unique_ptr<Primitive>>& primitives);

    /**
     * Sends a camera packet with the detected robots and ball.
     *
     * @param friendly_robots a vector of tuples of {robot id, robot location,
     *                        robot orientation}
     * @param ball
     */
    void sendVisionPacket(std::vector<std::tuple<uint8_t, Point, Angle>> friendly_robots,
                          Ball ball);

    /**
     * Sends a camera packet with the detected robots and ball.
     *
     * @param friendly_team
     * @param ball
     */
    void sendVisionPacket(const Team& friendly_team, Ball ball);

   private:
    RobotPrimitiveCommunicator primitive_comms;
    RobotVisionCommunicator vision_comms;
};
