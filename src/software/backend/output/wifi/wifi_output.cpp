#include "software/backend/output/radio/radio_output.h"

#include <chrono>
#include <g3log/g3log.hpp>

#include "shared/constants.h"

WiFiOutput::WiFiOutput(RobotPrimitiveCommunicator primitive_comms, 
        RobotVisionCommunicator vision_comms)
    : primitive_comms(primitive_comms),
    vision_comms(vision_comms)
{}

void WiFiOutput::sendPrimitives(
        const std::vector<std::unique_ptr<Primitive>> &primitives)
{
    dongle.send_drive_packet(primitives);
}

void WiFiOutput::sendVisionPacket(
        std::vector<std::tuple<uint8_t, Point, Angle>> friendly_robots, Ball ball)
{
    uint64_t timestamp = static_cast<uint64_t>(ball.lastUpdateTimestamp().getSeconds());
    dongle.send_camera_packet(friendly_robots,
            Point(ball.position().toVector() * MILLIMETERS_PER_METER),
            timestamp);
}

void WiFiOutput::sendVisionPacket(const Team &friendly_team, Ball ball)
{
    std::vector<std::tuple<uint8_t, Point, Angle>> robot_tuples;
    for (const Robot &robot : friendly_team.getAllRobots())
    {
        robot_tuples.emplace_back(std::make_tuple<uint8_t, Point, Angle>(
                    robot.id(), robot.position(), robot.orientation()));
    }
    sendVisionPacket(robot_tuples, ball);
}
