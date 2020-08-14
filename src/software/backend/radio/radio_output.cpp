#include "software/backend/radio/radio_output.h"

#include <chrono>

#include "shared/constants.h"
#include "software/logger/logger.h"

RadioOutput::RadioOutput(
    unsigned int config,
    std::function<void(RadioRobotStatus)> received_robot_status_callback)
    : annunciator(Annunciator(received_robot_status_callback)),
      dongle(MRFDongle(config, annunciator))
{
}

void RadioOutput::sendPrimitives(const TbotsProto::PrimitiveSet &primitives)
{
    dongle.send_drive_packet(primitives);
}

void RadioOutput::sendVisionPacket(
    std::vector<std::tuple<uint8_t, Point, Angle>> friendly_robots, Ball ball)
{
    uint64_t timestamp = static_cast<uint64_t>(ball.lastUpdateTimestamp().getSeconds());
    dongle.send_camera_packet(friendly_robots,
                              Point(ball.position().toVector() * MILLIMETERS_PER_METER),
                              timestamp);
}

void RadioOutput::sendVisionPacket(const Team &friendly_team, Ball ball)
{
    std::vector<std::tuple<uint8_t, Point, Angle>> robot_tuples;
    for (const Robot &robot : friendly_team.getAllRobots())
    {
        robot_tuples.emplace_back(std::make_tuple<uint8_t, Point, Angle>(
            static_cast<unsigned char>(robot.id()), robot.position(),
            robot.orientation()));
    }
    sendVisionPacket(robot_tuples, ball);
}
