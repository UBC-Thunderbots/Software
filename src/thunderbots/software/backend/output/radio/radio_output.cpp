#include "radio_output.h"

#include <chrono>

#include "util/logger/init.h"

RadioOutput::RadioOutput(unsigned int config, ros::NodeHandle &node_handle)
    : annunciator(Annunciator(node_handle)), dongle(MRFDongle(config, annunciator))
{
}

RadioOutput::~RadioOutput() {}

void RadioOutput::sendPrimitives(const std::vector<std::unique_ptr<Primitive>> &primitives)
{
    dongle.send_drive_packet(primitives);
}

void RadioOutput::send_vision_packet(std::vector<std::tuple<uint8_t, Point, Angle>> friendly_robots,
                                     Ball ball)
{
    uint64_t timestamp = static_cast<uint64_t>(ball.lastUpdateTimestamp().getSeconds());
    dongle.send_camera_packet(friendly_robots, ball.position() * MILLIMETERS_PER_METER, timestamp);
}

void RadioOutput::send_vision_packet(const Team & friendly_team, Ball ball) {
    std::vector<std::tuple<uint8_t, Point, Angle>> robot_tuples;
    for (const Robot& robot : friendly_team.getAllRobots()){
        robot_tuples.emplace_back(std::make_tuple<uint8_t, Point, Angle>(robot.id(), robot.position(), robot.orientation()));
    }
    send_vision_packet(robot_tuples, ball);
}
