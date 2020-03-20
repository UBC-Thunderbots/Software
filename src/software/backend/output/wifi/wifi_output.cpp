#include "software/backend/output/wifi/wifi_output.h"

#include <chrono>
#include <g3log/g3log.hpp>

#include "shared/constants.h"
#include "shared/proto/primitive.pb.h"
#include "shared/proto/status.pb.h"
#include "shared/proto/vision.pb.h"
#include "software/backend/output/wifi/communication/protobuf_primitive_visitor.h"

WifiOutput::WifiOutput(std::unique_ptr<RobotPrimitiveCommunicator> primitive_comms,
                       std::unique_ptr<RobotVisionCommunicator> vision_coms)
    : primitive_comms(std::move(primitive_comms)), vision_comms(std::move(vision_comms))
{
}

void WifiOutput::sendPrimitives(const std::vector<std::unique_ptr<Primitive>> &primitives)
{
    // convert primitive vector to protobuf vector
    std::vector<PrimitiveMsg> proto_primitives;

    // convert primitive to protobuf
    for (auto &&prim : primitives)
    {
        ProtobufPrimitiveVisitor visitor;
        prim->accept(visitor);
        proto_primitives.emplace_back(visitor.getPrimitiveMsg());
    }

    primitive_comms->send_proto_vector(proto_primitives);
}

void WifiOutput::sendVisionPacket(
    std::vector<std::tuple<uint8_t, Point, Angle>> friendly_robots, Ball ball)
{
    uint64_t timestamp = static_cast<uint64_t>(ball.lastUpdateTimestamp().getSeconds());
    // TODO IMPLEMENT ME!
}

void WifiOutput::sendVisionPacket(const Team &friendly_team, Ball ball)
{
    std::vector<std::tuple<uint8_t, Point, Angle>> robot_tuples;
    for (const Robot &robot : friendly_team.getAllRobots())
    {
        robot_tuples.emplace_back(std::make_tuple<uint8_t, Point, Angle>(
            robot.id(), robot.position(), robot.orientation()));
    }
    sendVisionPacket(robot_tuples, ball);
}
