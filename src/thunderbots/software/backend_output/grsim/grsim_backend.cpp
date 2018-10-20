#include "backend_output/grsim/grsim_backend.h"

#include <ctime>
#include <iostream>
#include <optional>
#include <utility>

#include "ai/primitive/move_primitive.h"
#include "ai/primitive/primitive.h"
#include "ai/world/team.h"
#include "motion_controller.h"
#include "proto/grSim_Commands.pb.h"
#include "shared/constants.h"

using namespace boost::asio;

GrSimBackend::GrSimBackend(std::string network_address, unsigned short port)
    : network_address(network_address), port(port), socket(io_service)
{
    socket.open(ip::udp::v4());
    remote_endpoint = ip::udp::endpoint(ip::address::from_string(network_address), port);
}

GrSimBackend::~GrSimBackend()
{
    socket.close();
}

grSim_Packet GrSimBackend::createGrSimPacket(unsigned int robot_id,
                                             TeamColour team_colour, Vector velocity,
                                             AngularVelocity angular_velocity) const
{
    grSim_Packet packet;

    packet.mutable_commands()->set_isteamyellow(team_colour == YELLOW);
    packet.mutable_commands()->set_timestamp(0.0);
    grSim_Robot_Command* command = packet.mutable_commands()->add_robot_commands();

    command->set_id(robot_id);

    // We set a robot velocity, not individual wheel velocities
    command->set_wheelsspeed(false);

    command->set_veltangent(static_cast<float>(velocity.x()));
    command->set_velnormal(static_cast<float>(velocity.y()));
    command->set_velangular(static_cast<float>(angular_velocity.toRadians()));

    command->set_kickspeedx(0.0);
    command->set_kickspeedz(0.0);
    command->set_spinner(false);

    return packet;
}

void GrSimBackend::sendGrSimPacket(const grSim_Packet& packet)
{
    boost::system::error_code err;
    socket.send_to(
        buffer(packet.SerializeAsString(), static_cast<size_t>(packet.ByteSize())),
        remote_endpoint, 0, err);
}


void GrSimBackend::sendPrimitives(
    const std::vector<std::unique_ptr<Primitive>>& primitives, const Team& team)
{
    std::vector<grSim_Packet> grsim_packets;

    std::pair<Vector, AngularVelocity> robot_velocities;
    grSim_Packet grsim_packet;


    // get timestamp before running bang-bang controller for all robots
    double bangbang_timestamp =
        std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    double current_time;

    // TODO: Implement this
    // https://github.com/UBC-Thunderbots/Software/issues/99
    for (auto& prim : primitives)
    {
        MovePrimitive movePrim = dynamic_cast<MovePrimitive&>(*prim);

        current_time = std::chrono::system_clock::to_time_t(
            std::chrono::system_clock::now());  // get the current time right before
                                                // running bang-bang to get a time-delta
                                                // for acceleration

        MotionController::grSimBangBang(
            *team.getRobotById(movePrim.getRobotId()), movePrim.getDestination(),
            movePrim.getFinalSpeed(), movePrim.getFinalAngle(),
            current_time - bangbang_timestamp);

        // send the velocity data via grsim_packet
        grsim_packet = createGrSimPacket(movePrim.getRobotId(), YELLOW,
                                         robot_velocities.first, robot_velocities.second);
    }

    sendGrSimPacket(grsim_packet);
}
