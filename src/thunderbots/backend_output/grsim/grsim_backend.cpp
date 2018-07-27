#include "backend_output/grsim/grsim_backend.h"
#include <iostream>
#include <optional>
#include "ai/primitive/move_prim.h"
#include "proto/grSim_Commands.pb.h"
#include "proto/grSim_Packet.pb.h"

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

void GrSimBackend::setRobotVelocities(
    unsigned int robot_id, bool team_colour_yellow, Point velocity,
    Angle angular_velocity)
{
    grSim_Packet packet;

    packet.mutable_commands()->set_isteamyellow(team_colour_yellow);
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

    // Send the packet via UDP
    boost::system::error_code err;
    socket.send_to(
        buffer(packet.SerializeAsString(), static_cast<size_t>(packet.ByteSize())),
        remote_endpoint, 0, err);
}

void GrSimBackend::sendPrimitives(const std::vector<Primitive>& primitives)
{
    setRobotVelocities(0, false, Point(0.5, -0.1), Angle::ofRadians(-0.8));
}
