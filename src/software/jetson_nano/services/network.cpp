#include "software/jetson_nano/services/network.h"

#include "software/networking/threaded_proto_udp_listener.hpp"
#include "software/networking/threaded_proto_udp_sender.hpp"


const float PACKET_LOSS_WARNING_THRESHOLD = 0.1f;

NetworkService::NetworkService(const std::string& ip_address,
                               unsigned short world_listener_port,
                               unsigned short primitive_listener_port,
                               unsigned short robot_status_sender_port, bool multicast)
{
    sender = std::make_unique<ThreadedProtoUdpSender<TbotsProto::RobotStatus>>(
        ip_address, robot_status_sender_port, multicast);
    listener_primitive_set =
        std::make_unique<ThreadedProtoUdpListener<TbotsProto::PrimitiveSet>>(
            ip_address, primitive_listener_port,
            boost::bind(&NetworkService::primitiveSetCallback, this, _1), multicast);
    listener_world = std::make_unique<ThreadedProtoUdpListener<TbotsProto::World>>(
        ip_address, world_listener_port,
        boost::bind(&NetworkService::worldCallback, this, _1), multicast);
}

std::tuple<TbotsProto::PrimitiveSet, TbotsProto::World> NetworkService::poll(
    const TbotsProto::RobotStatus& robot_status)
{
    std::scoped_lock lock{primitive_set_mutex, world_mutex};
    TbotsProto::RobotStatus new_status = robot_status;
    new_status.set_last_handled_primitive_set(primitive_set_msg.sequence_number());
    sender->sendProto(robot_status);
    return std::tuple<TbotsProto::PrimitiveSet, TbotsProto::World>{primitive_set_msg,
                                                                   world_msg};
}

void NetworkService::primitiveSetCallback(TbotsProto::PrimitiveSet input)
{
    std::scoped_lock<std::mutex> lock(primitive_set_mutex);
    primitive_set_msg = input;

    // TODO(#2727): Implement a recent packet loss instead of overall
    total_primitives_lost += input.sequence_number() - last_primitive_sequence_number - 1;

    float packet_loss = static_cast<float>(total_primitives_lost) /
                        static_cast<float>(input.sequence_number());

    if (packet_loss > PACKET_LOSS_WARNING_THRESHOLD)
    {
        LOG(WARNING) << "Primitive packet loss more than "
                     << PACKET_LOSS_WARNING_THRESHOLD << "% ";
    }

    last_primitive_sequence_number = input.sequence_number();
}

void NetworkService::worldCallback(TbotsProto::World input)
{
    std::scoped_lock<std::mutex> lock(world_mutex);
    world_msg = input;
    // TODO(#2728): Implement a recent world loss count and warning

    last_world_time = input.time_sent().epoch_timestamp_seconds();
}
