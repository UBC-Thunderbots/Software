#include "software/jetson_nano/services/network.h"

#include "software/networking/threaded_proto_udp_listener.hpp"
#include "software/networking/threaded_proto_udp_sender.hpp"



NetworkService::NetworkService(const std::string& ip_address,
                               unsigned short world_listener_port,
                               unsigned short primitive_listener_port,
                               unsigned short robot_status_sender_port, 
                               bool multicast)
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

    // LOG(DEBUG) << "interpacket delay primitives: " <<
    // input.time_sent().epoch_timestamp_seconds() - last_primitive_time;

    last_primitive_time     = input.time_sent().epoch_timestamp_seconds();
    static uint64_t counter = 0;
    counter++;
    // if (last_sequence_number != input.sequence_number() - 1)
    //{
    // LOG(DEBUG) << "PACKET LOST YOU CHUMP";
    //}
    last_sequence_number = input.sequence_number();
}

void NetworkService::worldCallback(TbotsProto::World input)
{
    std::scoped_lock<std::mutex> lock(world_mutex);
    world_msg = input;

    last_world_time = input.time_sent().epoch_timestamp_seconds();
}
