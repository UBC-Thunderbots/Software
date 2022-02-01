#include "software/jetson_nano/services/network.h"

#include "software/networking/threaded_proto_udp_listener.hpp"
#include "software/networking/threaded_proto_udp_sender.hpp"



NetworkService::NetworkService(const std::string& ip_address,
                               unsigned short vision_listener_port,
                               unsigned short primitive_listener_port,
                               unsigned short robot_status_sender_port, bool multicast)
{
    sender = std::make_unique<ThreadedProtoUdpSender<TbotsProto::RobotStatus>>(
        ip_address, robot_status_sender_port, multicast);
    listener_primitive_set =
        std::make_unique<ThreadedProtoUdpListener<TbotsProto::PrimitiveSet>>(
            ip_address, primitive_listener_port,
            boost::bind(&NetworkService::primitiveSetCallback, this, _1), multicast);
    listener_vision = std::make_unique<ThreadedProtoUdpListener<TbotsProto::Vision>>(
        ip_address, vision_listener_port,
        boost::bind(&NetworkService::visionCallback, this, _1), multicast);
}

void NetworkService::start()
{
    // TODO (#2436) remove
}

std::tuple<TbotsProto::PrimitiveSet, TbotsProto::Vision> NetworkService::poll(
    const TbotsProto::RobotStatus& robot_status)
{
    sender->sendProto(robot_status);
    return std::tuple<TbotsProto::PrimitiveSet, TbotsProto::Vision>{primitive_set_msg,
                                                                    vision_msg};
}


void NetworkService::stop()
{
    // TODO (#2436) remove
}

void NetworkService::primitiveSetCallback(TbotsProto::PrimitiveSet input)
{
    primitive_set_msg = input;
}
void NetworkService::visionCallback(TbotsProto::Vision input)
{
    vision_msg = input;
}
