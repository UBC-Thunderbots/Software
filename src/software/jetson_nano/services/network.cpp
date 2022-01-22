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

void NetworkService::start() {}
NetworkService::~NetworkService() {}

std::tuple<TbotsProto::PrimitiveSet, TbotsProto::Vision> NetworkService::poll(
    const TbotsProto::RobotStatus& robot_status)
{
    sender->sendProto(robot_status);
    return std::make_tuple<TbotsProto::PrimitiveSet, TbotsProto::Vision>(
        std::move(primitive_set_info), std::move(vision_info));
}


void NetworkService::stop()
{
    // Todo:
}

void NetworkService::primitiveSetCallback(TbotsProto::PrimitiveSet input)
{
    primitive_set_info = input;
}
void NetworkService::visionCallback(TbotsProto::Vision input)
{
    vision_info = input;
}
