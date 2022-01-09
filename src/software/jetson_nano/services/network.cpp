#include "software/networking/threaded_proto_udp_sender.hpp"
#include "software/networking/threaded_proto_udp_listener.hpp"
#include "software/jetson_nano/services/network.h"



NetworkService::NetworkService(const std::string& ip_address, 
                            unsigned short vision_listener_port,
                            unsigned short primitive_listener_port,
                            unsigned short robot_status_sender_port,
                            bool multicast)
{
    sender = std::make_unique<ThreadedProtoUdpSender<TbotsProto::RobotStatus>>(ip_address, robot_status_sender_port, multicast);
    listenerPrimitiveSet = std::make_unique<ThreadedProtoUdpListener<TbotsProto::PrimitiveSet>>(ip_address, primitive_listener_port, boost::bind(&NetworkService::primitiveSetCallback, this, _1), multicast);
    listenerVision = std::make_unique<ThreadedProtoUdpListener<TbotsProto::Vision>>(ip_address, vision_listener_port, boost::bind(&NetworkService::visionCallback, this, _1), multicast);

}

void NetworkService::start(){

}
NetworkService::~NetworkService(){}

std::tuple<TbotsProto::PrimitiveSet, TbotsProto::Vision> NetworkService::poll(const TbotsProto::RobotStatus & robot_status){

    sender->sendProto(robot_status);
    //return saved primitive and vision input
    TbotsProto::PrimitiveSet emptyPrimitiveSet;
    TbotsProto::Vision emptyVision;

    return std::make_tuple<TbotsProto::PrimitiveSet, TbotsProto::Vision>(std::move(emptyPrimitiveSet), std::move(emptyVision));

       
}


void NetworkService::stop(){
    //Todo:

}

void NetworkService::primitiveSetCallback(TbotsProto::PrimitiveSet input){
    //save input to a private variable in networking class
    std::cout<<"Receive out\n";

}
void NetworkService::visionCallback(TbotsProto::Vision input){
    //save input to a private variable in networking class
    std::cout<<"Received vision\n";
}

