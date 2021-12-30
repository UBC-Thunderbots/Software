#include "software/networking/threaded_proto_udp_sender.hpp"
#include "software/networking/threaded_proto_udp_listener.hpp"
#include "proto/robot_status_msg.pb.h"
#include "software/jetson_nano/services/network.h"



NetworkService::NetworkService(const std::string& ip_address, 
                            unsigned short port,
                            bool multicast,
                            const std::string& ip_addresslistener, 
                            unsigned short portlistener)
{
    sender = std::make_unique<ThreadedProtoUdpSender<TbotsProto::RobotStatus>>(ip_address, port, multicast);
    listenerPrimitiveSet = std::make_unique<ThreadedProtoUdpListener<TbotsProto::PrimitiveSet>>(ip_addresslistener, portlistener, boost::bind(&NetworkService::primitiveSetCallback, this, _1), multicast);
    listenerVision = std::make_unique<ThreadedProtoUdpListener<TbotsProto::Vision>>(ip_addresslistener, portlistener, boost::bind(&NetworkService::visionCallback, this, _1), multicast);

}

void NetworkService::start(){

}

//std::tuple<TbotsProto::PrimitiveSet, TbotsProto::Vision> 
void NetworkService::poll(const TbotsProto::RobotStatus & robot_status){

    sender->sendProto(robot_status);
    //TbotsProto::PrimitiveSet emptyPrimitiveSet;
    //TbotsProto::Vision emptyVision;
    //return std::make_tuple<TbotsProto::PrimitiveSet, TbotsProto::Vision>(emptyPrimitiveSet, emptyVision);

       
}


void NetworkService::stop(){
    //Todo:

}

void NetworkService::primitiveSetCallback(TbotsProto::PrimitiveSet input){
    std::cout<<"Receive out";
}
void NetworkService::visionCallback(TbotsProto::Vision input){
    std::cout<<"Received vision";
}

