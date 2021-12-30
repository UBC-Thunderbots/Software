#pragma once

#include "shared/robot_constants.h"
#include "software/jetson_nano/services/service.h"
#include "proto/tbots_software_msgs.pb.h"
#include "software/networking/threaded_proto_udp_sender.hpp"
#include "software/networking/threaded_proto_udp_listener.hpp"

class NetworkService: public Service
{

    NetworkService(const std::string& ip_address, 
                unsigned short port,
                bool multicast,
                const std::string& ip_addresslistener, 
                unsigned short portlistener);
    
    virtual ~NetworkService();

    //std::tuple<TbotsProto::PrimitiveSet, TbotsProto::Vision> 
    void poll(const TbotsProto::RobotStatus & robot_status);

    void start() override;

    void stop() override;
    private:
        std::unique_ptr<ThreadedProtoUdpSender<TbotsProto::RobotStatus>> sender;
        std::unique_ptr<ThreadedProtoUdpListener<TbotsProto::PrimitiveSet>> listenerPrimitiveSet; 
        std::unique_ptr<ThreadedProtoUdpListener<TbotsProto::Vision>> listenerVision;
        void primitiveSetCallback(TbotsProto::PrimitiveSet input);
        void visionCallback(TbotsProto::Vision input);

};