#pragma once

#include "shared/robot_constants.h"
#include "software/jetson_nano/services/service.h"
#include "proto/tbots_software_msgs.pb.h"
#include "proto/robot_status_msg.pb.h"
#include "software/networking/threaded_proto_udp_sender.hpp"
#include "software/networking/threaded_proto_udp_listener.hpp"

class NetworkService: public Service
{
    public:
        NetworkService(const std::string& ip_address, 
                            unsigned short vision_listener_port,
                            unsigned short primitive_listener_port,
                            unsigned short robot_status_sender_port,
                            bool multicast);
    
        virtual ~NetworkService();

        std::tuple<TbotsProto::PrimitiveSet, TbotsProto::Vision> poll(const TbotsProto::RobotStatus & robot_status);

        void start() override;

        void stop() override;
    private:
        std::unique_ptr<ThreadedProtoUdpSender<TbotsProto::RobotStatus>> sender;
        std::unique_ptr<ThreadedProtoUdpListener<TbotsProto::PrimitiveSet>> listenerPrimitiveSet; 
        std::unique_ptr<ThreadedProtoUdpListener<TbotsProto::Vision>> listenerVision;
        void primitiveSetCallback(TbotsProto::PrimitiveSet input);
        void visionCallback(TbotsProto::Vision input);

};