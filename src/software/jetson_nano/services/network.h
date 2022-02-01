#pragma once

#include "proto/robot_status_msg.pb.h"
#include "proto/tbots_software_msgs.pb.h"
#include "shared/robot_constants.h"
#include "software/jetson_nano/services/service.h"
#include "software/networking/threaded_proto_udp_listener.hpp"
#include "software/networking/threaded_proto_udp_sender.hpp"

class NetworkService : public Service
{
   public:
    /**
     * Service that communicates with our AI
     * Opens all the required ports and maintains them until destroyed.
     *
     * @param ip_address The IP Address the service should connect to
     * @param vision_listener_port The port to listen for vision protos
     * @param primitive_listener_port The port to listen for primitive protos
     * @param robot_status_sender_port The port to send robot status
     * @param multicast  If true, then the provided IP address is a multicast address and
     * we should join the group
     */
    NetworkService(const std::string& ip_address, unsigned short vision_listener_port,
                   unsigned short primitive_listener_port,
                   unsigned short robot_status_sender_port, bool multicast);

    /**
     * Starts the network service
     */
    void start() override;

    /**
     * Disable the network service
     */
    void stop() override;

    /**
     * When the network service is polled, it sends the robot_status and returns
     * a tuple of the most recent PrimitiveSet and Vision
     *
     * @returns a tuple of the stored primitive_set and vision
     */
    std::tuple<TbotsProto::PrimitiveSet, TbotsProto::Vision> poll(
        const TbotsProto::RobotStatus& robot_status);

   private:
    // Variables
    TbotsProto::PrimitiveSet primitive_set_msg;
    TbotsProto::Vision vision_msg;
    std::unique_ptr<ThreadedProtoUdpSender<TbotsProto::RobotStatus>> sender;
    std::unique_ptr<ThreadedProtoUdpListener<TbotsProto::PrimitiveSet>>
        listener_primitive_set;
    std::unique_ptr<ThreadedProtoUdpListener<TbotsProto::Vision>> listener_vision;
    // Functions to callback primitiveSet and vision and stores them in a variable
    void primitiveSetCallback(TbotsProto::PrimitiveSet input);
    void visionCallback(TbotsProto::Vision input);
};
