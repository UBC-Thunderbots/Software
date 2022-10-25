#pragma once

#include <mutex>
#include <queue>

#include "proto/robot_status_msg.pb.h"
#include "proto/tbots_software_msgs.pb.h"
#include "proto/world.pb.h"
#include "shared/robot_constants.h"
#include "software/networking/threaded_proto_udp_listener.hpp"
#include "software/networking/threaded_proto_udp_sender.hpp"

constexpr float PACKET_LOSS_WARNING_THRESHOLD = 0.1f;
constexpr uint8_t RECENT_PACKET_LOSS_PERIOD   = 100;

class NetworkService
{
   public:
    /**
     * Service that communicates with our AI
     * Opens all the required ports and maintains them until destroyed.
     *
     * @param ip_address The IP Address the service should connect to
     * @param world_listener_port The port to listen for world protos
     * @param primitive_listener_port The port to listen for primitive protos
     * @param robot_status_sender_port The port to send robot status
     * @param multicast  If true, then the provided IP address is a multicast address and
     * we should join the group
     */
    NetworkService(const std::string& ip_address, unsigned short world_listener_port,
                   unsigned short primitive_listener_port,
                   unsigned short robot_status_sender_port, bool multicast);

    /**
     * When the network service is polled, it sends the robot_status and returns
     * a tuple of the most recent PrimitiveSet and World
     *
     * @returns a tuple of the stored primitive_set and world
     */
    std::tuple<TbotsProto::PrimitiveSet, TbotsProto::World> poll(
        const TbotsProto::RobotStatus& robot_status);

   private:
    // Variables
    TbotsProto::PrimitiveSet primitive_set_msg;
    TbotsProto::World world_msg;

    std::mutex primitive_set_mutex;
    std::mutex world_mutex;

    std::unique_ptr<ThreadedProtoUdpSender<TbotsProto::RobotStatus>> sender;
    std::unique_ptr<ThreadedProtoUdpListener<TbotsProto::PrimitiveSet>>
        listener_primitive_set;
    std::unique_ptr<ThreadedProtoUdpListener<TbotsProto::World>> listener_world;


    // Functions to callback primitiveSet and world and stores them in a variable
    void primitiveSetCallback(TbotsProto::PrimitiveSet input);
    void worldCallback(TbotsProto::World input);

    // Queue of the sequence numbers of received primitive sets in the past
    // RECENT_PACKET_LOSS_PERIOD primitive sets
    std::queue<uint64_t> recent_primitive_set_seq_nums;
};
