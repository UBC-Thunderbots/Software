#pragma once

#include <mutex>
#include <queue>

#include "proto/ip_notification.pb.h"
#include "proto/primitive.pb.h"
#include "proto/robot_status_msg.pb.h"
#include "shared/constants.h"
#include "shared/robot_constants.h"
#include "software/embedded/services/network/proto_tracker.h"
#include "software/networking/radio/threaded_proto_radio_listener.hpp"
#include "software/networking/udp/threaded_proto_udp_listener.hpp"
#include "software/networking/udp/threaded_proto_udp_sender.hpp"
#include "software/time/duration.h"
#include "software/time/timestamp.h"
#include "software/world/robot_state.h"

class NetworkService
{
   public:
    /**
     * Service that communicates with our AI
     * Opens all the required ports and maintains them until destroyed.
     *
     * @param robot_id The robot id of the robot
     * @param ip_address The IP Address the service should connect to
     * @param primitive_listener_port The port to listen for primitive protos
     * @param full_system_to_robot_ip_notification_port The port to listen for full system IP discovery notification
     * @param robot_to_full_system_ip_notification_port The port to send robot IP discovery notification
     * @param robot_status_sender_port The port to send robot status
     * @param interface the interface to listen and send on
     */
    NetworkService(const RobotId& robot_id, const std::string& ip_address, unsigned short primitive_listener_port,
                   unsigned short robot_status_sender_port, unsigned short full_system_to_robot_ip_notification_port,
                  unsigned short robot_to_full_system_ip_notification_port, const std::string& interface);

    /**
     * When the network service is polled, it sends the robot_status and returns
     * a tuple of the most recent Primitive
     *
     * @returns a tuple of the stored primitive
     */
    TbotsProto::Primitive poll(TbotsProto::RobotStatus& robot_status);

   private:
    /**
     * Return true if a robot status message should be sent over the network.
     *
     * The update is required if any of the following are necessary:
     * 1. Any motor has a motor fault.
     * 2. The breakbeam status has changed between subsequent messages.
     * 3. If we have not sent back a robot_status message in a while (heartbeat).
     *
     * @param robot_status the current robot status containing all the feedback
     *
     * @returns true if an update to the network is required, false otherwise
     */
    bool shouldSendNewRobotStatus(const TbotsProto::RobotStatus& robot_status) const;

    /**
     * Tracks the given primitive for calculating round-trip time if valid
     *
     * @param input A potential primitive to be logged
     */
    void logNewPrimitive(const TbotsProto::Primitive& new_primitive);

    /**
     * Updates the cached primitive for Thunderscope to calculate round-trip time
     *
     * @param robot_status The robot status to compare to within the cache
     */
    void updatePrimitiveLog(TbotsProto::RobotStatus& robot_status);

    /**
     * Getter for the current epoch time in seconds as a double
     *
     * @return current epoch time in seconds as a double
     */
    double getCurrentEpochTimeInSeconds();

    // Constants
    static constexpr unsigned int ROBOT_STATUS_BROADCAST_RATE_HZ = 30;
    static constexpr double ROBOT_STATUS_TO_THUNDERLOOP_HZ_RATIO =
        ROBOT_STATUS_BROADCAST_RATE_HZ / (THUNDERLOOP_HZ + 1.0);

    // increases size of deque when robot status messages are sent less frequently
    static constexpr unsigned int PRIMITIVE_DEQUE_MAX_SIZE =
        static_cast<unsigned int>(1500 / ROBOT_STATUS_BROADCAST_RATE_HZ);

    // Variables

    // Mutex protects the primitive message
    std::mutex primitive_mutex;
    TbotsProto::Primitive primitive_msg;

    // Mutex protects the fullsystem IP address
    std::mutex fullsystem_ip_mutex;
    std::optional<std::string> fullsystem_ip;

    // Mutex protects the robot status sender
    std::mutex robot_status_sender_mutex;
    std::unique_ptr<ThreadedProtoUdpSender<TbotsProto::RobotStatus>> robot_status_sender;

    std::unique_ptr<ThreadedProtoUdpListener<TbotsProto::IpNotification>> fullsystem_to_robot_ip_listener;
    std::unique_ptr<ThreadedProtoUdpSender<TbotsProto::IpNotification>> robot_to_fullsystem_ip_sender;
    std::unique_ptr<ThreadedProtoUdpListener<TbotsProto::Primitive>>
        udp_listener_primitive;
    std::unique_ptr<ThreadedProtoRadioListener<TbotsProto::Primitive>>
        radio_listener_primitive_set;

    std::string interface;

    unsigned short robot_status_sender_port;
    unsigned int network_ticks     = 0;
    unsigned int thunderloop_ticks = 0;

    /**
     * Handler for received primitive packets
     *
     * @param input The primitive packet received
     */
    void primitiveCallback(TbotsProto::Primitive input);

    /**
     * Handler for received full system IP notification packets
     *
     * @param ip_notification The IP notification packet received
     */
    void onFullSystemIpNotification(const TbotsProto::IpNotification& ip_notification);

    /**
     * Send a robot status message over the network
     *
     * @param robot_status The robot status message to send
     */
    void sendRobotStatus(const TbotsProto::RobotStatus& robot_status);

    // ProtoTrackers for tracking recent primitive_set packet loss
    ProtoTracker primitive_tracker;

    // track last breakbeam state for sending RobotStatus outside of specified rate
    bool last_breakbeam_state_sent = false;

    struct RoundTripTime
    {
        // Primitive Sequence Number
        uint64_t primitive_sequence_num = 0;
        // Epoch time of primitive set sent time from Thunderscope in seconds
        double thunderscope_sent_time_seconds = 0;
        // System time for when primitive set was received by Thunderloop in seconds
        double thunderloop_recieved_time_seconds = 0;
    };

    std::deque<RoundTripTime> primitive_set_rtt;

    TbotsProto::IpNotification robot_ip_notification_msg;
};
