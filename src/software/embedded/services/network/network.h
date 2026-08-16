#pragma once

#include <mutex>
#include <queue>

#include "proto/ip_notification.pb.h"
#include "proto/primitive.pb.h"
#include "proto/robot_log_msg.pb.h"
#include "proto/robot_status_msg.pb.h"
#include "shared/constants.h"
#include "shared/robot_constants.h"
#include "software/embedded/services/network/primitive_tracker.h"
#include "software/networking/udp/threaded_proto_udp_listener.hpp"
#include "software/networking/udp/threaded_proto_udp_sender.hpp"
#include "software/time/duration.h"
#include "software/time/timestamp.h"
#include "software/world/robot_state.h"

class NetworkService
{
   public:
    /**
     * Configuration for the NetworkService.
     */
    struct NetworkConfig
    {
        // The robot id of the robot
        RobotId robot_id;

        // The IP address of the multicast group used for IP discovery notifications
        std::string multicast_ip;

        // The port to listen for primitive protos
        unsigned short primitive_listener_port;

        // The port to send robot status
        unsigned short robot_status_sender_port;

        // The port to listen for full system IP discovery notification
        unsigned short full_system_to_robot_ip_notification_port;

        // The port to send robot IP discovery notification
        unsigned short robot_to_full_system_ip_notification_port;

        // The interface to listen and send on
        std::string interface;
    };

    /**
     * Service that communicates with our AI
     * Opens all the required ports and maintains them until destroyed.
     *
     * @param config The configuration for the network service
     */
    explicit NetworkService(const NetworkConfig& config);

    /**
     * When the network service is polled, it sends the robot_status and returns
     * a tuple of the most recent Primitive
     *
     * @returns a tuple of the stored primitive
     */
    std::optional<TbotsProto::Primitive> poll(TbotsProto::RobotStatus& robot_status);

   private:
    /**
     * Wait for networking communication to be established. This function is blocking.
     */
    void waitForNetworkUp();

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
     * Handler for received primitive packets
     *
     * @param input The primitive packet received
     */
    void primitiveCallback(const TbotsProto::Primitive& input);

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
    void sendRobotStatus(TbotsProto::RobotStatus& robot_status);

    // Constants
    static constexpr unsigned int ROBOT_STATUS_BROADCAST_RATE_HZ = 30;
    static constexpr double ROBOT_STATUS_TO_THUNDERLOOP_HZ_RATIO =
        ROBOT_STATUS_BROADCAST_RATE_HZ / (THUNDERLOOP_HZ + 1.0);
    static constexpr int IP_DISCOVERY_NOTIFICATION_RATE_HZ = 1 * THUNDERLOOP_HZ;

    // 500 millisecond timeout on receiving primitives before we stop the robots
    static constexpr double PACKET_TIMEOUT_NS = 500.0 * NANOSECONDS_PER_MILLISECOND;

    // Timeout after a failed ping request
    static constexpr int PING_RETRY_DELAY_S = 1;

    // Mutex protects the primitive message
    std::mutex primitive_mutex;
    TbotsProto::Primitive primitive_msg;
    bool new_primitive_msg_received;

    // Mutex protects the fullsystem IP address
    std::mutex fullsystem_ip_mutex;
    std::optional<std::string> fullsystem_ip;

    // Mutex protects the robot status sender
    std::mutex robot_status_sender_mutex;
    std::unique_ptr<ThreadedProtoUdpSender<TbotsProto::RobotStatus>> robot_status_sender;

    std::unique_ptr<ThreadedProtoUdpListener<TbotsProto::IpNotification>>
        fullsystem_to_robot_ip_listener;
    std::unique_ptr<ThreadedProtoUdpSender<TbotsProto::IpNotification>>
        robot_to_fullsystem_ip_sender;
    std::unique_ptr<ThreadedProtoUdpListener<TbotsProto::Primitive>>
        udp_listener_primitive;
    std::shared_ptr<ThreadedProtoUdpSender<TbotsProto::RobotLog>> robot_log_sender;

    // The IP address of the multicast group used for IP discovery notifications
    std::string multicast_ip;

    // The network interface to listen and send messages on
    std::string interface;

    // Port to send robot status messages
    unsigned short robot_status_sender_port;

    // Counters for tracking rate-limited events
    unsigned int ip_notification_ticks = 0;
    unsigned int network_ticks         = 0;
    unsigned int thunderloop_ticks     = 0;

    // Tracks packet loss, round-trip time, and last-received time of primitives
    PrimitiveTracker primitive_tracker;

    // track last breakbeam state for sending RobotStatus outside of specified rate
    bool last_breakbeam_state_sent = false;

    // IP discovery message to send on the network
    TbotsProto::IpNotification robot_ip_notification_msg;
};
