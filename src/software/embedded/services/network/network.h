#pragma once

#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "proto/ip_notification.pb.h"
#include "proto/primitive.pb.h"
#include "proto/robot_log_msg.pb.h"
#include "proto/robot_status_msg.pb.h"
#include "software/embedded/services/network/primitive_tracker.h"
#include "software/networking/udp/threaded_proto_udp_listener.hpp"
#include "software/networking/udp/threaded_proto_udp_sender.hpp"
#include "software/world/robot_state.h"

/**
 * A service that handles communication with fullsystem over the network.
 * We receive primitives from fullsystem and send back the robot status.
 */
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

        // The network interface to listen and send on
        std::string interface;
    };

    /**
     * Constructs a NetworkService and opens all the required network ports.
     *
     * @param config The configuration for the network service
     */
    explicit NetworkService(const NetworkConfig& config);

    /**
     * Polls the network service: sends the robot status to fullsystem, and,
     * if a new primitive has been received, returns it.
     *
     * Note that each received primitive is returned at most once by this method;
     * subsequent calls will return std::nullopt until a newer primitive is received.
     *
     * @param robot_status the current robot status to send over the network
     * @param time_elapsed_since_last_poll_s the time in seconds since the last poll
     *
     * @returns the primitive to execute, if there is a new one
     */
    std::optional<TbotsProto::Primitive> poll(TbotsProto::RobotStatus& robot_status,
                                              double time_elapsed_since_last_poll_s);

   private:
    /**
     * Wait for networking communication to be established. This function is blocking.
     */
    void waitForNetworkUp();

    /**
     * Updates the network status fields of robot status from the primitive tracker.
     *
     * @param robot_status the current robot status containing all the feedback
     */
    void updateNetworkStatus(TbotsProto::RobotStatus& robot_status);

    /**
     * Sends the robot status over the network if it is time to do so.
     *
     * @param robot_status the current robot status containing all the feedback
     * @param time_elapsed_since_last_poll_s the time in seconds since the last poll
     */
    void sendRobotStatusIfNeeded(TbotsProto::RobotStatus& robot_status,
                                 double time_elapsed_since_last_poll_s);

    /**
     * Sends an IP discovery notification if it is time to do so.
     *
     * @param time_elapsed_since_last_poll_s the time in seconds since the last poll
     */
    void sendIpNotificationIfNeeded(double time_elapsed_since_last_poll_s);

    /**
     * Returns the latest received primitive, if it is new. If no new primitive has
     * been received since the last time this method was called (i.e. the primitive
     * to execute has not changed), std::nullopt is returned.
     *
     * If we have not received a primitive in a while, StopPrimitive is returned
     * as a safety precaution.
     *
     * @param time_elapsed_since_last_poll_s the time in seconds since the last poll
     *
     * @returns the primitive to execute, if there is a new one
     */
    std::optional<TbotsProto::Primitive> getPrimitiveToExecute(
        double time_elapsed_since_last_poll_s);

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

    // The rate at which to send out robot status / IP notification messages
    static constexpr double ROBOT_STATUS_SEND_INTERVAL_S    = 0.3;
    static constexpr double IP_NOTIFICATION_SEND_INTERVAL_S = 1.0;

    // Timeout on receiving primitives before we stop the robots
    static constexpr double PRIMITIVE_RECEIVE_TIMEOUT_S = 0.5;

    // Timeout after a failed ping request
    static constexpr int PING_RETRY_DELAY_S = 1;

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
    double time_since_last_robot_status_sent_s_    = 0.0;
    double time_since_last_ip_notification_sent_s_ = 0.0;
    double time_since_last_primitive_received_s_   = 0.0;

    // Mutex protects the primitive tracker
    std::mutex primitive_tracker_mutex;
    PrimitiveTracker primitive_tracker;

    // track last breakbeam state for sending RobotStatus outside of specified rate
    bool last_breakbeam_state_sent = false;

    // IP discovery message to send on the network
    TbotsProto::IpNotification robot_ip_notification_msg;
};
