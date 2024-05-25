#pragma once

#include <mutex>
#include <queue>

#include "proto/robot_status_msg.pb.h"
#include "proto/tbots_software_msgs.pb.h"
#include "shared/constants.h"
#include "shared/robot_constants.h"
#include "software/jetson_nano/services/network/proto_tracker.h"
#include "software/networking/radio/threaded_proto_radio_listener.hpp"
#include "software/networking/udp/threaded_proto_udp_listener.hpp"
#include "software/networking/udp/threaded_proto_udp_sender.hpp"
#include "software/time/duration.h"
#include "software/time/timestamp.h"

class NetworkService
{
   public:
    /**
     * Service that communicates with our AI
     * Opens all the required ports and maintains them until destroyed.
     *
     * @param ip_address The IP Address the service should connect to
     * @param primitive_listener_port The port to listen for primitive protos
     * @param robot_status_sender_port The port to send robot status
     * @param multicast  If true, then the provided IP address is a multicast address and
     * we should join the group
     */
    NetworkService(const std::string& ip_address, unsigned short primitive_listener_port,
                   unsigned short robot_status_sender_port, bool multicast);

    /**
     * When the network service is polled, it sends the robot_status and returns
     * a tuple of the most recent PrimitiveSet
     *
     * @returns a tuple of the stored primitive_set
     */
    TbotsProto::PrimitiveSet poll(TbotsProto::RobotStatus& robot_status);

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
     * Tracks the given primitive set for calculating round-trip time if valid
     *
     * @param input A potential primitive set to be logged
     */
    void logNewPrimitiveSet(TbotsProto::PrimitiveSet input);

    /**
     * Updates the cached primitive sets for Thunderscope to calculate round-trip time
     *
     * @param robot_status The robot status to compare to within the cache
     */
    void updatePrimitiveSetLog(TbotsProto::RobotStatus& robot_status);

    /**
     * Getter for the current epoch time in seconds as a double
     *
     * @return current epoch time in seconds as a double
     */
    double getCurrentEpochTimeInSeconds();

    // Constants
    static constexpr float PROTO_LOSS_WARNING_THRESHOLD          = 0.1f;
    static constexpr unsigned int ROBOT_STATUS_BROADCAST_RATE_HZ = 30;
    static constexpr double ROBOT_STATUS_TO_THUNDERLOOP_HZ_RATIO =
        ROBOT_STATUS_BROADCAST_RATE_HZ / (THUNDERLOOP_HZ + 1.0);

    // Variables
    TbotsProto::PrimitiveSet primitive_set_msg;

    std::mutex primitive_set_mutex;

    std::unique_ptr<ThreadedProtoUdpSender<TbotsProto::RobotStatus>> sender;
    std::unique_ptr<ThreadedProtoUdpListener<TbotsProto::PrimitiveSet>>
        udp_listener_primitive_set;
    std::unique_ptr<ThreadedProtoRadioListener<TbotsProto::PrimitiveSet>>
        radio_listener_primitive_set;

    unsigned int network_ticks     = 0;
    unsigned int thunderloop_ticks = 0;

    // Callback function for storing the received primitive_sets
    void primitiveSetCallback(TbotsProto::PrimitiveSet input);

    // ProtoTrackers for tracking recent primitive_set packet loss
    ProtoTracker primitive_tracker;

    // track last breakbeam state for sending RobotStatus outside of specified rate
    bool last_breakbeam_state_sent = false;

    /**
     * Struct of key primitive set information as for round-trip time calculations:
     * sequence_num: Primitive Sequence Number
     * thunderloop_recieved_time_seconds: System time for when primitive set was received by Thunderloop in seconds
     * thunderscope_sent_time_seconds: Epoch time of primitive set sent time from Thunderscope in seconds
     */
    struct RoundTripTime {
        uint64_t primitive_sequence_num;
        double thunderscope_sent_time_seconds;
        double thunderloop_recieved_time_seconds;
    };

    std::deque<RoundTripTime> primitive_set_rtt;
};
