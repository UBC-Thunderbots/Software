#pragma once

#include <chrono>
#include <cstdint>
#include <deque>
#include <queue>

#include "proto/primitive.pb.h"
#include "proto/robot_status_msg.pb.h"
#include "shared/constants.h"
#include "software/logger/logger.h"

/**
 * Tracks received primitives and reports packet loss and average round-trip time.
 */
class PrimitiveTracker
{
   public:
    explicit PrimitiveTracker();

    /**
     * Records a newly received primitive, updating packet loss, round-trip time, and
     * last-received-time tracking.
     *
     * @param primitive The newly received primitive.
     *
     * @return true if the primitive is newer than the last received one.
     */
    bool track(const TbotsProto::Primitive& primitive);

    /**
     * @return the packet loss rate (0.0 to 1.0) of recent primitives.
     */
    float getPacketLoss() const;

    /**
     * @return the average one-way round-trip time (Thunderscope -> robot delivery
     * latency) in seconds over the tracked primitives, or 0.0 if none are tracked.
     */
    double getAverageRoundTripTime() const;

    /**
     * @return the time the last primitive was received.
     */
    std::chrono::steady_clock::time_point getLastPrimitiveReceivedTime() const;

    /**
     * Matches the robot status's last handled primitive set against the tracked
     * primitives and sets the robot status's adjusted_time_sent, which Thunderscope uses
     * to calculate round-trip time.
     *
     * @param robot_status The robot status containing the last handled primitive set.
     */
    void updatePrimitiveLog(TbotsProto::RobotStatus& robot_status);

   private:
    struct RoundTripTime
    {
        uint64_t primitive_sequence_num = 0;

        // Epoch time of primitive sent time from Thunderscope in seconds
        double thunderscope_sent_time_seconds = 0;

        // System time for when primitive was received by Thunderloop in seconds
        double thunderloop_received_time_seconds = 0;
    };

    /**
     * Private function for calculating the proto loss rate
     *
     * @param seq_num The sequence number of the newly received protobuf
     * @return a float equal to the proto loss rate
     */
    float calculateProtoLossRate(uint64_t seq_num) const;

    /**
     * Getter for the current epoch time in seconds as a double
     *
     * @return current epoch time in seconds as a double
     */
    static double getCurrentEpochTimeInSeconds();

    // Constants
    static constexpr uint8_t RECENT_PROTO_LOSS_PERIOD = 100;

    // Maximum number of primitives to keep for round-trip-time calculations. Sized to
    // cover ~1.5 seconds of primitives at a 30 Hz robot status broadcast rate.
    static constexpr unsigned int PRIMITIVE_DEQUE_MAX_SIZE = 50;

    // Packet loss tracking
    bool last_valid       = false;
    float proto_loss_rate = 0;

    // Queue of the sequence numbers of received protos in the past
    // RECENT_PROTO_LOSS_PERIOD protos
    std::queue<uint64_t> recent_proto_seq_nums;

    // Stores the most recent primitives for calculating round-trip time
    std::deque<RoundTripTime> primitive_rtt;

    // The time the last valid primitive was received
    std::chrono::steady_clock::time_point last_primitive_received_time_;
};
