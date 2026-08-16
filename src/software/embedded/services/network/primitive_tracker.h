#pragma once

#include <chrono>
#include <cstdint>
#include <deque>
#include <optional>
#include <queue>

#include "proto/primitive.pb.h"
#include "proto/robot_status_msg.pb.h"
#include "shared/constants.h"
#include "software/logger/logger.h"

/**
 * Tracks received primitives: packet loss, the latest received primitive, and the
 * round-trip-time data that Thunderscope needs.
 */
class PrimitiveTracker
{
   public:
    explicit PrimitiveTracker();

    /**
     * Records a newly received primitive. Out-of-order (older or duplicate) primitives
     * are ignored.
     *
     * @param primitive The newly received primitive.
     */
    void track(const TbotsProto::Primitive& primitive);

    /**
     * @return the packet loss rate (0.0 to 1.0) of recent primitives.
     */
    float getPrimitiveLossRate() const;

    /**
     * Returns the latest valid primitive, at most once. Subsequent calls return
     * std::nullopt until a newer primitive is received.
     *
     * @return the latest valid primitive, or std::nullopt.
     */
    std::optional<TbotsProto::Primitive> getLatestPrimitive();

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
    float calculateLossRate(uint64_t seq_num) const;

    /**
     * Getter for the current epoch time in seconds as a double
     *
     * @return current epoch time in seconds as a double
     */
    static double getCurrentEpochTimeInSeconds();

    // Constants
    static constexpr uint8_t RECENT_PRIMITIVE_LOSS_PERIOD = 100;

    // Maximum number of primitives to keep for round-trip-time calculations. Sized to
    // cover ~1.5 seconds of primitives at a 30 Hz robot status broadcast rate.
    static constexpr unsigned int PRIMITIVE_DEQUE_MAX_SIZE = 50;

    // Packet loss tracking: sequence numbers of the most recent primitives
    std::queue<uint64_t> recent_proto_seq_nums;
    float proto_loss_rate = 0;

    // Round-trip time tracking: the most recent primitives used to compute
    // adjusted_time_sent for Thunderscope
    std::deque<RoundTripTime> primitive_rtt;

    // The latest valid primitive, consumed once by getLatestPrimitive
    std::optional<TbotsProto::Primitive> latest_primitive;
};
