#pragma once

#include <deque>
#include <optional>
#include <queue>

#include "proto/primitive.pb.h"
#include "proto/robot_status_msg.pb.h"

/**
 * Tracks received primitives; calculates packet loss rate and the round-trip time data
 * that Thunderscope needs.
 */
class PrimitiveTracker
{
   public:
    explicit PrimitiveTracker();

    /**
     * Records a newly received primitive. Out-of-order (older or duplicate) primitives
     * are ignored.
     *
     * @param primitive The newly received primitive
     */
    void track(const TbotsProto::Primitive& primitive);

    /**
     * Reports the fraction of recently received primitives that were lost.
     *
     * @return the packet loss rate (0.0 to 1.0) of recent primitives
     */
    float getPrimitiveLossRate() const;

    /**
     * Returns the latest valid primitive, at most once. Subsequent calls return
     * std::nullopt until a newer primitive is received.
     *
     * @return the latest valid primitive, or std::nullopt
     */
    std::optional<TbotsProto::Primitive> getLatestPrimitive();

    /**
     * Updates the robot status with the timestamp Thunderscope uses to calculate the
     * round-trip time of the most recently handled primitive.
     *
     * @param robot_status The robot status to update
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
     * Computes the loss rate from the sequence numbers of recently received primitives.
     *
     * @param seq_num The sequence number of the newly received primitive
     * @return the loss rate (0.0 to 1.0)
     */
    float calculateLossRate(uint64_t seq_num) const;

    /**
     * Returns the current epoch time, used to timestamp received primitives.
     *
     * @return the current epoch time in seconds
     */
    static double getCurrentEpochTimeInSeconds();

    // Minimum distance away from latest known sequence number for primitive
    // to be considered out of date
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

    // The latest valid primitive
    std::optional<TbotsProto::Primitive> latest_primitive;
};
