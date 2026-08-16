#include "software/embedded/services/network/primitive_tracker.h"

#include <algorithm>

PrimitiveTracker::PrimitiveTracker()
    : last_primitive_received_time_(std::chrono::steady_clock::now())
{
}

bool PrimitiveTracker::track(const TbotsProto::Primitive& primitive)
{
    const uint64_t seq_num = primitive.sequence_number();

    // Log the primitive for round-trip time calculations
    if (primitive_rtt.size() >= PRIMITIVE_DEQUE_MAX_SIZE)
    {
        LOG(WARNING)
            << "Too many primitive sets logged for round-trip calculations, halting log process";
    }
    else if (!primitive_rtt.empty() &&
             seq_num <= primitive_rtt.back().primitive_sequence_num)
    {
        // If the proto is older than the last received proto, then ignore it
    }
    else
    {
        const RoundTripTime current_round_trip_time{
            .primitive_sequence_num = seq_num,
            .thunderscope_sent_time_seconds =
                primitive.time_sent().epoch_timestamp_seconds(),
            .thunderloop_received_time_seconds = getCurrentEpochTimeInSeconds(),
        };

        primitive_rtt.emplace_back(current_round_trip_time);
    }

    // Track packet loss from the sequence number
    if (!recent_proto_seq_nums.empty() &&
        seq_num + RECENT_PROTO_LOSS_PERIOD <= recent_proto_seq_nums.back())
    {
        // If the proto seems very out of date, then this is likely due to an AI reset.
        // Clear the queue
        recent_proto_seq_nums = std::queue<uint64_t>();
        LOG(WARNING) << "Old primitive set received. Resetting sequence number tracking.";
    }
    else if (!recent_proto_seq_nums.empty() && seq_num <= recent_proto_seq_nums.back())
    {
        // If the proto is older than the last received proto, then ignore it
        last_valid = false;
        return false;
    }

    recent_proto_seq_nums.push(seq_num);

    // Pop sequence numbers of protos that are no longer recent
    while (seq_num - recent_proto_seq_nums.front() >= RECENT_PROTO_LOSS_PERIOD)
    {
        recent_proto_seq_nums.pop();
    }

    proto_loss_rate = calculateProtoLossRate(seq_num);
    last_valid      = true;

    last_primitive_received_time_ = std::chrono::steady_clock::now();

    return true;
}

float PrimitiveTracker::getPacketLoss() const
{
    return proto_loss_rate;
}

double PrimitiveTracker::getAverageRoundTripTime() const
{
    if (primitive_rtt.empty())
    {
        return 0.0;
    }

    double total_round_trip_time_seconds = 0.0;
    for (const auto& round_trip_time : primitive_rtt)
    {
        total_round_trip_time_seconds +=
            round_trip_time.thunderloop_received_time_seconds -
            round_trip_time.thunderscope_sent_time_seconds;
    }

    return total_round_trip_time_seconds / static_cast<double>(primitive_rtt.size());
}

std::chrono::steady_clock::time_point PrimitiveTracker::getLastPrimitiveReceivedTime()
    const
{
    return last_primitive_received_time_;
}

void PrimitiveTracker::updatePrimitiveLog(TbotsProto::RobotStatus& robot_status)
{
    const uint64_t seq_num = robot_status.last_handled_primitive_set();
    while (!primitive_rtt.empty())
    {
        if (primitive_rtt.front().primitive_sequence_num == seq_num)
        {
            double received_epoch_time_seconds =
                primitive_rtt.front().thunderloop_received_time_seconds;
            double processing_time_seconds =
                getCurrentEpochTimeInSeconds() - received_epoch_time_seconds;

            robot_status.mutable_adjusted_time_sent()->set_epoch_timestamp_seconds(
                primitive_rtt.front().thunderscope_sent_time_seconds +
                processing_time_seconds);
            return;
        }
        primitive_rtt.pop_front();
    }
}

float PrimitiveTracker::calculateProtoLossRate(const uint64_t seq_num) const
{
    // seq_num + 1 is to account for the sequence numbers starting from 0
    const uint64_t expected_count =
        std::min(seq_num + 1, static_cast<uint64_t>(RECENT_PROTO_LOSS_PERIOD));
    const uint64_t lost_count = expected_count - recent_proto_seq_nums.size();
    return static_cast<float>(lost_count) / static_cast<float>(expected_count);
}

double PrimitiveTracker::getCurrentEpochTimeInSeconds()
{
    using Seconds     = std::chrono::duration<double>;
    const Seconds now = std::chrono::duration_cast<Seconds>(
        std::chrono::system_clock::now().time_since_epoch());
    return now.count();
}
