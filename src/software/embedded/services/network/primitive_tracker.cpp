#include "software/embedded/services/network/primitive_tracker.h"

#include <algorithm>

#include "software/logger/logger.h"

PrimitiveTracker::PrimitiveTracker() {}

void PrimitiveTracker::track(const TbotsProto::Primitive& primitive)
{
    const uint64_t seq_num = primitive.sequence_number();

    // If the proto seems very out of date, then this is likely due to an AI reset.
    // Reset all tracking so we start fresh.
    if (!recent_proto_seq_nums.empty() &&
        seq_num + RECENT_PRIMITIVE_LOSS_PERIOD <= recent_proto_seq_nums.back())
    {
        recent_proto_seq_nums = std::queue<uint64_t>();
        primitive_rtt.clear();
        LOG(WARNING) << "Old primitive set received. Resetting sequence number tracking.";
    }
    else if (!recent_proto_seq_nums.empty() && seq_num <= recent_proto_seq_nums.back())
    {
        // If the proto is older than the last received proto, then ignore it
        return;
    }

    // Update packet loss tracking
    recent_proto_seq_nums.push(seq_num);
    // Pop sequence numbers of protos that are no longer recent
    while (seq_num - recent_proto_seq_nums.front() >= RECENT_PRIMITIVE_LOSS_PERIOD)
    {
        recent_proto_seq_nums.pop();
    }
    proto_loss_rate = calculateLossRate(seq_num);

    // Log the primitive for round-trip time calculations
    if (primitive_rtt.size() >= PRIMITIVE_DEQUE_MAX_SIZE)
    {
        LOG(WARNING)
            << "Too many primitive sets logged for round-trip calculations, halting log process";
    }
    else
    {
        primitive_rtt.emplace_back(RoundTripTime{
            .primitive_sequence_num = seq_num,
            .thunderscope_sent_time_seconds =
                primitive.time_sent().epoch_timestamp_seconds(),
            .thunderloop_received_time_seconds = getCurrentEpochTimeInSeconds(),
        });
    }

    latest_primitive = primitive;
}

float PrimitiveTracker::getPrimitiveLossRate() const
{
    return proto_loss_rate;
}

std::optional<TbotsProto::Primitive> PrimitiveTracker::getLatestPrimitive()
{
    std::optional<TbotsProto::Primitive> primitive = std::move(latest_primitive);
    latest_primitive.reset();
    return primitive;
}

void PrimitiveTracker::updatePrimitiveLog(TbotsProto::RobotStatus& robot_status)
{
    const uint64_t seq_num = robot_status.last_handled_primitive_seq_num();
    while (!primitive_rtt.empty())
    {
        if (primitive_rtt.front().primitive_sequence_num == seq_num)
        {
            const double received_epoch_time_seconds =
                primitive_rtt.front().thunderloop_received_time_seconds;
            const double processing_time_seconds =
                getCurrentEpochTimeInSeconds() - received_epoch_time_seconds;

            robot_status.mutable_adjusted_time_sent()->set_epoch_timestamp_seconds(
                primitive_rtt.front().thunderscope_sent_time_seconds +
                processing_time_seconds);
            return;
        }
        primitive_rtt.pop_front();
    }
}

float PrimitiveTracker::calculateLossRate(const uint64_t seq_num) const
{
    // seq_num + 1 is to account for the sequence numbers starting from 0
    const uint64_t expected_count =
        std::min(seq_num + 1, static_cast<uint64_t>(RECENT_PRIMITIVE_LOSS_PERIOD));
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
