#include "software/jetson_nano/services/network/packet_tracker.h"

PacketTracker::PacketTracker(const std::string& type) : proto_type(type) {}

void PacketTracker::send(uint64_t seq_num)
{
    // If the proto seems very out of date, then this is likely due to an AI reset. Clear
    // the queue
    if (!recent_proto_seq_nums.empty() &&
        seq_num + RECENT_PACKET_LOSS_PERIOD <= recent_proto_seq_nums.back())
    {
        recent_proto_seq_nums = std::queue<uint64_t>();
        LOG(WARNING) << "Old " << proto_type
                     << " received. Resetting sequence number tracking.";
    }
    else if (!recent_proto_seq_nums.empty() && seq_num <= recent_proto_seq_nums.back())
    {
        // If the proto is older than the last received proto, then ignore it
        last_valid = false;
        return;
    }

    recent_proto_seq_nums.push(seq_num);
    // Pop sequence numbers of protos that are no longer recent
    while (seq_num - recent_proto_seq_nums.front() >= RECENT_PACKET_LOSS_PERIOD)
    {
        recent_proto_seq_nums.pop();
    }

    float packet_loss_rate = calculate_packet_loss_rate(seq_num);

    if (packet_loss_rate > PACKET_LOSS_WARNING_THRESHOLD)
    {
        LOG(WARNING) << proto_type << " loss in the past "
                     << std::min(seq_num,
                                 static_cast<uint64_t>(RECENT_PACKET_LOSS_PERIOD))
                     << " packets is " << packet_loss_rate << "%";
    }

    last_valid = true;
}

bool PacketTracker::isLastValid()
{
    return last_valid;
}

float PacketTracker::calculate_packet_loss_rate(uint64_t seq_num)
{
    // seq_num + 1 is to account for the sequence numbers starting from 0
    uint64_t expected_proto_count =
        std::min(seq_num + 1, static_cast<uint64_t>(RECENT_PACKET_LOSS_PERIOD));
    uint64_t lost_proto_count = expected_proto_count - recent_proto_seq_nums.size();
    float packet_loss_rate =
        static_cast<float>(lost_proto_count) / static_cast<float>(expected_proto_count);

    return packet_loss_rate;
}
