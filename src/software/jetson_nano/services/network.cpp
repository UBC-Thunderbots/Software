#include "software/jetson_nano/services/network.h"

#include "software/networking/threaded_proto_udp_listener.hpp"
#include "software/networking/threaded_proto_udp_sender.hpp"

NetworkService::NetworkService(const std::string& ip_address,
                               unsigned short world_listener_port,
                               unsigned short primitive_listener_port,
                               unsigned short robot_status_sender_port, bool multicast)
{
    sender = std::make_unique<ThreadedProtoUdpSender<TbotsProto::RobotStatus>>(
        ip_address, robot_status_sender_port, multicast);
    listener_primitive_set =
        std::make_unique<ThreadedProtoUdpListener<TbotsProto::PrimitiveSet>>(
            ip_address, primitive_listener_port,
            boost::bind(&NetworkService::primitiveSetCallback, this, _1), multicast);
    listener_world = std::make_unique<ThreadedProtoUdpListener<TbotsProto::World>>(
        ip_address, world_listener_port,
        boost::bind(&NetworkService::worldCallback, this, _1), multicast);
}

std::tuple<TbotsProto::PrimitiveSet, TbotsProto::World> NetworkService::poll(
    const TbotsProto::RobotStatus& robot_status)
{
    std::scoped_lock lock{primitive_set_mutex, world_mutex};
    TbotsProto::RobotStatus new_status = robot_status;
    new_status.set_last_handled_primitive_set(primitive_set_msg.sequence_number());
    sender->sendProto(robot_status);
    return std::tuple<TbotsProto::PrimitiveSet, TbotsProto::World>{primitive_set_msg,
                                                                   world_msg};
}

void NetworkService::primitiveSetCallback(TbotsProto::PrimitiveSet input)
{
    std::scoped_lock<std::mutex> lock(primitive_set_mutex);
    const uint64_t& seq_num = input.sequence_number();

    // If the primitive set seems very out of date, then this is likely due to an AI
    // reset. Clear the queue
    if (!recent_primitive_set_seq_nums.empty() &&
        seq_num + RECENT_PACKET_LOSS_PERIOD <= recent_primitive_set_seq_nums.back())
    {
        recent_primitive_set_seq_nums = std::queue<uint64_t>();
        LOG(WARNING)
            << "Old primitive set received. Resetting primitive set sequence number tracking";
    }
    else if (!recent_primitive_set_seq_nums.empty() &&
             seq_num <= recent_primitive_set_seq_nums.back())
    {
        // If the primitive set is older than the last received primitive set, then ignore
        // it
        return;
    }
    primitive_set_msg = input;
    recent_primitive_set_seq_nums.push(seq_num);

    // Pop sequence numbers of primitive sets that are no longer recent
    while (seq_num - recent_primitive_set_seq_nums.front() >= RECENT_PACKET_LOSS_PERIOD)
    {
        recent_primitive_set_seq_nums.pop();
    }

    // seq_num + 1 is to account for the sequence numbers starting from 0
    uint64_t expected_primitive_set_count =
        std::min(seq_num + 1, static_cast<uint64_t>(RECENT_PACKET_LOSS_PERIOD));
    uint64_t lost_primitive_set_count =
        expected_primitive_set_count - recent_primitive_set_seq_nums.size();
    float packet_loss_rate = static_cast<float>(lost_primitive_set_count) /
                             static_cast<float>(expected_primitive_set_count);

    if (packet_loss_rate > PACKET_LOSS_WARNING_THRESHOLD)
    {
        LOG(WARNING) << "Primitive set packet loss in the past "
                     << expected_primitive_set_count << " packets is more than "
                     << PACKET_LOSS_WARNING_THRESHOLD * 100 << "% ";
    }
}

void NetworkService::worldCallback(TbotsProto::World input)
{
    // TODO(#2728): Implement a recent world loss count and warning
    std::scoped_lock<std::mutex> lock(world_mutex);
    world_msg = input;
}
