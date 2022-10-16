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
    primitive_set_msg   = input;
    const auto& seq_num = input.sequence_number();

    // Ignore the primitive and consider it as lost if it is received out of order
    if (!recent_primitive_seq_nums.empty() && seq_num <= recent_primitive_seq_nums.back())
    {
        return;
    }

    recent_primitive_seq_nums.push(seq_num);

    // Pop sequence numbers of primitives that are no longer recent
    while (seq_num - recent_primitive_seq_nums.front() >= RECENT_PACKET_LOSS_PERIOD)
    {
        recent_primitive_seq_nums.pop();
    }

    auto expected_primitives_count =
        std::min(seq_num, static_cast<uint64_t>(RECENT_PACKET_LOSS_PERIOD));
    auto lost_primitives_count =
        expected_primitives_count - recent_primitive_seq_nums.size();
    auto packet_loss_rate = static_cast<float>(lost_primitives_count) /
                            static_cast<float>(expected_primitives_count);

    if (packet_loss_rate > PACKET_LOSS_WARNING_THRESHOLD)
    {
        LOG(WARNING) << "Primitive packet loss in the past " << expected_primitives_count
                     << " packets more than " << PACKET_LOSS_WARNING_THRESHOLD * 100
                     << "% ";
    }
}

void NetworkService::worldCallback(TbotsProto::World input)
{
    std::scoped_lock<std::mutex> lock(world_mutex);
    world_msg = input;
    // TODO(#2728): Implement a recent world loss count and warning

    last_world_time = input.time_sent().epoch_timestamp_seconds();
}
