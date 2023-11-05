#include "software/jetson_nano/services/network/network.h"

NetworkService::NetworkService(const std::string& ip_address,
                               unsigned short world_listener_port,
                               unsigned short primitive_listener_port,
                               unsigned short robot_status_sender_port, bool multicast,
                               const unsigned thunderloop_hz)
    : ROBOT_STATUS_TO_THUNDERLOOP_HZ_RATIO(ROBOT_STATUS_BROADCAST_RATE_HZ /
                                           (thunderloop_hz + 1.0)),
      primitive_tracker(ProtoTracker("primitive set")),
      world_tracker(ProtoTracker("world")),
      control_loop_hz(thunderloop_hz)
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
    TbotsProto::RobotStatus& robot_status)
{
    std::scoped_lock lock{primitive_set_mutex, world_mutex};
    robot_status.mutable_network_status()->set_primitive_packet_loss_percentage(
        static_cast<unsigned int>(primitive_tracker.getLossRate() * 100));
    robot_status.mutable_network_status()->set_world_packet_loss_percentage(
        static_cast<unsigned int>(world_tracker.getLossRate() * 100));

    // Rate limit sending of proto based on thunderloop freq
    if ((robot_status.motor_status().front_left().motor_faults_size() > 0 ||
         robot_status.motor_status().front_right().motor_faults_size() > 0 ||
         robot_status.motor_status().back_left().motor_faults_size() > 0 ||
         robot_status.motor_status().back_right().motor_faults_size() > 0) ||
        (robot_status.has_power_status() &&
         robot_status.power_status().breakbeam_tripped() != last_breakbeam_state_sent) ||
        network_ticks / (thunderloop_ticks + 1.0) <= ROBOT_STATUS_TO_THUNDERLOOP_HZ_RATIO)
    {
        last_breakbeam_state_sent = robot_status.power_status().breakbeam_tripped();
        sender->sendProto(robot_status);
        network_ticks = (network_ticks + 1) % ROBOT_STATUS_BROADCAST_RATE_HZ;
    }
    thunderloop_ticks = (thunderloop_ticks + 1) % control_loop_hz;
    return std::tuple<TbotsProto::PrimitiveSet, TbotsProto::World>{primitive_set_msg,
                                                                   world_msg};
}

void NetworkService::primitiveSetCallback(TbotsProto::PrimitiveSet input)
{
    std::scoped_lock<std::mutex> lock(primitive_set_mutex);
    const uint64_t seq_num = input.sequence_number();

    primitive_tracker.send(seq_num);
    if (primitive_tracker.isLastValid())
    {
        primitive_set_msg = input;
    }
}

void NetworkService::worldCallback(TbotsProto::World input)
{
    std::scoped_lock<std::mutex> lock(world_mutex);
    const uint64_t seq_num = input.sequence_number();

    world_tracker.send(seq_num);
    if (world_tracker.isLastValid())
    {
        world_msg = input;
    }
}
