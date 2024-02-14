#include "software/jetson_nano/services/network/network.h"

NetworkService::NetworkService(const std::string& ip_address,
                               unsigned short primitive_listener_port,
                               unsigned short robot_status_sender_port, bool multicast)
    : primitive_tracker(ProtoTracker("primitive set"))
{
    sender = std::make_unique<ThreadedProtoUdpSender<TbotsProto::RobotStatus>>(
        ip_address, robot_status_sender_port, multicast);
    listener_primitive_set =
        std::make_unique<ThreadedProtoUdpListener<TbotsProto::PrimitiveSet>>(
            ip_address, primitive_listener_port,
            boost::bind(&NetworkService::primitiveSetCallback, this, std::placeholders::_1), multicast);
}

TbotsProto::PrimitiveSet NetworkService::poll(TbotsProto::RobotStatus& robot_status)
{
    std::scoped_lock lock{primitive_set_mutex};

    robot_status.mutable_network_status()->set_primitive_packet_loss_percentage(
        static_cast<unsigned int>(primitive_tracker.getLossRate() * 100));

    // Rate limit sending of proto based on thunderloop freq
    if (shouldSendNewRobotStatus(robot_status))
    {
        last_breakbeam_state_sent = robot_status.power_status().breakbeam_tripped();
        sender->sendProto(robot_status);
        network_ticks = (network_ticks + 1) % ROBOT_STATUS_BROADCAST_RATE_HZ;
    }
    thunderloop_ticks = (thunderloop_ticks + 1) % CONTROL_LOOP_HZ;
    return primitive_set_msg;
}

bool NetworkService::shouldSendNewRobotStatus(
    const TbotsProto::RobotStatus& robot_status) const
{
    bool has_motor_fault =
        robot_status.motor_status().front_left().motor_faults_size() > 0 ||
        robot_status.motor_status().front_right().motor_faults_size() > 0 ||
        robot_status.motor_status().back_left().motor_faults_size() > 0 ||
        robot_status.motor_status().back_right().motor_faults_size() > 0;

    bool has_breakbeam_status_changed =
        robot_status.has_power_status() &&
        robot_status.power_status().breakbeam_tripped() != last_breakbeam_state_sent;

    bool require_heartbeat_status_update = (network_ticks / (thunderloop_ticks + 1.0)) <=
                                           ROBOT_STATUS_TO_THUNDERLOOP_HZ_RATIO;

    return has_motor_fault || has_breakbeam_status_changed ||
           require_heartbeat_status_update;
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

    float primitive_set_loss_rate = primitive_tracker.getLossRate();
    if (primitive_set_loss_rate > PROTO_LOSS_WARNING_THRESHOLD)
    {
        LOG(WARNING) << "Primitive set loss rate is " << primitive_set_loss_rate * 100
                     << "%";
    }
}
