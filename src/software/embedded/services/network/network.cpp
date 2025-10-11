#include "software/embedded/services/network/network.h"

#include "software/logger/network_logger.h"
#include "software/networking/tbots_network_exception.h"

NetworkService::NetworkService(const RobotId& robot_id, const std::string& ip_address,
                               unsigned short primitive_listener_port,
                               unsigned short robot_status_sender_port,
                               unsigned short full_system_to_robot_ip_notification_port,
                               unsigned short robot_to_full_system_ip_notification_port,
                               unsigned short robot_logs_port,
                               const std::string& interface)
    : interface(interface),
      robot_status_sender_port(robot_status_sender_port),
      primitive_tracker(ProtoTracker("primitive set"))
{
    try
    {
        fullsystem_to_robot_ip_listener =
            std::make_unique<ThreadedProtoUdpListener<TbotsProto::IpNotification>>(
                ip_address, full_system_to_robot_ip_notification_port, interface,
                std::bind(&NetworkService::onFullSystemIpNotification, this,
                          std::placeholders::_1),
                true);

        robot_to_fullsystem_ip_sender =
            std::make_unique<ThreadedProtoUdpSender<TbotsProto::IpNotification>>(
                ip_address, robot_to_full_system_ip_notification_port, interface, false);


        udp_listener_primitive =
            std::make_unique<ThreadedProtoUdpListener<TbotsProto::Primitive>>(
                primitive_listener_port,
                [&](TbotsProto::Primitive primitive) { primitiveCallback(primitive); });
    }
    catch (const TbotsNetworkException& e)
    {
        LOG(FATAL) << e.what();
    }

    robot_ip_notification_msg.set_robot_id(robot_id);
    std::optional<std::string> local_ip = getLocalIp(interface);
    if (local_ip)
    {
        robot_ip_notification_msg.set_ip_address(*local_ip);
    }
    else
    {
        LOG(FATAL) << "Failed to get IP addresses associated with " << interface;
    }

}

void NetworkService::onFullSystemIpNotification(
    const TbotsProto::IpNotification& ip_notification)
{
    std::string new_fullsystem_ip = ip_notification.ip_address();
    bool rebuild_sender           = false;

    {
        std::scoped_lock lock(fullsystem_ip_mutex);
        if (!fullsystem_ip.has_value() || fullsystem_ip.value() != new_fullsystem_ip)
        {
            fullsystem_ip  = new_fullsystem_ip;
            rebuild_sender = true;
        }
    }

    if (rebuild_sender)
    {
        std::unique_lock lock(robot_status_sender_mutex);

        try
        {
            robot_status_sender =
                std::make_unique<ThreadedProtoUdpSender<TbotsProto::RobotStatus>>(
                    fullsystem_ip.value(), robot_status_sender_port, interface, false);
            robot_log_sender =
                std::make_shared<ThreadedProtoUdpSender<TbotsProto::RobotLog>>(
                    fullsystem_ip.value(), ROBOT_LOGS_PORT, interface, false);
            NetworkLoggerSingleton::replaceUdpSender(robot_log_sender);
        }
        catch (const TbotsNetworkException& error)
        {
            LOG(FATAL) << error.what();
        }

        LOG(INFO) << "Now sending RobotStatus messages to " << fullsystem_ip.value();
    }
}

TbotsProto::Primitive NetworkService::poll(TbotsProto::RobotStatus& robot_status)
{
    std::scoped_lock lock{primitive_mutex};

    robot_status.mutable_network_status()->set_primitive_packet_loss_percentage(
        static_cast<unsigned int>(primitive_tracker.getLossRate() * 100));

    // Rate limit sending of proto based on thunderloop freq
    if (shouldSendNewRobotStatus(robot_status))
    {
        last_breakbeam_state_sent = robot_status.power_status().breakbeam_tripped();
        updatePrimitiveLog(robot_status);
        sendRobotStatus(robot_status);
        network_ticks = (network_ticks + 1) % ROBOT_STATUS_BROADCAST_RATE_HZ;
    }

    if (ip_notification_ticks == 0)
    {
        robot_to_fullsystem_ip_sender->sendProto(robot_ip_notification_msg, true);
    }
    ip_notification_ticks =
        (ip_notification_ticks + 1) % IP_DISCOVERY_NOTIFICATION_RATE_HZ;

    thunderloop_ticks = (thunderloop_ticks + 1) % THUNDERLOOP_HZ;
    return primitive_msg;
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

void NetworkService::sendRobotStatus(const TbotsProto::RobotStatus& robot_status)
{
    std::scoped_lock lock(robot_status_sender_mutex);

    if (robot_status_sender)
    {
        robot_status_sender->sendProto(robot_status, true);
    }
}

void NetworkService::primitiveCallback(const TbotsProto::Primitive& input)
{
    std::scoped_lock<std::mutex> lock(primitive_mutex);
    const uint64_t seq_num = input.sequence_number();

    logNewPrimitive(input);

    primitive_tracker.send(seq_num);
    if (primitive_tracker.isLastValid())
    {
        primitive_msg = input;
    }
}

void NetworkService::logNewPrimitive(const TbotsProto::Primitive& new_primitive)
{
    if (primitive_rtt.size() >= PRIMITIVE_DEQUE_MAX_SIZE)
    {
        LOG(WARNING)
            << "Too many primitive sets logged for round-trip calculations, halting log process";
        return;
    }

    if (!primitive_rtt.empty() &&
        new_primitive.sequence_number() <= primitive_rtt.back().primitive_sequence_num)
    {
        // If the proto is older than the last received proto, then ignore it
        return;
    }

    NetworkService::RoundTripTime current_round_trip_time;
    current_round_trip_time.primitive_sequence_num = new_primitive.sequence_number();
    current_round_trip_time.thunderscope_sent_time_seconds =
        new_primitive.time_sent().epoch_timestamp_seconds();
    current_round_trip_time.thunderloop_recieved_time_seconds =
        getCurrentEpochTimeInSeconds();

    primitive_rtt.emplace_back(current_round_trip_time);
}

void NetworkService::updatePrimitiveLog(TbotsProto::RobotStatus& robot_status)
{
    uint64_t seq_num = robot_status.last_handled_primitive_set();
    while (!primitive_rtt.empty())
    {
        if (primitive_rtt.front().primitive_sequence_num == seq_num)
        {
            double received_epoch_time_seconds =
                primitive_rtt.front().thunderloop_recieved_time_seconds;
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

double NetworkService::getCurrentEpochTimeInSeconds()
{
    const auto clock_time = std::chrono::system_clock::now();
    double time_in_seconds =
        static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                                clock_time.time_since_epoch())
                                .count()) *
        SECONDS_PER_MICROSECOND;
    return time_in_seconds;
}
