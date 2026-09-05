#include "software/embedded/services/network/network.h"

#include <Tracy.hpp>

#include "software/logger/network_logger.h"
#include "software/networking/tbots_network_exception.h"

NetworkService::NetworkService(const NetworkConfig& config)
    : multicast_ip(config.multicast_ip),
      interface(config.interface),
      robot_status_sender_port(config.robot_status_sender_port)
{
    waitForNetworkUp();

    try
    {
        fullsystem_to_robot_ip_listener =
            std::make_unique<ThreadedProtoUdpListener<TbotsProto::IpNotification>>(
                config.multicast_ip, config.full_system_to_robot_ip_notification_port,
                config.interface,
                [&](const TbotsProto::IpNotification& ip_notification)
                { onFullSystemIpNotification(ip_notification); },
                true);

        robot_to_fullsystem_ip_sender =
            std::make_unique<ThreadedProtoUdpSender<TbotsProto::IpNotification>>(
                config.multicast_ip, config.robot_to_full_system_ip_notification_port,
                config.interface, false);

        udp_listener_primitive =
            std::make_unique<ThreadedProtoUdpListener<TbotsProto::Primitive>>(
                config.primitive_listener_port,
                [&](const TbotsProto::Primitive& prim) { primitiveCallback(prim); });
    }
    catch (const TbotsNetworkException& e)
    {
        LOG(FATAL) << e.what();
    }

    robot_ip_notification_msg.set_robot_id(static_cast<int>(config.robot_id));
    if (std::optional<std::string> local_ip = getLocalIp(config.interface))
    {
        robot_ip_notification_msg.set_ip_address(*local_ip);
    }
    else
    {
        LOG(FATAL) << "Failed to get IP addresses associated with " << config.interface;
    }
}

void NetworkService::waitForNetworkUp()
{
    std::unique_ptr<ThreadedUdpSender> network_tester;
    try
    {
        network_tester = std::make_unique<ThreadedUdpSender>(
            multicast_ip, NETWORK_COMM_TEST_PORT, interface, true);
    }
    catch (TbotsNetworkException& e)
    {
        LOG(FATAL) << "Thunderloop cannot connect to the network. Error: " << e.what();
    }

    // Send an empty packet on the specific network interface to ensure that
    // we're connected to the network. Keep trying until successful
    while (true)
    {
        try
        {
            network_tester->sendString("");
            break;
        }
        catch (std::exception& e)
        {
            // Resend the message after a delay
            LOG(WARNING) << "Thunderloop cannot connect to network!"
                         << "Waiting for connection...";
            sleep(PING_RETRY_DELAY_S);
        }
    }

    LOG(INFO) << "Thunderloop connected to network!";
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

std::optional<TbotsProto::Primitive> NetworkService::poll(
    TbotsProto::RobotStatus& robot_status, const double time_elapsed_since_last_poll_s)
{
    ZoneNamedN(_tracy_network_poll, "Thunderloop: Poll NetworkService", true);

    const auto poll_start = std::chrono::steady_clock::now();

    const std::optional<TbotsProto::Primitive> primitive_to_return =
        getPrimitiveToExecute(time_elapsed_since_last_poll_s);

    updateNetworkStatus(robot_status);
    sendRobotStatusIfNeeded(robot_status, time_elapsed_since_last_poll_s);
    sendIpNotificationIfNeeded(time_elapsed_since_last_poll_s);

    const auto poll_end    = std::chrono::steady_clock::now();
    using Millis           = std::chrono::duration<double, std::milli>;
    const Millis poll_time = std::chrono::duration_cast<Millis>(poll_end - poll_start);

    robot_status.mutable_thunderloop_status()->set_network_service_poll_time_ms(
        poll_time.count());

    return primitive_to_return;
}

void NetworkService::updateNetworkStatus(TbotsProto::RobotStatus& robot_status)
{
    float packet_loss;
    {
        std::scoped_lock lock(primitive_tracker_mutex);
        packet_loss = primitive_tracker.getPrimitiveLossRate();
    }

    robot_status.mutable_network_status()->set_primitive_packet_loss_percentage(
        static_cast<unsigned int>(packet_loss * 100));

    robot_status.mutable_network_status()->set_ms_since_last_primitive_received(
        time_since_last_primitive_received_s_ * MILLISECONDS_PER_SECOND);
}

void NetworkService::sendRobotStatusIfNeeded(TbotsProto::RobotStatus& robot_status,
                                             const double time_elapsed_since_last_poll_s)
{
    time_since_last_robot_status_sent_s_ += time_elapsed_since_last_poll_s;

    if (!shouldSendNewRobotStatus(robot_status))
    {
        return;
    }

    time_since_last_robot_status_sent_s_ = 0.0;
    last_breakbeam_state_sent = robot_status.power_status().breakbeam_tripped();
    {
        std::scoped_lock lock(primitive_tracker_mutex);
        primitive_tracker.updatePrimitiveLog(robot_status);
    }
    sendRobotStatus(robot_status);
}

void NetworkService::sendIpNotificationIfNeeded(
    const double time_elapsed_since_last_poll_s)
{
    time_since_last_ip_notification_sent_s_ += time_elapsed_since_last_poll_s;

    if (time_since_last_ip_notification_sent_s_ < IP_NOTIFICATION_SEND_INTERVAL_S)
    {
        return;
    }

    time_since_last_ip_notification_sent_s_ = 0.0;
    robot_to_fullsystem_ip_sender->sendProto(robot_ip_notification_msg, true);
}

std::optional<TbotsProto::Primitive> NetworkService::getPrimitiveToExecute(
    const double time_elapsed_since_last_poll_s)
{
    std::optional<TbotsProto::Primitive> latest_primitive;
    {
        std::scoped_lock lock(primitive_tracker_mutex);
        latest_primitive = primitive_tracker.getLatestPrimitive();
    }

    if (latest_primitive.has_value())
    {
        time_since_last_primitive_received_s_ = 0.0;
        return latest_primitive;
    }

    time_since_last_primitive_received_s_ += time_elapsed_since_last_poll_s;

    // If we haven't received a primitive recently, stop the robot
    if (time_since_last_primitive_received_s_ >= PRIMITIVE_RECEIVE_TIMEOUT_S)
    {
        TbotsProto::Primitive stop_primitive;
        stop_primitive.mutable_stop();
        return stop_primitive;
    }

    return std::nullopt;
}

bool NetworkService::shouldSendNewRobotStatus(
    const TbotsProto::RobotStatus& robot_status) const
{
    const bool has_motor_fault =
        robot_status.motor_status().front_left().motor_faults_size() > 0 ||
        robot_status.motor_status().front_right().motor_faults_size() > 0 ||
        robot_status.motor_status().back_left().motor_faults_size() > 0 ||
        robot_status.motor_status().back_right().motor_faults_size() > 0;

    const bool has_breakbeam_status_changed =
        robot_status.has_power_status() &&
        robot_status.power_status().breakbeam_tripped() != last_breakbeam_state_sent;

    const bool require_heartbeat_status_update =
        time_since_last_robot_status_sent_s_ >= ROBOT_STATUS_SEND_INTERVAL_S;

    return has_motor_fault || has_breakbeam_status_changed ||
           require_heartbeat_status_update;
}

void NetworkService::sendRobotStatus(TbotsProto::RobotStatus& robot_status)
{
    const auto epoch_time = std::chrono::steady_clock::now().time_since_epoch();
    robot_status.mutable_time_sent()->set_epoch_timestamp_seconds(
        std::chrono::duration_cast<std::chrono::duration<double>>(epoch_time).count());

    std::scoped_lock lock(robot_status_sender_mutex);

    if (robot_status_sender)
    {
        robot_status_sender->sendProto(robot_status, true);
    }
}

void NetworkService::primitiveCallback(const TbotsProto::Primitive& input)
{
    std::scoped_lock lock(primitive_tracker_mutex);
    primitive_tracker.track(input);
}
