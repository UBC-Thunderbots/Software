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
                [&](const TbotsProto::Primitive& primitive)
                { primitiveCallback(primitive); });
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

    // Send an empty packet on the specific network interface to
    // ensure wifi is connected. Keeps trying until successful
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
    TbotsProto::RobotStatus& robot_status)
{
    ZoneNamedN(_tracy_network_poll, "Thunderloop: Poll NetworkService", true);

    const auto poll_start = std::chrono::steady_clock::now();

    std::scoped_lock lock{primitive_mutex};

    robot_status.mutable_network_status()->set_primitive_packet_loss_percentage(
        static_cast<unsigned int>(primitive_tracker.getPacketLoss() * 100));

    // Rate limit sending of proto based on thunderloop freq
    if (shouldSendNewRobotStatus(robot_status))
    {
        last_breakbeam_state_sent = robot_status.power_status().breakbeam_tripped();
        primitive_tracker.updatePrimitiveLog(robot_status);
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

    const auto now = std::chrono::steady_clock::now();
    const auto time_since_last_primitive =
        now - primitive_tracker.getLastPrimitiveReceivedTime();

    std::optional<TbotsProto::Primitive> primitive_to_return;
    if (new_primitive_msg_received)
    {
        new_primitive_msg_received = false;
        primitive_to_return        = primitive_msg;
    }

    // If we haven't received a primitive recently, stop the robot
    if (time_since_last_primitive >=
        std::chrono::nanoseconds(static_cast<long long>(PACKET_TIMEOUT_NS)))
    {
        TbotsProto::Primitive stop_primitive;
        stop_primitive.mutable_stop();
        primitive_to_return = stop_primitive;
    }

    using Millis = std::chrono::duration<double, std::milli>;
    robot_status.mutable_network_status()->set_ms_since_last_primitive_received(
        std::chrono::duration_cast<Millis>(time_since_last_primitive).count());

    const auto poll_end    = std::chrono::steady_clock::now();
    const Millis poll_time = std::chrono::duration_cast<Millis>(poll_end - poll_start);

    robot_status.mutable_thunderloop_status()->set_network_service_poll_time_ms(
        poll_time.count());

    return primitive_to_return;
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
        (network_ticks / (thunderloop_ticks + 1.0)) <=
        ROBOT_STATUS_TO_THUNDERLOOP_HZ_RATIO;

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
    std::scoped_lock lock(primitive_mutex);

    if (primitive_tracker.track(input))
    {
        primitive_msg              = input;
        new_primitive_msg_received = true;
    }
}
