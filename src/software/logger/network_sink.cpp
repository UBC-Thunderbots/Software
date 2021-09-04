#include "software/logger/network_sink.h"

#include <experimental/filesystem>

#include "shared/proto/robot_log_msg.pb.h"
#include "software/constants.h"

NetworkSinc::NetworkSinc(int channel, const std::string& interface, int robot_id)
{
    robot_id_ = robot_id;
    log_output.reset(new ThreadedProtoUdpSender<TbotsProto::RobotLog>(
        std::string(NETWORK_LOGGING_MULTICAST_CHANNELS[channel]) + "%" + "interface",
        NETWORK_LOGS_PORT, true));
}

void NetworkSinc::sendToNetwork(g3::LogMessageMover log_entry)
{
    auto log_msg_proto = std::make_unique<TbotsProto::RobotLog>();
    TbotsProto::LogLevel log_level_proto;

    if (TbotsProto::LogLevel_Parse(log_entry.get().level(), &log_level_proto))
    {
        log_msg_proto->set_log_msg(log_entry.get().message());
        log_msg_proto->set_robot_id(robot_id_);
        log_msg_proto->set_log_level(log_level_proto);
        log_msg_proto->set_file_name(log_entry.get().file());
        log_msg_proto->set_line_number(
            static_cast<uint32_t>(std::stoul(log_entry.get().line())));

        log_output->sendProto(*log_msg_proto);
    }
}
