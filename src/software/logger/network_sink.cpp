#include "software/logger/network_sink.h"

#include "proto/robot_log_msg.pb.h"
#include "shared/constants.h"

NetworkSink::NetworkSink(unsigned int channel, const std::string& interface, int robot_id)
    : robot_id(robot_id), merger(LogMerger())
{
    log_output.reset(new ThreadedProtoUdpSender<TbotsProto::RobotLog>(
        std::string(ROBOT_MULTICAST_CHANNELS.at(channel)) + "%" + interface,
        ROBOT_LOGS_PORT, true));
}

void NetworkSink::sendToNetwork(g3::LogMessageMover log_entry)
{
    auto log_msg_proto = std::make_unique<TbotsProto::RobotLog>();
    TbotsProto::LogLevel log_level_proto;

    if (TbotsProto::LogLevel_Parse(log_entry.get().level(), &log_level_proto))
    {
        std::list<std::string> logs = merger.log(log_entry.get().message());
        for (std::string msg : logs)
        {
            log_msg_proto->set_log_msg(msg);
            log_msg_proto->set_robot_id(robot_id);
            log_msg_proto->set_log_level(log_level_proto);
            log_msg_proto->set_file_name(log_entry.get().file());
            log_msg_proto->set_line_number(
                static_cast<uint32_t>(std::stoul(log_entry.get().line())));

            log_output->sendProto(*log_msg_proto);
        }
    }
}
