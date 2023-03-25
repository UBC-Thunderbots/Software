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

            TbotsProto::Timestamp timestamp;
            const auto current_time_ms =
                std::chrono::time_point_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now());
            timestamp.set_epoch_timestamp_seconds(
                static_cast<double>(current_time_ms.time_since_epoch().count()) /
                MILLISECONDS_PER_SECOND);
            *(log_msg_proto->mutable_created_timestamp()) = timestamp;

            log_output->sendProto(*log_msg_proto);
        }
    }
}
