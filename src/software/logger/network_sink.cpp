#include "software/logger/network_sink.h"

#include "proto/robot_log_msg.pb.h"
#include "shared/constants.h"

NetworkSink::NetworkSink(unsigned int channel, const std::string& interface, int robot_id)
    : robot_id(robot_id)
{
    log_output.reset(new ThreadedProtoUdpSender<TbotsProto::RobotLog>(
        std::string(ROBOT_MULTICAST_CHANNELS.at(channel)) + "%" + interface,
        ROBOT_LOGS_PORT, true));
    num_repeats = 0;
}

void NetworkSink::sendToNetwork(g3::LogMessageMover log_entry)
{
    auto log_msg_proto = std::make_unique<TbotsProto::RobotLog>();
    TbotsProto::LogLevel log_level_proto;

    if (TbotsProto::LogLevel_Parse(log_entry.get().level(), &log_level_proto))
    {
        std::chrono::_V2::system_clock::time_point current_time =
            std::chrono::system_clock::now();
        bool past_time = current_time - LOG_INTERVAL_TIMESTAMP < last_msg_timestamp;
        if (log_entry.get().message() == last_msg && past_time)
        {
            // repeated message outside timestamp, increase repeats and don't log
            num_repeats++;
            return;
        }

        // log and save info
        last_msg           = log_entry.get().message();
        last_msg_timestamp = current_time;

        // remove newline from end of message
        if (log_entry.get()._message.back() == '\n')
        {
            log_entry.get()._message.pop_back();
        }

        if (num_repeats > 1)
        {
            log_entry.get()._message += " (" + std::to_string(num_repeats) + " repeats)";
        }

        log_msg_proto->set_log_msg(log_entry.get().message());
        log_msg_proto->set_robot_id(robot_id);
        log_msg_proto->set_log_level(log_level_proto);
        log_msg_proto->set_file_name(log_entry.get().file());
        log_msg_proto->set_line_number(
            static_cast<uint32_t>(std::stoul(log_entry.get().line())));

        log_output->sendProto(*log_msg_proto);

        num_repeats = 1;
    }
}
