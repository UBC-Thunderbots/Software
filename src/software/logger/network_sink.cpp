#include "software/logger/network_sink.h"

#include "proto/robot_log_msg.pb.h"
#include "shared/constants.h"

NetworkSink::NetworkSink(unsigned int channel, const std::string& interface, int robot_id)
    : robot_id(robot_id)
{
    log_output.reset(new ThreadedProtoUdpSender<TbotsProto::RobotLog>(
        std::string(ROBOT_MULTICAST_CHANNELS.at(channel)) + "%" + interface,
        ROBOT_LOGS_PORT, true));
}

void NetworkSink::sendToNetwork(g3::LogMessageMover log_entry)
{
    // Spam reduction notes TODO delete
    // Different from last message: log it, save timestamp and message
    // Same as last message: check timestamp, if within interval of saved timestamp, repeats++
    //         if outside interval, log it with repeats added, reset saved repeats and timestamp
    
    // problem?: drops number of repeats on new message

    auto log_msg_proto = std::make_unique<TbotsProto::RobotLog>();
    TbotsProto::LogLevel log_level_proto;

    if (TbotsProto::LogLevel_Parse(log_entry.get().level(), &log_level_proto))
    {   
        // reduce spam by sending a period if message is same as previous
        std::string msg_to_send;
        if (log_entry.get().message() == last_msg) {
            msg_to_send = ".";
        } else {
            msg_to_send = log_entry.get().message();
        }

        log_msg_proto->set_log_msg(msg_to_send);
        log_msg_proto->set_robot_id(robot_id);
        log_msg_proto->set_log_level(log_level_proto);
        log_msg_proto->set_file_name(log_entry.get().file());
        log_msg_proto->set_line_number(
            static_cast<uint32_t>(std::stoul(log_entry.get().line())));

        log_output->sendProto(*log_msg_proto);

        last_msg = log_entry.get().message();
    }
}
