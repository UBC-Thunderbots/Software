#include "software/logger/network_sink.h"
#include "shared/proto/robot_log_msg.pb.h"
#include "software/constants.h"

#include <experimental/filesystem>

NetworkSinc::
NetworkSinc(int channel, const std::string& interface, int robot_id) {

    robot_id_ = robot_id;
    log_output.reset(new ThreadedProtoUdpSender<TbotsProto::RobotLog>(
            std::string(NETWORK_LOGGING_MULTICAST_CHANNELS[channel]) + "%" + interface, NETWORK_LOGS_PORT,
            true));
    std::cout<<std::string(NETWORK_LOGGING_MULTICAST_CHANNELS[channel]) + "%" + interface<<std::endl;
}

void NetworkSinc::sendToNetwork(g3::LogMessageMover log_entry)
{
    auto log = log_entry.get().toString();
    auto log_msg_to_send = std::make_unique<TbotsProto::RobotLog>();
//auto log_level = std::make_unique<TbotsProto::LogLevel>();

    log_msg_to_send->set_log_msg(log);
    log_msg_to_send->set_robot_id(robot_id_);
//log_msg->set_log_level(log_entry.get()._level.value);
//    log_msg->set_log_level("DEBUG");


    log_output->sendProto(*log_msg_to_send);
    std::cout<<log_msg_to_send->log_msg()<<std::endl;
}
