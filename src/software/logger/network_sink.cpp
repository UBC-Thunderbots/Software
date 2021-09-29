#include "software/logger/network_sink.h"

#include <experimental/filesystem>

#include "shared/proto/robot_log_msg.pb.h"
#include "shared/constants.h"
#include "software/proto/message_translation/robot_log.h"

NetworkSink::NetworkSink(int channel, const std::string& interface, int robot_id) :robot_id(robot_id)
{
    log_output.reset(new ThreadedProtoUdpSender<TbotsProto::RobotLog>(
        std::string(ROBOT_MULTICAST_CHANNELS[channel]) + "%" + interface,
        ROBOT_LOGS_PORT, true));
}

void NetworkSink::sendToNetwork(g3::LogMessageMover log_entry)
{
    TbotsProto::RobotLog log_msg_proto;
    log_msg_proto = *(createRobotLog(log_entry, robot_id));
    log_output->sendProto(log_msg_proto);
}
