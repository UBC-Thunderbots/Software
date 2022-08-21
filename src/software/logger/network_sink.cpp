#include "software/logger/network_sink.h"

#include "proto/robot_log_msg.pb.h"
#include "shared/constants.h"

NetworkSink::NetworkSink(unsigned int channel, const std::string& interface, int robot_id)
    : robot_id(robot_id)
{
    log_output.reset(new ThreadedProtoUdpSender<TbotsProto::RobotLog>(
        std::string(ROBOT_MULTICAST_CHANNELS.at(channel)) + "%" + interface,
        ROBOT_LOGS_PORT, true));
    serialized_proto_log_output.reset(new ThreadedProtoUdpSender<TbotsProto::HRVOVisualization>(
                            std::string(ROBOT_MULTICAST_CHANNELS.at(channel)) + "%" + interface,
                            SERIALIZED_PROTO_LOGS_PORT, true));
}

void NetworkSink::sendToNetwork(g3::LogMessageMover log_entry)
{
    auto level = log_entry.get()._level;
//    LOG(INFO) << "receiving level " << level.value;

   if (level.value == VISUALIZE.value)
   {
       auto log_msg_proto = std::make_unique<TbotsProto::HRVOVisualization>();
       std::string msg       = log_entry.get().message();
       size_t file_name_pos  = msg.find(TYPE_DELIMITER);
       std::string file_name = msg.substr(0, file_name_pos);

       size_t proto_type_name_pos = msg.find(TYPE_DELIMITER, file_name_pos + 1);
       std::string proto_type_name =
           msg.substr(file_name_pos + TYPE_DELIMITER.length(),
                      proto_type_name_pos - TYPE_DELIMITER.length());
       std::string serialized_proto =
           msg.substr(proto_type_name_pos + TYPE_DELIMITER.length());
//       LOG(INFO) << "Received log visualize " << proto_type_name;
       if (proto_type_name == "TbotsProto.HRVOVisualization")
       {
//           LOG(INFO) << "Parsing a HRVOVisualization";
           log_msg_proto->ParseFromString(serialized_proto);

           serialized_proto_log_output->sendProto(*log_msg_proto);
//           LOG(INFO) << "Sent a HRVO Visualization";
       } 
   }
   else
   {
        auto log_msg_proto = std::make_unique<TbotsProto::RobotLog>();
        TbotsProto::LogLevel log_level_proto;

        if (TbotsProto::LogLevel_Parse(log_entry.get().level(), &log_level_proto))
        {
            log_msg_proto->set_log_msg(log_entry.get().message());
            log_msg_proto->set_robot_id(robot_id);
            log_msg_proto->set_log_level(log_level_proto);
            log_msg_proto->set_file_name(log_entry.get().file());
            log_msg_proto->set_line_number(
                static_cast<uint32_t>(std::stoul(log_entry.get().line())));

            log_output->sendProto(*log_msg_proto);
        }
   } 
}
