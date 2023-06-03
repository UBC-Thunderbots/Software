#include "software/logger/network_sink.h"

#include "base64.h"
#include "google/protobuf/any.pb.h"
#include "proto/message_translation/tbots_protobuf.h"
#include "proto/robot_log_msg.pb.h"
#include "shared/constants.h"
#include "software/logger/custom_logging_levels.h"

NetworkSink::NetworkSink(unsigned int channel, const std::string& interface, int robot_id)
    : robot_id(robot_id)
{
    log_output.reset(new ThreadedProtoUdpSender<TbotsProto::RobotLog>(
        std::string(ROBOT_MULTICAST_CHANNELS.at(channel)) + "%" + interface,
        ROBOT_LOGS_PORT, true));

    log_visualize_output.reset(new ThreadedProtoUdpSender<TbotsProto::HRVOVisualization>(
        std::string(ROBOT_MULTICAST_CHANNELS.at(channel)) + "%" + interface,
        HRVO_VISUALIZATION_PORT, true));
}

void NetworkSink::sendToNetwork(g3::LogMessageMover log_entry)
{
    auto level = log_entry.get()._level;

    if (level.value == VISUALIZE.value)
    {
        TbotsProto::HRVOVisualization log_msg_proto;
        std::string msg       = log_entry.get().message();
        size_t file_name_pos  = msg.find(PROTO_MSG_TYPE_DELIMITER);
        std::string file_name = msg.substr(0, file_name_pos);

        unsigned int delimiter_length = strlen(PROTO_MSG_TYPE_DELIMITER);
        size_t proto_type_name_pos =
            msg.find(PROTO_MSG_TYPE_DELIMITER, file_name_pos + 1);
        std::string proto_type_name  = msg.substr(file_name_pos + delimiter_length,
                                                 proto_type_name_pos - delimiter_length);
        std::string serialized_proto = msg.substr(proto_type_name_pos + delimiter_length);

        // TODO (#2838): Rewrite the following code to be generalized and work for all
        // LOG(VISUALIZE) protobuf types
        if (proto_type_name == "TbotsProto.HRVOVisualization")
        {
            // We actually only just send the Serialized Proto, and exclude the proto type
            // and delimiters
            google::protobuf::Any any;
            any.ParseFromString(base64_decode(serialized_proto));
            any.UnpackTo(&log_msg_proto);
            std::string data_buffer;
            log_msg_proto.SerializeToString(&data_buffer);

            log_visualize_output->sendProto(log_msg_proto);
        }
    }

    TbotsProto::LogLevel log_level_proto;
    if (TbotsProto::LogLevel_Parse(log_entry.get().level(), &log_level_proto))
    {
        TbotsProto::RobotLog log_msg_proto;
        log_msg_proto.set_log_msg(log_entry.get().message());
        log_msg_proto.set_robot_id(robot_id);
        log_msg_proto.set_log_level(log_level_proto);
        log_msg_proto.set_file_name(log_entry.get().file());
        log_msg_proto.set_line_number(
            static_cast<uint32_t>(std::stoul(log_entry.get().line())));

        TbotsProto::Timestamp timestamp;
        const auto current_time_ms =
            std::chrono::time_point_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now());
        timestamp.set_epoch_timestamp_seconds(
            static_cast<double>(current_time_ms.time_since_epoch().count()) /
            MILLISECONDS_PER_SECOND);
        *(log_msg_proto.mutable_created_timestamp()) = timestamp;

        log_output->sendProto(log_msg_proto);
    }
}
