#include "software/logger/protobuf_sink.h"

#include <chrono>

#include "base64.h"
#include "google/protobuf/text_format.h"
#include "proto/robot_log_msg.pb.h"
#include "shared/constants.h"

ProtobufSink::ProtobufSink(std::string runtime_dir)
{
    // Setup the logs
    protobuf_sender = std::make_unique<ThreadedUnixSender>(runtime_dir + "/protobuf");
}

void ProtobufSink::sendProtobuf(g3::LogMessageMover log_entry)
{
    auto level = log_entry.get()._level;

    if (level.value == VISUALIZE.value)
    {
        // Send the protobuf
        protobuf_sender->sendString(log_entry.get().message());
    }
    else
    {
        TbotsProto::RobotLog log_msg_proto;
        TbotsProto::LogLevel log_level_proto;

        if (TbotsProto::LogLevel_Parse(log_entry.get().level(), &log_level_proto))
        {
            const auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now());
            log_msg_proto.mutable_created_timestamp()->set_epoch_timestamp_seconds(
                static_cast<double>(now_ms.time_since_epoch().count()) /
                MILLISECONDS_PER_SECOND);

            log_msg_proto.set_log_msg(log_entry.get().message());
            log_msg_proto.set_log_level(log_level_proto);
            log_msg_proto.set_file_name(log_entry.get().file());
            log_msg_proto.set_line_number(
                static_cast<uint32_t>(std::stoul(log_entry.get().line())));

            // Pack into Any
            google::protobuf::Any any;
            any.PackFrom(log_msg_proto);

            // Serialize into any
            std::string serialized_any;
            any.SerializeToString(&serialized_any);
            protobuf_sender->sendString(log_msg_proto.GetTypeName() + TYPE_DELIMITER +
                                        base64_encode(serialized_any));
        }
    }
}

std::ostream& operator<<(std::ostream& os, const google::protobuf::Message& message)
{
    // Pack into Any
    google::protobuf::Any any;
    any.PackFrom(message);

    // Serialize into any
    std::string serialized_any;
    any.SerializeToString(&serialized_any);

    os << message.GetTypeName() << TYPE_DELIMITER << base64_encode(serialized_any);
    return os;
}
