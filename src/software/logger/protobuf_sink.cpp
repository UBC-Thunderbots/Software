#include "software/logger/protobuf_sink.h"

#include <chrono>

#include "base64.h"
#include "google/protobuf/text_format.h"
#include "proto/robot_log_msg.pb.h"

ProtobufSink::ProtobufSink()
{
    // Setup the logs
    unix_senders["log"] = std::make_unique<ThreadedUnixSender>(UNIX_BASE_PATH + "log");
}

void ProtobufSink::sendProtobuf(g3::LogMessageMover log_entry)
{
    auto level = log_entry.get()._level;

    if (level.value == VISUALIZE.value)
    {
        std::string msg = log_entry.get().message();
        size_t pos      = msg.find(TYPE_DELIMITER);

        std::string proto_type_name  = msg.substr(0, pos);
        std::string serialized_proto = msg.substr(pos + TYPE_DELIMITER.length());

        // If we don't already have a unix sender for this type, lets create it
        if (unix_senders.count(proto_type_name) == 0)
        {
            unix_senders[proto_type_name] =
                std::make_unique<ThreadedUnixSender>(UNIX_BASE_PATH + proto_type_name);
        }

        // Send the protobuf
        unix_senders[proto_type_name]->sendString(serialized_proto);
    }
    else
    {
        TbotsProto::RobotLog log_msg_proto;
        TbotsProto::LogLevel log_level_proto;

        if (TbotsProto::LogLevel_Parse(log_entry.get().level(), &log_level_proto))
        {
            std::time_t t = std::time(0);
            log_msg_proto.mutable_created_timestamp()->set_epoch_timestamp_seconds(
                static_cast<double>(t));
            log_msg_proto.set_log_msg(log_entry.get().message());
            log_msg_proto.set_log_level(log_level_proto);
            log_msg_proto.set_file_name(log_entry.get().file());
            log_msg_proto.set_line_number(
                static_cast<uint32_t>(std::stoul(log_entry.get().line())));

            std::string log_msg;
            log_msg_proto.SerializeToString(&log_msg);
            unix_senders["log"]->sendString(base64_encode(log_msg));
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
