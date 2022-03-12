#pragma once
#include <fstream>
#include <g3log/logmessage.hpp>
#include <iostream>

#include "google/protobuf/any.pb.h"
#include "software/logger/custom_logging_levels.h"
#include "software/networking/threaded_unix_sender.h"

static const std::string TYPE_DELIMITER = "!!!";
static const std::string UNIX_BASE_PATH = "/tmp/tbots/";

class ProtobufSink
{
   public:
    ProtobufSink();

    /*
     * Send the protobuf to /tmp/tbots/(protobuf typename)
     *
     * @param log_entry The entry to log
     */
    void sendProtobuf(g3::LogMessageMover log_entry);

   private:
    std::unique_ptr<ThreadedUnixSender> protobuf_sender;
};

/*
 * Serialize a protobuf and pack it into a msg
 *
 * @param os The output stream
 * @param message The message to serialize
 * @return The output stream containing the base64 encoded serialized Any protobuf
 */
std::ostream& operator<<(std::ostream& os, const google::protobuf::Message& message);
