#pragma once

#include <google/protobuf/message.h>

#include <functional>
#include <string>
#include <thread>

#include "software/multithreading/thread_safe_buffer.hpp"

/**
 * Logs incoming Protobufs to a folder to be played back later.
 *
 * Each entry will contain:
 *  - The timestamp
 *  - The protobuf type
 *  - The protobuf serialized as base64 (to remove newline characters)
 *
 * Stored in log_path/proto_YYYY_MM_DD_HH_MM_SS/
 * With each entry in the file formatted as timestamp,protobuf_type,protobuf_base64
 * Note that in order to reduce the size of the log files, the files are compressed
 * using gzip.
 *
 * We need to store the data in a way that we can:
 *  1. Replay the data chronologically
 *  2. Seek to a specific time (random access)
 *
 * To seek to a specific time, we need to load the entire log file into memory.
 * To make this feasible, we store the data in chunks. Each chunk contains
 * REPLAY_MAX_CHUNK_SIZE_BYTES of serialized protos.
 * We can load each chunk into memory and perform search operations to find the
 * appropriate entry. Or we can just play the chunks in order.
 */
class ProtoLogger
{
   public:
    /**
     * Constructor
     * @param log_path The path to the directory where the logs will be saved
     * @param time_provider A function that returns the current time in seconds
     * @param friendly_colour_yellow Whether the friendly team is yellow or not
     */
    explicit ProtoLogger(const std::string& log_path,
                         std::function<double()> time_provider,
                         bool friendly_colour_yellow);

    ~ProtoLogger();

    /**
     * Saves a serialized protobuf message to the log
     * @param protobuf_type_full_name The full name of the protobuf message type
     * (e.g. TbotsProto.ThunderbotsConfig)
     * @param serialized_proto The serialized protobuf message to store
     */
    void saveSerializedProto(const std::string& protobuf_type_full_name,
                             const std::string& serialized_proto);

    /**
     * Saves a serialized protobuf message to the log
     * @tparam ProtoType The type of the protobuf message (Must extend the
     * google::protobuf::Message class and not google::protobuf::MessageLite so it has
     * reflection capabilities)
     * @param serialized_proto The serialized protobuf message to store
     */
    template <typename ProtoType>
    inline void saveSerializedProto(const std::string& serialized_proto)
    {
        static_assert(std::is_base_of<google::protobuf::Message, ProtoType>::value,
                      "ProtoType has to be a protobuf message type");

        saveSerializedProto(ProtoType::GetDescriptor()->full_name(), serialized_proto);
    };

   private:
    /**
     * The loop which will be continuously logging the protobufs
     */
    void logProtobufs();

    std::string log_path_;
    std::string log_folder_;
    std::function<double()> time_provider_;
    double start_time_;
    bool stop_logging_;
    std::thread log_thread_;

    // Buffer storing the full name of the protobuf type to the serialized
    // message which should be stored.
    ThreadSafeBuffer<std::pair<std::string, std::string>> buffer_;

    // TODO (NIMA): Move to constants and pybind to python
    const Duration BUFFER_BLOCK_TIMEOUT                = Duration::fromSeconds(0.1);
    const std::string REPLAY_FILE_PREFIX               = "proto_";
    const std::string REPLAY_FILE_EXTENSION            = "replay";
    const std::string REPLAY_FILE_TIME_FORMAT          = "%Y_%m_%d_%H_%M_%S";
    const std::string REPLAY_METADATA_DELIMETER        = ",";
    static constexpr unsigned int PROTOBUF_BUFFER_SIZE = 1000;
    static constexpr unsigned int REPLAY_MAX_CHUNK_SIZE_BYTES = 1024 * 1024;  // 1 MB
};
