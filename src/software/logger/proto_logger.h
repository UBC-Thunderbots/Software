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
    /**
     * A struct to store a serialized protobuf message for the purpose of replay logging
     */
    struct SerializedProtoLog
    {
        std::string protobuf_type_full_name;
        std::string serialized_proto;
        double receive_time_sec;
    };

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

    ProtoLogger() = delete;

    /**
     * Removing copy constructor and assignment operator to prevent accidental copying
     * of the ProtoLogger.
     */
    ProtoLogger(const ProtoLogger&)            = delete;
    ProtoLogger& operator=(const ProtoLogger&) = delete;

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

    /**
     * Update the time provider used to create timestamp for the protobufs
     * @param time_provider A function that returns the current time in seconds
     */
    void updateTimeProvider(std::function<double()> time_provider);

    /**
     * Flushes the buffer and stops the logging thread.
     *
     * @note ProtoLogger will try to flush the buffer for up to
     * MAX_TIME_TO_EXIT_FULL_SYSTEM_SEC seconds before stopping the logging thread. As
     * such, if there are many logs in the buffer, not all may be flushed in time and some
     * data may be lost.
     */
    void flushAndStopLogging();

    /**
     * Helper function for creating a log entry
     * @param protobuf_type_full_name The full name of the protobuf message type
     * (e.g. TbotsProto.ThunderbotsConfig)
     * @param serialized_proto The serialized protobuf message to store
     * @param receive_time_sec The time the protobuf was received
     * @return A string containing the log entry
     */
    static std::string createLogEntry(const std::string& proto_full_name,
                                      const std::string& serialized_proto,
                                      double receive_time_sec);

   private:
    /**
     * The loop which will be continuously logging the protobufs
     */
    void logProtobufs();

    /**
     * Helper function to determine if the logging thread should stop
     * @return True if the logging thread should stop, false otherwise
     */
    bool shouldStopLogging() const;

    std::string log_path_;
    std::string log_folder_;
    std::function<double()> time_provider_;
    double start_time_;
    bool friendly_colour_yellow_;
    unsigned int failed_logs_frequency_counter_ = 0;

    std::thread log_thread_;
    std::atomic<bool> stop_logging_;
    double destructor_called_time_sec_;

    ThreadSafeBuffer<SerializedProtoLog> buffer_;

    const Duration BUFFER_BLOCK_TIMEOUT                = Duration::fromSeconds(0.1);
    const std::string REPLAY_FILE_PREFIX               = "proto_";
    const std::string REPLAY_FILE_TIME_FORMAT          = "%Y_%m_%d_%H_%M_%S";
    static constexpr unsigned int PROTOBUF_BUFFER_SIZE = 1000;
    static constexpr unsigned int REPLAY_MAX_CHUNK_SIZE_BYTES = 1024 * 1024;  // 1 MB
    static constexpr unsigned int FAILED_LOG_PRINT_FREQUENCY  = 100;
};
