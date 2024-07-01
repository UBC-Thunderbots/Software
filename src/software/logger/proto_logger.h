#pragma once

#include <functional>
#include <string>
#include <thread>
#include <google/protobuf/message.h>
#include "software/multithreading/thread_safe_buffer.hpp"

//template <typename T=std::pair<std::string, std::string>>
//class ThreadSafeBuffer;

class ProtoLogger
{
   public:
    explicit ProtoLogger(const std::string& log_path, std::function<double()> time_provider = nullptr);

    ~ProtoLogger();

    void saveSerializedProto(const std::string& protobuf_type_full_name, const std::string& serialized_proto);

    template <typename ProtoType>
    inline void saveSerializedProto(const std::string& serialized_proto)
    {
        static_assert(std::is_base_of<google::protobuf::Message, ProtoType>::value,
                      "ProtoType has to be a protobuf message type");

        saveSerializedProto(ProtoType::GetDescriptor()->full_name(), serialized_proto);
    };

   private:
    void logProtobufs();

    std::string log_path_;
    std::string log_folder_;
    std::function<double()> time_provider_;
    double start_time_;
    bool stop_logging_;
    std::thread log_thread_;
    ThreadSafeBuffer<std::pair<std::string, std::string>> buffer_;
    const Duration BUFFER_BLOCK_TIMEOUT = Duration::fromSeconds(0.1);

    // TODO (NIMA): Move to constants and pybind to python
    const std::string REPLAY_FILE_PREFIX                      = "proto_";
    const std::string REPLAY_FILE_EXTENSION                   = "replay";
    const std::string REPLAY_FILE_TIME_FORMAT                 = "%Y_%m_%d_%H_%M_%S";
    const std::string REPLAY_METADATA_DELIMETER               = "!#!";
    static constexpr unsigned int PROTOBUF_BUFFER_SIZE        = 1000;
    static constexpr unsigned int REPLAY_MAX_CHUNK_SIZE_BYTES = 1024 * 1024;  // 1 MB
};
