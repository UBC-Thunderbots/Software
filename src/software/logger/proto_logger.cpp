#include "proto_logger.h"

#include <boost/filesystem.hpp>
//#include <boost/iostreams/filter/gzip.hpp>
//#include <boost/iostreams/filtering_stream.hpp>
#include <google/protobuf/message.h>
#include <experimental/filesystem>
#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
//#include <iostream>
//#include <memory>
#include <optional>
//#include <sstream>
//#include <stdexcept>
#include <zlib.h>
#include <vector>

#include "base64.h"

ProtoLogger::ProtoLogger(const std::string& log_path, std::function<double()> time_provider)
    : log_path_(log_path),
      time_provider_(time_provider),
      stop_logging_(false),
      buffer_(PROTOBUF_BUFFER_SIZE, true)
{
    start_time_ = time_provider_();

    // Create a folder for the logs with the formatted current time
    std::time_t t = std::time(nullptr);
    std::tm tm    = *std::localtime(&t);
    std::stringstream ss;
    ss << std::put_time(&tm, REPLAY_FILE_TIME_FORMAT.data());
    log_folder_ = log_path_ + "/" + REPLAY_FILE_PREFIX + ss.str() + "/";
    std::experimental::filesystem::create_directories(log_folder_);

    // Start logging in a separate thread
    log_thread_ = std::thread(&ProtoLogger::logProtobufs, this);
}

ProtoLogger::~ProtoLogger()
{
    stop_logging_ = true;
    if (log_thread_.joinable())
    {
        log_thread_.join();
    }
}

void ProtoLogger::saveSerializedProto(const std::string &protobuf_type_full_name, const std::string &serialized_proto)
{
    buffer_.push(std::make_pair(protobuf_type_full_name, serialized_proto));
}

void ProtoLogger::logProtobufs()
{
    int replay_index = 0;

    try
    {
        while (!stop_logging_)
        {
            std::string log_file_path =
                log_folder_ + std::to_string(replay_index) + "." + REPLAY_FILE_EXTENSION;
            std::ofstream file(log_file_path, std::ios_base::out | std::ios_base::binary);
            std::cout << "Writing to " << log_file_path << std::endl;

            if (!file.is_open())
            {
                throw std::runtime_error("Failed to open log file");
            }

            gzFile gz_file = gzopen(log_file_path.c_str(), "wb");
            if (!gz_file) { // TODO (NIMA): On failure, gzopen() shall return Z_NULL and may set errno accordingly.
                throw std::runtime_error("Failed to open gzip file");
            }

            while (!stop_logging_)
            {
                auto serialized_proto_opt = buffer_.popLeastRecentlyAddedValue(BUFFER_BLOCK_TIMEOUT);
                if (!serialized_proto_opt.has_value())
                {
                    // Timed out without getting a new value
                    continue;
                }

                double current_time = time_provider_() - start_time_;
                const auto& [proto_full_name, serialized_proto] =
                        serialized_proto_opt.value();
                std::stringstream log_entry_ss;
                log_entry_ss << current_time << REPLAY_METADATA_DELIMETER
                             << proto_full_name << REPLAY_METADATA_DELIMETER
                             << base64_encode(serialized_proto) << "\n";
                std::string log_entry = log_entry_ss.str();

                gzwrite(gz_file, log_entry.c_str(), static_cast<unsigned>(log_entry.size()));

                if (gzoffset(gz_file) > REPLAY_MAX_CHUNK_SIZE_BYTES)
                {
                    break;
                }
            }

            gzclose(gz_file);
            file.close();
            replay_index++;
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "Exception detected in ProtoLogger: " << e.what() << std::endl;
    }
}
