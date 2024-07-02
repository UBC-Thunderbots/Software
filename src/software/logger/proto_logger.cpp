#include "proto_logger.h"

#include <google/protobuf/message.h>
#include <zlib.h>

#include <chrono>
#include <ctime>
#include <experimental/filesystem>
#include <fstream>
#include <iomanip>
#include <optional>
#include <vector>

#include "base64.h"
#include "shared/constants.h"

ProtoLogger::ProtoLogger(const std::string& log_path,
                         std::function<double()> time_provider,
                         const bool friendly_colour_yellow)
    : log_path_(log_path),
      time_provider_(time_provider),
      stop_logging_(false),
      friendly_colour_yellow_(friendly_colour_yellow),
      buffer_(PROTOBUF_BUFFER_SIZE, true)
{
    start_time_ = time_provider_(); // TODO (NIMA): Consider making the start time, the time of the first protobuf received

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
    flushAndStopLogging();
}

void ProtoLogger::saveSerializedProto(const std::string& protobuf_type_full_name,
                                      const std::string& serialized_proto)
{
    buffer_.push({
        .protobuf_type_full_name  = protobuf_type_full_name,
        .serialized_proto = serialized_proto,
        .receive_time_sec = time_provider_() - start_time_,
    });
}

void ProtoLogger::logProtobufs()
{
    unsigned int replay_index = 0;

    try
    {
        while (!shouldStopLogging())
        {
            std::string log_file_path =
                log_folder_ + std::to_string(replay_index) + "." + REPLAY_FILE_EXTENSION;

            gzFile gz_file = gzopen(log_file_path.c_str(), "wb");
            if (!gz_file)
            {
                std::string error_msg =
                    "Failed to open gzip file. Error: " + std::string(strerror(errno));
                throw std::runtime_error(error_msg);
            }

            while (!shouldStopLogging())
            {
                auto serialized_proto_opt =
                    buffer_.popLeastRecentlyAddedValue(BUFFER_BLOCK_TIMEOUT);
                if (!serialized_proto_opt.has_value())
                {
                    // Timed out without getting a new value
                    continue;
                }

                const auto& [proto_full_name, serialized_proto, receive_time_sec] =
                    serialized_proto_opt.value();

                // Write the log entry to the file with the format:
                // <time>,<protobuf_type_full_name>,<base64_encoded_serialized_proto>
                std::stringstream log_entry_ss;
                log_entry_ss << receive_time_sec << REPLAY_METADATA_DELIMETER
                             << proto_full_name << REPLAY_METADATA_DELIMETER
                             << base64_encode(serialized_proto) << "\n";
                std::string log_entry = log_entry_ss.str();

                int num_bytes_written = gzwrite(gz_file, log_entry.c_str(),
                        static_cast<unsigned>(log_entry.size()));

                // Check if write was successful
                if (num_bytes_written == 0)
                {
                    if (num_failed_logs_++ % FAILED_LOG_PRINT_FREQUENCY == 0)
                    {
                        std::cerr << "Failed to write " << proto_full_name << " to log file: " << log_file_path << " " << std::to_string(num_failed_logs_) << " times" << std::endl;
                    }
                }

                // Limit the size of each replay chunk
                if (gzoffset(gz_file) > REPLAY_MAX_CHUNK_SIZE_BYTES)
                {
                    break;
                }
            }

            gzclose(gz_file);
            replay_index++;
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "Exception detected in ProtoLogger: " << e.what() << std::endl;
    }
}

void ProtoLogger::updateTimeProvider(std::function<double()> time_provider)
{
    time_provider_ = time_provider;
}

bool ProtoLogger::shouldStopLogging() const
{
    if (!stop_logging_)
    {
        return false;
    }

    // If stop_logging_ is true, empty the buffer for up to 0.5 seconds to ensure all
    // logs are written
    double curr_time_sec = std::chrono::duration<double>(
            std::chrono::system_clock::now().time_since_epoch())
            .count();
    return (curr_time_sec - destructor_called_time_sec_) > MAX_TIME_TO_EXIT_FULL_SYSTEM_SEC || buffer_.empty();
}

void ProtoLogger::flushAndStopLogging()
{
    stop_logging_ = true;
    destructor_called_time_sec_ = std::chrono::duration<double>(
            std::chrono::system_clock::now().time_since_epoch())
            .count();
    if (log_thread_.joinable())
    {
        log_thread_.join();
    }

    // Print the command to run to watch the replay
    if (friendly_colour_yellow_)
    {
        std::cout
                << "\nTo watch the replay for the yellow team, go to the `src` folder and run \n./tbots.py run thunderscope --yellow_log  "
                << log_folder_ << std::endl;
    }
    else
    {
        std::cout
                << "\nTo watch the replay for the blue team, go to the `src` folder and run \n./tbots.py run thunderscope --blue_log  "
                << log_folder_ << std::endl;
    }
}
