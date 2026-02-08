#include "proto_logger.h"

#include <google/protobuf/message.h>
#include <zlib.h>

#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <optional>
#include <vector>

#include "base64.h"
#include "compat_flags.h"
#include "shared/constants.h"

ProtoLogger::ProtoLogger(const std::string& log_path,
                         std::function<double()> time_provider,
                         const bool friendly_colour_yellow)
    : log_path_(log_path),
      time_provider_(time_provider),
      friendly_colour_yellow_(friendly_colour_yellow),
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
    fs::create_directories(log_folder_);

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
        .protobuf_type_full_name = protobuf_type_full_name,
        .serialized_proto        = serialized_proto,
        .receive_time_sec        = time_provider_() - start_time_,
    });
}

void ProtoLogger::logProtobufs()
{
    unsigned int replay_index = 0;

    while (!shouldStopLogging())
    {
        std::string log_file_path =
            log_folder_ + std::to_string(replay_index) + "." + REPLAY_FILE_EXTENSION;

        gzFile gz_file = gzopen(log_file_path.c_str(), "wb");
        if (!gz_file)
        {
            std::cerr << "ProtoLogger: Failed to open gzip log file: " << log_file_path
                      << " Error: " + std::string(strerror(errno))
                      << "\nStopping ProtoLogger logger thread!" << std::endl;
            return;
        }

        // Start every replay file with the metadata, which includes the file format
        // version. This allows us to keep backwards compatibility as the replay file
        // format evolves.
        std::string file_metadata =
            REPLAY_FILE_VERSION_PREFIX + std::to_string(REPLAY_FILE_VERSION) + "\n";
        int num_bytes_written = gzwrite(gz_file, file_metadata.c_str(),
                                        static_cast<unsigned>(file_metadata.size()));
        if (num_bytes_written != static_cast<int>(file_metadata.size()))
        {
            std::cerr << "ProtoLogger: Failed to write metadata to log file: "
                      << log_file_path << std::endl;
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
            std::string log_entry =
                createLogEntry(proto_full_name, serialized_proto, receive_time_sec);
            num_bytes_written = gzwrite(gz_file, log_entry.c_str(),
                                        static_cast<unsigned>(log_entry.size()));

            // Check if write was successful
            if (num_bytes_written != static_cast<int>(log_entry.size()))
            {
                // Only log every FAILED_LOG_PRINT_FREQUENCY times to avoid
                // spamming the console if the error persists.
                if (failed_logs_frequency_counter_ == 0)
                {
                    std::cerr << "ProtoLogger: Failed to write " << proto_full_name
                              << " to log file: " << log_file_path << " "
                              << std::to_string(failed_logs_frequency_counter_)
                              << " times" << std::endl;
                }
                failed_logs_frequency_counter_ =
                    (failed_logs_frequency_counter_ + 1) % FAILED_LOG_PRINT_FREQUENCY;
            }

            // Limit the size of each replay chunk
            if (gzoffset(gz_file) > REPLAY_MAX_CHUNK_SIZE_BYTES)
            {
                break;
            }
        }

        int result = gzclose(gz_file);
        if (result != Z_OK)
        {
            std::cerr << "ProtoLogger: Failed to close log file: " << log_file_path
                      << " with error " << result << std::endl;
        }
        replay_index++;
    }
}

std::string ProtoLogger::createLogEntry(const std::string& proto_full_name,
                                        const std::string& serialized_proto,
                                        const double receive_time_sec)
{
    // <time>,<protobuf_type_full_name>,<base64_encoded_serialized_proto>
    std::stringstream log_entry_ss;
    log_entry_ss << receive_time_sec << REPLAY_METADATA_DELIMITER << proto_full_name
                 << REPLAY_METADATA_DELIMITER << base64_encode(serialized_proto) << "\n";
    return log_entry_ss.str();
}

void ProtoLogger::updateTimeProvider(std::function<double()> time_provider)
{
    time_provider_ = time_provider;
}

bool ProtoLogger::shouldStopLogging() const
{
    if (!stop_logging_.load())
    {
        return false;
    }

    // If stop_logging_ is true, empty the buffer for up to 0.5 seconds to ensure all
    // logs are written
    double curr_time_sec =
        std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch())
            .count();
    return buffer_.empty() || (curr_time_sec - destructor_called_time_sec_) >
                                  MAX_TIME_TO_EXIT_FULL_SYSTEM_SEC;
}

void ProtoLogger::flushAndStopLogging()
{
    destructor_called_time_sec_ =
        std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch())
            .count();
    stop_logging_.store(true);

    if (log_thread_.joinable())
    {
        log_thread_.join();
    }

    // In blue, print the command to run to watch the replay
    if (friendly_colour_yellow_)
    {
        std::cout
            << "\nTo watch the replay for the yellow team, go to the `src` folder and run \n\033[34m./tbots.py run thunderscope --yellow_log  "
            << log_folder_ << "\033[m" << std::endl;
    }
    else
    {
        std::cout
            << "\nTo watch the replay for the blue team, go to the `src` folder and run \n\033[34m./tbots.py run thunderscope --blue_log  "
            << log_folder_ << "\033[m" << std::endl;
    }
}
