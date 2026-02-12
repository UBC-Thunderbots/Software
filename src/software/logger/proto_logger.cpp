#include "proto_logger.h"

#include <google/protobuf/message.h>

#include <chrono>
#include <ctime>
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
        auto logger = GenericLogger::createLogger(log_folder_, std::to_string(replay_index) + "." + REPLAY_FILE_EXTENSION);

        if (!logger)
        {
            std::cerr << "Stopping ProtoLogger logger thread!" << std::endl;
            return;
        }

        // Start every replay file with the metadata, which includes the file format
        // version. This allows us to keep backwards compatibility as the replay file
        // format evolves.
        std::string file_metadata =
            REPLAY_FILE_VERSION_PREFIX + std::to_string(REPLAY_FILE_VERSION) + "\n";
        if (!logger->log(file_metadata.c_str()))
        {
            std::cerr << "ProtoLogger: Failed to write metadata to log file: "
                      << logger->getLogFileName() << std::endl;
        }

        std::cout << "####### \n\n\n\n\n LOGGER WORKS  2\n\n\n\n\n #########" << std::endl;


        while (!shouldStopLogging())
        {            std::cout << "####### \n\n\n\n\n LOGGER WORKS 7 \n\n\n\n\n #########" << std::endl;

            auto serialized_proto_opt =
                buffer_.popLeastRecentlyAddedValue(BUFFER_BLOCK_TIMEOUT);
            if (!serialized_proto_opt.has_value())
            {
                // Timed out without getting a new value
                continue;
            }

        std::cout << "####### \n\n\n\n\n LOGGER WORKS 3 \n\n\n\n\n #########" << std::endl;


            const auto& [proto_full_name, serialized_proto, receive_time_sec] =
                serialized_proto_opt.value();

            // Write the log entry to the file with the format:
            std::string log_entry =
                createLogEntry(proto_full_name, serialized_proto, receive_time_sec);


            std::cout << "####### \n\n\n\n\n LOGGER WORKS 4 \n\n\n\n\n #########" << std::endl;

            // Check if write was successful
            if (!logger->log(log_entry.c_str()))
            {            std::cout << "####### \n\n\n\n\n LOGGER WORKS 6 \n\n\n\n\n #########" << std::endl;

                // Only log every FAILED_LOG_PRINT_FREQUENCY times to avoid
                // spamming the console if the error persists.
                if (failed_logs_frequency_counter_ == 0)
                {
                    std::cerr << "ProtoLogger: Failed to write " << proto_full_name
                              << " to log file: " << logger->getLogFileName() << " "
                              << std::to_string(failed_logs_frequency_counter_)
                              << " times" << std::endl;
                }
                failed_logs_frequency_counter_ =
                    (failed_logs_frequency_counter_ + 1) % FAILED_LOG_PRINT_FREQUENCY;
            }

        std::cout << "####### \n\n\n\n\n LOGGER WORKS 5\n\n\n\n\n #########" << std::endl;


            // Limit the size of each replay chunk
            if (logger->getLogFileSize() > REPLAY_MAX_CHUNK_SIZE_BYTES)
            {

        std::cout << "####### \n\n\n\n\n LOGGER WORKS 6 \n\n\n\n\n #########" << std::endl;

                break;
            }
        }

        std::cout << "####### \n\n\n\n\n LOGGER WORKS 7 \n\n\n\n\n #########" << std::endl;


        logger->close();
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
