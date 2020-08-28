#include "software/backend/replay_logging/replay_logger.h"

#include <google/protobuf/util/delimited_message_util.h>

#include <fstream>

#include "software/logger/logger.h"

namespace fs = std::experimental::filesystem;

ReplayLogger::ReplayLogger(const std::string& out_dir_path, int _msgs_per_chunk)
    : FirstInFirstOutThreadedObserver<SensorProto>(2000),
      current_chunk(),
      current_chunk_idx(0),
      output_dir_path(out_dir_path),
      msgs_per_chunk(_msgs_per_chunk)
{
    // check if directory exists, if not make a directory
    if (fs::exists(output_dir_path))
    {
        if (!fs::is_directory(output_dir_path))
        {
            throw std::invalid_argument(out_dir_path + " exists and is not a directory!");
        }
        if (!fs::is_empty(output_dir_path))
        {
            // this is better behavior than either adding more chunks to the same
            // directory (and having one directory end up with multiple replays) or
            // silently overwriting and destroying a previous replay_logging
            throw std::invalid_argument(out_dir_path +
                                        " is not empty! Find another directory!");
        }
    }
    else
    {
        fs::create_directory(output_dir_path);
        LOG(INFO) << "Created directory " << output_dir_path;
    }

    LOG(INFO) << "Logging to " << output_dir_path.string();
}


ReplayLogger::~ReplayLogger()
{
    saveCurrentChunk();
}


void ReplayLogger::onValueReceived(SensorProto msg)
{
    current_chunk.mutable_replay_msgs()->Add(std::move(msg));
    if (current_chunk.replay_msgs_size() >= msgs_per_chunk)
    {
        saveCurrentChunk();
        nextChunk();
    }
}


void ReplayLogger::nextChunk()
{
    current_chunk_idx++;
    current_chunk.Clear();
}


void ReplayLogger::saveCurrentChunk()
{
    fs::path chunk_path = output_dir_path / std::to_string(current_chunk_idx);
    std::ofstream chunk_ofstream(chunk_path);
    auto result = google::protobuf::util::SerializeDelimitedToOstream(current_chunk,
                                                                      &chunk_ofstream);
    if (!result)
    {
        LOG(WARNING) << "Failed to serialize chunk to output filestream: " << chunk_path;
    }
    else
    {
        LOG(DEBUG) << "Successfully saved chunk " << current_chunk_idx << "to disk";
    }
}
