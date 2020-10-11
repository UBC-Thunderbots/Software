#include <google/protobuf/util/delimited_message_util.h>

#include <fstream>

#include "software/logger/logger.h"
#include "software/proto/proto_logger/proto_logger.h"

template <typename Msg>
ProtoLogger<Msg>::ProtoLogger(const std::string& output_directory, int _msgs_per_chunk)
    : FirstInFirstOutThreadedObserver<Msg>(2000),
      current_chunk(),
      current_chunk_idx(0),
      output_dir_path(output_directory),
      msgs_per_chunk(_msgs_per_chunk)
{
    // check if directory exists, if not make a directory
    if (std::experimental::filesystem::exists(output_dir_path))
    {
        if (!std::experimental::filesystem::is_directory(output_dir_path))
        {
            throw std::invalid_argument(output_directory +
                                        " exists and is not a directory!");
        }
        if (!std::experimental::filesystem::is_empty(output_dir_path))
        {
            // this is better behavior than either adding more chunks to the same
            // directory (and having one directory end up with multiple replays) or
            // silently overwriting and destroying a previous replay_logging
            throw std::invalid_argument(output_directory +
                                        " is not empty! Find another directory!");
        }
    }
    else
    {
        std::experimental::filesystem::create_directory(output_dir_path);
        LOG(INFO) << "Created directory " << output_dir_path;
    }

    // set the current chunk's message_type to the name of Msg's type
    *current_chunk.mutable_message_type() = TYPENAME(Msg);

    LOG(INFO) << "Logging to " << output_dir_path.string();
}

template <typename Msg>
ProtoLogger<Msg>::~ProtoLogger()
{
    saveCurrentChunk();
}

template <typename Msg>
void ProtoLogger<Msg>::onValueReceived(Msg msg)
{
    current_chunk.add_messages()->PackFrom(std::move(msg));
    if (current_chunk.messages_size() >= msgs_per_chunk)
    {
        saveCurrentChunk();
        nextChunk();
    }
}

template <typename Msg>
void ProtoLogger<Msg>::nextChunk()
{
    current_chunk_idx++;
    current_chunk.Clear();
}

template <typename Msg>
void ProtoLogger<Msg>::saveCurrentChunk()
{
    std::experimental::filesystem::path chunk_path =
        output_dir_path / std::to_string(current_chunk_idx);
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
