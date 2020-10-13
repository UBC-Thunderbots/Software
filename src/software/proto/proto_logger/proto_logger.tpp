#include <google/protobuf/util/delimited_message_util.h>

#include <fstream>

#include "software/logger/logger.h"
#include "software/proto/proto_logger/proto_logger.h"

template <typename Msg>
ProtoLogger<Msg>::ProtoLogger(
    const std::string& output_directory, int _msgs_per_chunk,
    std::optional<std::function<bool(const Msg&, const Msg&)>> message_sort_comparator)
    : FirstInFirstOutThreadedObserver<Msg>(2000),
      current_chunk(),
      current_chunk_idx(0),
      output_dir_path(output_directory),
      msgs_per_chunk(_msgs_per_chunk),
      sort_comparator(message_sort_comparator)
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
    if (sort_comparator)
    {
        // if a function is passed in to compare the chunks to sort them, use it
        // to sort the outgoing chunk
        std::sort(current_chunk.mutable_messages()->begin(),
                  current_chunk.mutable_messages()->end(),
                  [this](const google::protobuf::Any& l, const google::protobuf::Any& r) {
                        // we have to convert the Any's back into Msg here in order to sort them
                        // and this also provides a cleaner interface externally for the sort comparator
                      Msg lhs;
                      l.UnpackTo(&lhs);
                      Msg rhs;
                      r.UnpackTo(&rhs);
                      return (*sort_comparator)(lhs, rhs);
                  });
    }

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
