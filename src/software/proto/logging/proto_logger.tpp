#include <google/protobuf/util/delimited_message_util.h>

#include <fstream>

#include "software/logger/logger.h"
#include "software/proto/logging/proto_logger.h"

template <typename MsgT>
ProtoLogger<MsgT>::ProtoLogger(
    const std::string& output_directory, int _msgs_per_chunk,
    std::optional<std::function<bool(const MsgT&, const MsgT&)>> message_sort_comparator)
    : FirstInFirstOutThreadedObserver<MsgT>(2000),
      current_chunk(),
      current_chunk_idx(0),
      output_dir_path(output_directory),
      msgs_per_chunk(_msgs_per_chunk),
      sort_comparator(message_sort_comparator),
      chunk_mutex()
{
    std::lock_guard<std::mutex> lock(chunk_mutex);
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

    // set the current chunk's message_type to the name of MsgT's type
    *current_chunk.mutable_message_type() = TYPENAME(MsgT);

    LOG(INFO) << "Logging " << TYPENAME(MsgT) << " to " << output_dir_path.string();
}

template <typename MsgT>
ProtoLogger<MsgT>::~ProtoLogger()
{
    // saveCurrentChunk will also take the chunk_mutex lock
    saveCurrentChunk();
}

template <typename MsgT>
void ProtoLogger<MsgT>::onValueReceived(MsgT msg)
{
    std::lock_guard<std::mutex> lock(chunk_mutex);
    current_chunk.add_messages()->PackFrom(std::move(msg));
    if (current_chunk.messages_size() >= msgs_per_chunk)
    {
        saveCurrentChunkHelper();
        nextChunk();
    }
}

template <typename MsgT>
void ProtoLogger<MsgT>::saveCurrentChunk()
{
    std::lock_guard<std::mutex> lock(chunk_mutex);
    saveCurrentChunkHelper();
}

template <typename MsgT>
void ProtoLogger<MsgT>::nextChunk()
{
    current_chunk_idx++;
    current_chunk.Clear();
}

template <typename MsgT>
void ProtoLogger<MsgT>::saveCurrentChunkHelper()
{
    if (sort_comparator)
    {
        // if a function is passed in to compare the chunks to sort them, use it
        // to sort the outgoing chunk
        std::sort(current_chunk.mutable_messages()->begin(),
                  current_chunk.mutable_messages()->end(),
                  [this](const google::protobuf::Any& l, const google::protobuf::Any& r) {
                      // we have to convert the Any's back into MsgT here in order to sort
                      // them and this also provides a cleaner interface externally for
                      // the sort comparator
                      MsgT lhs;
                      l.UnpackTo(&lhs);
                      MsgT rhs;
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
        LOG(DEBUG) << "Successfully saved " << TYPENAME(MsgT) << " chunk "
                   << current_chunk_idx << " to disk";
    }
}
