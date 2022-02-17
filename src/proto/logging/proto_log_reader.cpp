#include "proto_log_reader.h"

#include <google/protobuf/util/delimited_message_util.h>

#include <fstream>

namespace fs = std::filesystem;

RepeatedAnyMsg ProtoLogReader::readDelimitedRepeatedAnyMsgFile(const fs::path& file_path)
{
    std::ifstream file_ifstream(file_path, std::ios_base::in | std::ios_base::binary);
    auto file_input =
        std::make_unique<google::protobuf::io::IstreamInputStream>(&file_ifstream);
    auto coded_input =
        std::make_unique<google::protobuf::io::CodedInputStream>(file_input.get());

    RepeatedAnyMsg msg;
    bool result = google::protobuf::util::ParseDelimitedFromCodedStream(
        dynamic_cast<google::protobuf::MessageLite*>(&msg), coded_input.get(), nullptr);
    if (!result)
    {
        throw std::invalid_argument("Failed to parse protobuf from file " +
                                    file_path.string());
    }
    return msg;
}

ProtoLogReader::ProtoLogReader(const std::string& _replay_dir)
    : cur_msg_idx(0), replay_dir(_replay_dir)
{
    if (!fs::exists(replay_dir) || !fs::is_directory(replay_dir))
    {
        throw std::invalid_argument(replay_dir.string() +
                                    "does not exist or is not a directory!");
    }

    if (fs::is_empty(replay_dir))
    {
        throw std::invalid_argument(replay_dir.string() + " is empty!");
    }

    // we assume chunk indices are monotonically increasing, but not necessarily starting
    // at 0; this is so that we can trivially 'crop' replays
    fs::directory_iterator dir_it(replay_dir);
    // filter the filenames in the replay_logging out directory to only numeric filenames
    std::vector<size_t> chunk_indices;
    std::for_each(fs::begin(dir_it), fs::end(dir_it),
                  [&chunk_indices](const fs::directory_entry& dir_entry) {
                      try
                      {
                          auto fname_num = std::stoul(dir_entry.path().filename());
                          chunk_indices.emplace_back(fname_num);
                      }
                      catch (std::invalid_argument&)
                      {
                      };
                  });

    if (chunk_indices.size() == 0)
    {
        throw std::invalid_argument(
            replay_dir.string() +
            " contains no files with numerical names! It is not a valid replay_logging directory!");
    }

    std::sort(chunk_indices.begin(), chunk_indices.end());
    cur_chunk_idx   = *chunk_indices.begin();
    max_chunk_idx   = *chunk_indices.rbegin();
    auto chunk_path = replay_dir / std::to_string(cur_chunk_idx);

    cur_chunk.CopyFrom(readDelimitedRepeatedAnyMsgFile(chunk_path));
}


bool ProtoLogReader::hasNextMsg() const
{
    return !(cur_chunk_idx == max_chunk_idx && cur_msg_idx == cur_chunk.messages_size());
}

void ProtoLogReader::nextChunk()
{
    cur_chunk_idx++;
    auto chunk_path = replay_dir / std::to_string(cur_chunk_idx);

    if (!fs::is_regular_file(chunk_path))
    {
        throw std::out_of_range("Reached end of replay_logging!");
    }

    cur_chunk   = readDelimitedRepeatedAnyMsgFile(chunk_path);
    cur_msg_idx = 0;
}
