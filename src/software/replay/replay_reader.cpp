#include "replay_reader.h"

#include <google/protobuf/util/delimited_message_util.h>

#include <fstream>

// unfortunately protobuf forces me to write caveman code
extern "C"
{
#include <fcntl.h>
#include <unistd.h>
}

namespace fs = std::experimental::filesystem;

namespace
{
    TbotsReplay readDelimitedReplayProtobufFile(const fs::path& file_path)
    {
        // imagine having to write caveman code to read protobuf msgs from files
        TbotsReplay msg;
        int fd = open(file_path.c_str(), O_RDONLY);

        if (fd < 0)
        {
            throw std::invalid_argument(std::string("Failed to open() ") +
                                        file_path.string() + " with error " +
                                        std::to_string(fd));
        }

        auto file_input = std::make_unique<google::protobuf::io::FileInputStream>(fd);
        auto coded_input =
            std::make_unique<google::protobuf::io::CodedInputStream>(file_input.get());
        bool result = google::protobuf::util::ParseDelimitedFromCodedStream(
            &msg, coded_input.get(), nullptr);
        if (!result)
        {
            throw std::invalid_argument("Failed to parse protobuf from file " +
                                        file_path.string());
        }
        close(fd);
        return msg;
    }
}  // namespace

template <typename T>
std::ostream& operator<<(std::ostream& os, const std::vector<T>& set)
{
    os << '{';
    for (const auto& val : set)
    {
        os << val << ' ';
    }
    os << '}';
    return os;
}

ReplayReader::ReplayReader(const std::string& _replay_dir)
    : replay_dir(_replay_dir), cur_msg_idx(0)
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
    // filter the filenames in the replay out directory to only numeric filenames
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
            " contains no files with numerical names! It is not a valid replay directory!");
    }

    std::sort(chunk_indices.begin(), chunk_indices.end());
    cur_chunk_idx   = *chunk_indices.begin();
    max_chunk_idx   = *chunk_indices.rbegin();
    auto chunk_path = replay_dir / std::to_string(cur_chunk_idx);

    cur_chunk.CopyFrom(::readDelimitedReplayProtobufFile(chunk_path));
}

std::optional<SensorMsg> ReplayReader::getNextMsg()
{
    if (!hasNextMsg())
    {
        return std::nullopt;
    }

    if (cur_msg_idx >= cur_chunk.replay_frames_size())
    {
        nextChunk();
    }

    auto ret = cur_chunk.replay_frames(cur_msg_idx);
    cur_msg_idx++;
    return ret;
}

bool ReplayReader::hasNextMsg() const
{
    return !(cur_chunk_idx == max_chunk_idx &&
             cur_msg_idx == cur_chunk.replay_frames_size());
}

void ReplayReader::nextChunk()
{
    cur_chunk_idx++;
    auto chunk_path = replay_dir / std::to_string(cur_chunk_idx);

    if (!fs::is_regular_file(chunk_path))
    {
        throw std::out_of_range("Reached end of replay!");
    }

    cur_chunk   = ::readDelimitedReplayProtobufFile(chunk_path);
    cur_msg_idx = 0;
}
