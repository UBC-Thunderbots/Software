#include "replay_reader.h"
#include <fstream>
#include <google/protobuf/util/delimited_message_util.h>
extern "C" {
#include <unistd.h>
#include <fcntl.h>
}

namespace fs = std::experimental::filesystem;

template <typename T>
std::ostream& operator<<(std::ostream& os, const std::set<T>& set) {
    os << '{';
    for (const auto& val : set) {
        os << val << ' ';
    }
    os << '}';
    return os;
}

ReplayReader::ReplayReader(const std::string& _replay_dir)
: replay_dir(_replay_dir), cur_frame_idx(0)
{
    if (!fs::exists(replay_dir) || !fs::is_directory(replay_dir)) {
        throw std::invalid_argument(replay_dir.string() +
        "does not exist or is not a directory!");
    }

    if (fs::is_empty(replay_dir)) {
        throw std::invalid_argument(replay_dir.string() + " is empty!");
    }

    // we assume chunk indices are monotonically increasing, but not necessarily starting
    // at 0; this is so that we can trivially 'crop' replays
    fs::directory_iterator dir_it(replay_dir);
    // filter the filenames in the replay out directory to only numeric filenames
    std::set<size_t> chunk_indices;
    std::for_each(fs::begin(dir_it), fs::end(dir_it),
        [&chunk_indices](const fs::directory_entry& dir_entry) {
            try {
                auto fname_num = std::stoul(dir_entry.path().filename());
                chunk_indices.emplace(fname_num);
            } catch(std::invalid_argument&) {};
    });

    std::cout << chunk_indices << std::endl;

    if (chunk_indices.size() == 0) {
        throw std::invalid_argument(replay_dir.string() +
        " contains no files with number names! It is not a valid replay directory!");
    }

    // TODO: timestamp functionality, including seeking by timestamp
    // std::set::begin() is guaranteed to point to the smallest item
    cur_chunk_idx = *chunk_indices.begin();
    auto chunk_path = replay_dir / std::to_string(cur_chunk_idx);

    // imagine having to write caveman code to read protobuf msgs from files
    int fd = open(chunk_path.c_str(), O_RDONLY);
    auto file_input = std::make_unique<google::protobuf::io::FileInputStream>(fd);
    auto coded_input = std::make_unique<google::protobuf::io::CodedInputStream>(file_input.get());
    bool result = google::protobuf::util::ParseDelimitedFromCodedStream(&cur_chunk, coded_input.get(), nullptr);
    if (!result) {
        throw std::invalid_argument("Failed to parse protobuf from file " + chunk_path.string());
    }
    close(fd);
}

std::optional<SensorMsg> ReplayReader::getNextFrame()
{
    if (cur_frame_idx >= cur_chunk.replay_frames_size()) {
        try {
            nextChunk();
        } catch (std::out_of_range& e) {
            std::cout << "PEA BRAIN TIME " << e.what() << std::endl;
            return std::nullopt;
        }
    }

    auto ret = cur_chunk.replay_frames(cur_frame_idx);
    cur_frame_idx++;
    return ret;
}

void ReplayReader::nextChunk() {
    cur_chunk_idx++;
    auto chunk_path = replay_dir / std::to_string(cur_chunk_idx);

    if (!fs::is_regular_file(chunk_path)) {
        throw std::out_of_range("Reached end of replay!");
    }

    // TODO: move this monstrosity into its own function
    // imagine having to write caveman code to read protobuf msgs from files
    int fd = open(chunk_path.c_str(), O_RDONLY);
    auto file_input = std::make_unique<google::protobuf::io::FileInputStream>(fd);
    auto coded_input = std::make_unique<google::protobuf::io::CodedInputStream>(file_input.get());
    bool result = google::protobuf::util::ParseDelimitedFromCodedStream(&cur_chunk, coded_input.get(), nullptr);
    if (!result) {
        throw std::invalid_argument("Failed to parse protobuf from file " + chunk_path.string());
    }
    close(fd);
    cur_frame_idx = 0;
}
