#include "replay_reader.h"
#include <fstream>

namespace fs = std::experimental::filesystem;

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

    if (chunk_indices.size() == 0) {
        throw std::invalid_argument(replay_dir.string() +
        " contains no files with number names! It is not a valid replay directory!");
    }

    // TODO: timestamp functionality, including seeking by timestamp
    // std::set::begin() is guaranteed to point to the smallest item
    cur_chunk_idx = *chunk_indices.begin();
    auto cur_chunk_ifstream = std::ifstream(replay_dir / std::to_string(cur_chunk_idx));
    cur_chunk.ParseFromIstream(&cur_chunk_ifstream);
}

std::optional<TbotsSensorProto> ReplayReader::getNextFrame()
{
    if (cur_chunk_idx >= cur_chunk.replay_frames_size()) {
        try {
            nextChunk();
        } catch (std::out_of_range&) {
            return std::nullopt;
        }
    }

    auto ret = cur_chunk.replay_frames(cur_frame_idx);
    cur_frame_idx++;
    return ret;
}

void ReplayReader::nextChunk() {
    cur_chunk_idx++;
    auto cur_chunk_ifstream = std::ifstream(replay_dir / std::to_string(cur_chunk_idx));
    bool success = cur_chunk.ParseFromIstream(&cur_chunk_ifstream);
    if (!success) {
        throw std::out_of_range("Reached end of replay!");
    }
    cur_frame_idx = 0;
}
