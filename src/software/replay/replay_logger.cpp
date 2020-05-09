//
// Created by jordan on 5/2/20.
//
#include "software/replay/replay_logger.h"
#include <fstream>

namespace fs = std::experimental::filesystem;

ReplayLogger::ReplayLogger(const std::string& out_dir_path) :
output_dir_path(out_dir_path), current_chunk(), current_chunk_idx(0)
{
    // check if directory exists, if not make a directory
    if (fs::exists(output_dir_path) && !fs::is_directory(output_dir_path)) {
        throw std::invalid_argument(out_dir_path + " exists and is not a directory!");
    }

    if (!fs::exists(output_dir_path))
    {
        fs::create_directory(output_dir_path);
    }
}


ReplayLogger::~ReplayLogger() {
    saveCurrentChunk();
}


void ReplayLogger::onValueReceived(TbotsSensorProto frame)
{
    current_chunk.mutable_replay_frames()->Add(dynamic_cast<TbotsSensorProto&&>(frame));
    if (current_chunk.replay_frames_size() > FRAMES_PER_CHUNK) {
        saveCurrentChunk();
        nextChunk();
    }
}


void ReplayLogger::nextChunk() {
    current_chunk_idx++;
    current_chunk.clear_replay_frames();
}
void ReplayLogger::saveCurrentChunk() {
    fs::path chunk_path = output_dir_path / std::to_string(current_chunk_idx);
    std::ofstream chunk_ofstream(chunk_path);
    current_chunk.SerializeToOstream(&chunk_ofstream);
}
