#pragma once
#include <experimental/filesystem>

#include "software/proto/tbots_replay.pb.h"
#include "software/proto/sensor_msg.pb.h"

class ReplayReader {
   public:
    explicit ReplayReader(const std::string& _replay_dir);
    std::optional<SensorMsg> getNextFrame();
   private:
    void nextChunk();
    size_t cur_chunk_idx;
    size_t cur_frame_idx;
    TbotsReplay cur_chunk;
    std::experimental::filesystem::path replay_dir;
};