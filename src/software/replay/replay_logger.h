#pragma once
#include <experimental/filesystem>
#include "software/multithreading/threaded_observer.h"
#include "software/proto/sensor_msg.pb.h"
#include "software/proto/tbots_replay.pb.h"


class ReplayLogger : public OrderedThreadedObserver<SensorMsg>
{
   public:
    explicit ReplayLogger(const std::string& output_directory,
                          int _frames_per_chunk = DEFAULT_FRAMES_PER_CHUNK);
    ReplayLogger(const ReplayLogger&) = delete;
    ~ReplayLogger() override;
   private:
    void onValueReceived(SensorMsg frame) override;
    void nextChunk();
    void saveCurrentChunk();

    static constexpr int DEFAULT_FRAMES_PER_CHUNK = 1000;

    TbotsReplay current_chunk;
    size_t current_chunk_idx;
    std::experimental::filesystem::path output_dir_path;
    const int frames_per_chunk;
};