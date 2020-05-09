#pragma once
#include <experimental/filesystem>
#include "software/multithreading/threaded_observer.h"
#include "software/proto/tbots_sensor_proto.pb.h"
#include "software/proto/tbots_replay.pb.h"


class ReplayLogger : public ThreadedObserver<TbotsSensorProto>
{
   public:
    explicit ReplayLogger(const std::string& output_directory);
    ReplayLogger(const ReplayLogger&) = delete;
    ~ReplayLogger() override;
   private:
    void onValueReceived(TbotsSensorProto frame) override;
    void nextChunk();
    void saveCurrentChunk();

    static constexpr int FRAMES_PER_CHUNK = 300;

    TbotsReplay current_chunk;
    size_t current_chunk_idx;
    std::experimental::filesystem::path output_dir_path;
};