#pragma once
#include <experimental/filesystem>

#include "software/multithreading/last_in_first_out_threaded_observer.h"
#include "software/proto/replay_msg.pb.h"
#include "software/proto/sensor_msg.pb.h"


class ReplayLogger : public LastInFirstOutThreadedObserver<SensorMsg>
{
   public:
    /**
     * Constructs a Replay Logger. _msgs_per_chunk is a parameter that sets
     * how many individual SensorMsg's go into one replay_logging 'chunk' file.
     * We separate replays into files of a certain number of messages to
     * reduce the amount of lost data in the case of a crash.
     * @param output_directory The directory that we output replay_logging chunk files to.
     * @param _msgs_per_chunk number of messages per chunk
     */
    explicit ReplayLogger(const std::string& output_directory,
                          int _msgs_per_chunk = DEFAULT_MSGS_PER_CHUNK);
    ReplayLogger(const ReplayLogger&) = delete;
    ~ReplayLogger() override;

   private:
    /**
     * Adds a SensorMsg to the current chunk. This will also save it to disk and
     * clear the chunk in memory.
     * @param frame a SensorMsg
     */
    void onValueReceived(SensorMsg msg) override;
    void nextChunk();
    void saveCurrentChunk();

    static constexpr int DEFAULT_MSGS_PER_CHUNK = 1000;

    ReplayMsg current_chunk;
    size_t current_chunk_idx;
    std::experimental::filesystem::path output_dir_path;
    const int msgs_per_chunk;
};
