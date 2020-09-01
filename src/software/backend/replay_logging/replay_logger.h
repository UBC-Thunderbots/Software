#pragma once
#include <experimental/filesystem>

#include "software/multithreading/first_in_first_out_threaded_observer.h"
#include "software/proto/replay_msg.pb.h"
#include "software/proto/sensor_msg.pb.h"


class ReplayLogger : public FirstInFirstOutThreadedObserver<SensorProto>
{
   public:
    /**
     * Constructs a Replay Logger. _msgs_per_chunk is a parameter that sets
     * how many individual SensorProto's go into one replay_logging 'chunk' file.
     * We separate replays into files of a certain number of messages to
     * reduce the amount of lost data in the case of a crash.
     *
     * @param output_directory The absolute path of the directory that we output
     *                         replay_logging chunk files to.
     * @param _msgs_per_chunk number of messages per chunk
     */
    explicit ReplayLogger(const std::string& output_directory,
                          int _msgs_per_chunk = DEFAULT_MSGS_PER_CHUNK);

    // if we allow copying of a `ReplayLogger`, we could end up with 2 `ReplayLogger`s
    // writing over each other and possibly resulting in lost data
    ReplayLogger(const ReplayLogger&) = delete;
    ~ReplayLogger() override;

   private:
    /**
     * Adds a SensorProto to the current chunk. This will also save it to disk and
     * clear the chunk in memory.
     *
     * @param frame a SensorProto
     */
    void onValueReceived(SensorProto msg) override;

    /**
     * Increments the chunk index of the file we are writing to.
     */
    void nextChunk();

    /**
     * Saves the current chunk to a file in the output directory with a filename that is
     * the index of the chunk.
     */
    void saveCurrentChunk();

    static constexpr int DEFAULT_MSGS_PER_CHUNK = 1000;

    ReplayProto current_chunk;
    size_t current_chunk_idx;
    std::experimental::filesystem::path output_dir_path;
    const int msgs_per_chunk;
};
