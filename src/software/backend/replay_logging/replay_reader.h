#pragma once
#include <experimental/filesystem>

#include "software/proto/replay_msg.pb.h"
#include "software/proto/sensor_msg.pb.h"

class ReplayReader
{
   public:
    /**
     * Constructs a `ReplayReader` that will read numerically-named files containing
     * a delimited protobuf message with type `ReplayProto`, which itself contains
     * multiple `SensorProto`s.
     *
     * @param _replay_dir The directory that we want to read replay proto messages from
     */
    explicit ReplayReader(const std::string& _replay_dir);

    /**
     * Returns the next recorded message from the replay chunk files in the replay
     * directory. This will increment the chunk index if we have reached the end of the
     * current chunk. Returns std::nullopt if there are no more messages available in
     * the current chunk, and there are no more chunks in the current directory.
     *
     * @return the next recorded SensorProto if available, nullopt otherwise.
     */
    std::optional<SensorProto> getNextMsg();

   private:
    /**
     * Returns true if another recorded message is available, and false otherwise.
     *
     * @return true if there is another message available
     */
    bool hasNextMsg() const;

    /**
     * Increments the chunk index currently being played from, and reads it from the file.
     */
    void nextChunk();

    /**
     * Reads a delimited protobuf file from a file located at the given path.
     *
     * @param file_path the path of the file containing a delimited protobuf file.
     * @return a ReplayProto object that has been read from the given path.
     */
    static ReplayProto readDelimitedReplayProtobufFile(
        const std::experimental::filesystem::path& file_path);

    size_t max_chunk_idx;
    size_t cur_chunk_idx;
    int cur_msg_idx;
    ReplayProto cur_chunk;
    std::experimental::filesystem::path replay_dir;
};
