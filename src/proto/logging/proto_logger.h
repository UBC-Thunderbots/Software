#pragma once
#include <filesystem>

#include "proto/repeated_any_msg.pb.h"
#include "software/multithreading/first_in_first_out_threaded_observer.h"

template <typename MsgT>
class ProtoLogger : public FirstInFirstOutThreadedObserver<MsgT>
{
    static_assert(
        std::is_base_of_v<google::protobuf::Message, MsgT>,
        "ProtoLogger can only be instantiated with a protobuf message as template parameter!");

   public:
    /**
     * Constructs a Protobuf Logger. _msgs_per_chunk is a parameter that sets
     * how many individual MsgT's go into one 'chunk' file. We separate replays into files
     * of a certain number of messages to reduce the amount of lost data in the case of a
     * crash.
     *
     * @param output_directory The absolute path of the directory that we output
     *                         RepeatedAnyMsg chunk files to.
     * @param _msgs_per_chunk number of messages per chunk
     */
    explicit ProtoLogger(const std::string& output_directory,
                         int _msgs_per_chunk = DEFAULT_MSGS_PER_CHUNK,
                         std::optional<std::function<bool(const MsgT&, const MsgT&)>>
                             message_sort_comparator = std::nullopt);

    // if we allow copying of a `ProtoLogger`, we could end up with 2 `ProtoLogger`s
    // writing over each other and possibly resulting in lost data
    ProtoLogger(const ProtoLogger&) = delete;
    ~ProtoLogger() override;

    static constexpr int DEFAULT_MSGS_PER_CHUNK = 1000;

    /**
     * Adds a MsgT to the current chunk. This will also save it to disk and clear the
     * chunk in memory, if the chunk contains `msgs_per_chunk` messages after the
     * addition.
     *
     * @param frame a MsgT
     */
    void onValueReceived(MsgT msg) override;

    /**
     * Safely save the current replay chunk into the output directory. Calls
     * saveCurrentChunkHelper() *after* acquiring the chunk_mutex, this can be
     * called from any thread.
     */
    void saveCurrentChunk();

   private:
    /**
     * Increments the chunk index of the file we are writing to.
     */
    void nextChunk();

    /**
     * Saves the current chunk to a file in the output directory with a filename that is
     * the index of the chunk.
     */
    void saveCurrentChunkHelper();

    RepeatedAnyMsg current_chunk;
    size_t current_chunk_idx;
    std::filesystem::path output_dir_path;
    const int msgs_per_chunk;
    std::optional<std::function<bool(MsgT, MsgT)>> sort_comparator;
    // this allows us to save ProtoLog's from the main thread
    std::mutex chunk_mutex;
};


#include "proto/logging/proto_logger.tpp"
