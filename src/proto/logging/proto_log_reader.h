#pragma once
#include <experimental/filesystem>
#include <optional>

#include "proto/repeated_any_msg.pb.h"
#include "software/util/typename/typename.h"


class ProtoLogReader
{
   public:
    /**
     * Constructs a `ProtoLogReader` that will read numerically-named files containing
     * a delimited protobuf message with type `RepeatedAnyMsg`, which itself contains
     * multiple `Any`s, each of which can contain a Protobuf message of any type.
     *
     * @param _replay_dir The directory that we want to read replay proto messages from
     */
    explicit ProtoLogReader(const std::string& _replay_dir);

    /**
     * Returns the next `Any` message from the current `RepeatedAnyMsg` message, unpacked
     * into the desired type `MsgT`. Throws std::invalid_argument if unsuccessful.
     *
     * @return the next recorded SensorProto if available, nullopt otherwise.
     */
    template <typename MsgT>
    std::optional<MsgT> getNextMsg();

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
     * Reads a delimited RepeatedAnyMsg protobuf message from a file located at the given
     * path.
     *
     * @param file_path the path of the file containing a delimited protobuf file.
     * @return a RepeatedAnyMsg object that has been read from the given path.
     */
    static RepeatedAnyMsg readDelimitedRepeatedAnyMsgFile(
        const std::experimental::filesystem::path& file_path);

    size_t max_chunk_idx;
    size_t cur_chunk_idx;
    int cur_msg_idx;
    RepeatedAnyMsg cur_chunk;
    std::experimental::filesystem::path replay_dir;
};

template <typename MsgT>
std::optional<MsgT> ProtoLogReader::getNextMsg()
{
    static_assert(std::is_base_of_v<google::protobuf::Message, MsgT>,
                  "MsgT must be a derived class of google::protobuf::Message!");

    if (!hasNextMsg())
    {
        return std::nullopt;
    }

    if (cur_msg_idx >= cur_chunk.messages_size())
    {
        nextChunk();
    }

    MsgT ret;
    bool success = cur_chunk.messages(cur_msg_idx).UnpackTo(&ret);

    if (!success)
    {
        throw std::invalid_argument("Failed to parse " + TYPENAME(MsgT) + " into " +
                                    cur_chunk.messages(cur_msg_idx).type_url());
    }

    cur_msg_idx++;
    return ret;
}
