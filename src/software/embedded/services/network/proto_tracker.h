#include <queue>
#include <string>

#include "software/logger/logger.h"

class ProtoTracker
{
   public:
    /**
     * Tracks recent proto loss given the sequence numbers of received protos
     *
     * @param type The name of the type of protos being tracked
     */
    ProtoTracker(const std::string& type);

    /**
     * When a new sequence number is sent, the ProtoTracker updates the proto tracking
     *
     * @param seq_num The sequence number of the newly received proto
     */
    void send(uint64_t seq_num);

    /**
     * @return a boolean indicating whether the last received proto was valid
     */
    bool isLastValid();

    /**
     * @return a float equal to the proto loss rate
     */
    float getLossRate();

   private:
    /**
     * Private function for calculating the proto loss rate
     *
     * @param seq_num The sequence number of the newly received protobuf
     * @return a float equal to the proto loss rate
     */
    float calculate_proto_loss_rate(uint64_t seq_num);

    // Constants
    static constexpr uint8_t RECENT_PROTO_LOSS_PERIOD = 100;

    // Variables
    std::string proto_type;
    bool last_valid       = false;
    float proto_loss_rate = 0;

    // Queue of the sequence numbers of received protos in the past
    // RECENT_PROTO_LOSS_PERIOD protos
    std::queue<uint64_t> recent_proto_seq_nums;
};
