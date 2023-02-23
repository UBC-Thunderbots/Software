#include <queue>
#include <string>

#include "software/logger/logger.h"

class PacketTracker
{
   public:
    /**
     * Tracks recent packet loss given the sequence numbers of received packets
     *
     * @param type The name of the type of protobufs being tracked
     */
    PacketTracker(const std::string& type);
    /**
     * When a new sequence number is sent, the PacketTracker updates the proto tracking
     * and logs a warning if loss rate is greater than the threshold
     *
     * @param seq_num The sequence number of the newly received protobuf
     */
    void send(uint64_t seq_num);
    /**
     * @return a boolean indicating whether the last received packet was valid
     */
    bool isLastValid();

   private:
    // Function for calculating the packet loss rate
    float calculate_packet_loss_rate(uint64_t seq_num);

    // Constants
    static constexpr float PACKET_LOSS_WARNING_THRESHOLD = 0.1f;
    static constexpr uint8_t RECENT_PACKET_LOSS_PERIOD   = 100;

    // Variables
    std::string proto_type;
    bool last_valid = false;

    // Queue of the sequence numbers of received protos in the past
    // RECENT_PACKET_LOSS_PERIOD protos
    std::queue<uint64_t> recent_proto_seq_nums;
};
