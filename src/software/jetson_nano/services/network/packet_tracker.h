#include <queue>
#include <string>
#include "software/logger/logger.h"

class PacketTracker
{
   public:
//    PacketTracker();  // constructor
    PacketTracker(const std::string& type);  // constructor
    void send(uint64_t seq_num);        // sends a proto
    bool isLastValid(); // checks if the last proto is valid
   private:
    // Functions
    float calculate_packet_loss_rate(uint64_t seq_num);  // calculate_packet_loss_rate loss rate
    // Variables
    // Queue of the sequence numbers of received protos in the past
    // RECENT_PACKET_LOSS_PERIOD protos
    std::string proto_type;
    std::queue<uint64_t> recent_proto_seq_nums;

    bool last_valid = false;

    static constexpr float PACKET_LOSS_WARNING_THRESHOLD = 0.1f;
    static constexpr uint8_t RECENT_PACKET_LOSS_PERIOD   = 100;
};
