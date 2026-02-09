#include "vis_proto_deduper.h"

VisProtoDeduper::VisProtoDeduper(unsigned int window_size):
    window_size_(window_size) {}

void VisProtoDeduper::dedupeAndFill(const std::vector<ObstaclePtr> &obstacle_list, TbotsProto::ObstacleList& obstacle_list_out) {
    // lazily evict the ObstacleList from the deque
    if (sent_queue_.size() > window_size_) {
        const std::vector<std::size_t>& popped_hashes = sent_queue_.front();
        for (const auto &obstacle_hash : popped_hashes) {
            sent_set_.erase(obstacle_hash);
        }

        sent_queue_.pop_front();
    }

    // computing hashes of the current obstacle list and compare with the window
    std::vector<std::size_t> current_hashes;
    for (const auto &obstacle : obstacle_list) {
        std::size_t hash_val = obstacle_hasher_(*obstacle);
        // only push to the output if this packet has not been seen in the window
        if (sent_set_.count(hash_val) == 0) {
            TbotsProto::Obstacle proto = obstacle->createObstacleProto();
            sent_set_.insert(hash_val);
            obstacle_list_out.add_obstacles()->CopyFrom(proto);
            current_hashes.push_back(hash_val);
        }
    }
    sent_queue_.push_back(std::move(current_hashes));
}

