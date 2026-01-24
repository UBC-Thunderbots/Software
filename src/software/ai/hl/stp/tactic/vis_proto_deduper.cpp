#include "vis_proto_deduper.h"

VisProtoDeduper::VisProtoDeduper(unsigned int window_size):
    window_size(window_size) {}

void VisProtoDeduper::dedupeAndFill(const std::vector<ObstaclePtr> &obstacle_list, TbotsProto::ObstacleList& obstacle_list_out) {
    // Lazily evit the ObstacleList from the deque
    if (sent_queue.size() > window_size) {
        const std::vector<std::size_t>& popped_hashes = sent_queue.front();
        for (const auto &obstacle_hash : popped_hashes) {
            sent_set.erase(obstacle_hash);
        }

        sent_queue.pop_front();
    }

    // Computing hashes of the current obstacle list and compare with the window
    std::vector<std::size_t> current_hashes;
    for (const auto &obstacle : obstacle_list) {
        std::size_t hash_val = obstacle_hasher(*obstacle);
        if (sent_set.count(hash_val) == 0) {
            TbotsProto::Obstacle proto = obstacle->createObstacleProto();
            sent_set.insert(hash_val);
            obstacle_list_out.add_obstacles()->CopyFrom(proto);
            current_hashes.push_back(hash_val);
        }
    }
    sent_queue.push_back(std::move(current_hashes));
}

