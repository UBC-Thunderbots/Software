#pragma once

#include <unordered_set>
#include <deque>

#include "software/ai/navigator/obstacle/obstacle.hpp"

class VisProtoDeduper {
public:
    /**
    * Creates a Visualization Proto Sliding Window Deduplicator
    * 
    * @param window_size size of the sliding window
    */
    VisProtoDeduper(unsigned int window_size);

    void dedupeAndFill(const std::vector<ObstaclePtr>& obstacle_list, TbotsProto::ObstacleList& obstacle_list_out);

    std::size_t hash(const TbotsProto::Obstacle& obstacle) const;

private:
    unsigned int window_size;
    std::unordered_set<std::size_t> sent_set;
    std::deque<std::vector<std::size_t>> sent_queue;

    void hashCombine(std::size_t& seed, std::size_t value) const;
};

