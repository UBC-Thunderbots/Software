#pragma once

#include <unordered_set>
#include <deque>

#include "software/ai/navigator/obstacle/obstacle.hpp"

/**
 * The VisProtoDeduper maintains a rolling history of obstacles that have already been transmitted. 
 * By using a combination of a sliding window (deque) and a fast lookup (hash set), 
 * it ensures that only "new" or "expired" information is added to the outgoing protobuf message.
 *
 *  For example:
 *  TIME STEP [t]                                     INTERNAL STATE
 *  -------------------------------------------       ------------------------------
 *  Incoming Obstacles List: [ A, B, C ]              sent_set: { A, B, C }
 *  Action: All are NEW.                              sent_queue: [ {A,B,C} ]
 *  Output Proto: { A, B, C }
 *
 *  TIME STEP [t+1]
 *  -------------------------------------------       sent_set: { A, B, C, D }
 *  Incoming ObstaclesList: [ A, D ]                  sent_queue: [ {A,B,C}, {D} ]
 *  Action: A is DUPE, D is NEW.
 *  Output Proto: { D }
 *
 *  TIME STEP [t+2] (Window Size = 2)
 *  -------------------------------------------       sent_set: { D, E }
 *  Incoming ObstaclesList: [ A, E ]                  sent_queue: [ {D}, {E} ]
 *  Action: A was EVICTED from window,                ( {A,B,C} was popped )
 *       so A is NEW again. E is NEW.
 *  Output Proto: { A, E }
 */
class VisProtoDeduper {
public:
    /**
    * Creates a sliding window deduplicater
    * 
    * @param window_size size of the sliding window
    */
    VisProtoDeduper(unsigned int window_size);

    /**
    * Given an input obstacle list 
    * 
    * @param obstacle_list input list of obstacle
    * @param obstacle_list_out output list of obstacle after filtered
    */
    void dedupeAndFill(const std::vector<ObstaclePtr>& obstacle_list, TbotsProto::ObstacleList& obstacle_list_out);


private:
    unsigned int window_size_;
    std::unordered_set<std::size_t> sent_set_;
    std::deque<std::vector<std::size_t>> sent_queue_;

    std::hash<Obstacle> obstacle_hasher_;
};

