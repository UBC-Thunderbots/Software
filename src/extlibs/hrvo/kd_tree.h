/*
 * kd_tree.h
 * Hrvo Library
 *
 * Copyright 2009 University of North Carolina at Chapel Hill
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Please send all bug reports to <geom@cs.unc.edu>.
 *
 * The authors may be contacted via:
 *
 * Jamie Snape, Jur van den Berg, Stephen J. Guy, and Dinesh Manocha
 * Dept. of Computer Science
 * 201 S. Columbia St.
 * Frederick P. Brooks, Jr. Computer Science Bldg.
 * Chapel Hill, N.C. 27599-3175
 * United States of America
 *
 * <https://gamma.cs.unc.edu/HRVO/>
 */

#pragma once

#include <cstddef>
#include <vector>

#include "software/geom/vector.h"

class HrvoAgent;
class HrvoSimulator;

/**
 * k-D trees for agents in the simulation.
 */
class KdTree
{
   private:
    /**
     * Defines a k-D tree node.
     */
    class Node
    {
       public:
        Node()
            : begin_(0),
              end_(0),
              left_(0),
              right_(0),
              maxX_(0.0f),
              maxY_(0.0f),
              minX_(0.0f),
              minY_(0.0f)
        {
        }

        // The beginning node number.
        std::size_t begin_;

        // The ending node number.
        std::size_t end_;

        // The left node number.
        std::size_t left_;

        // The right node number.
        std::size_t right_;

        // The maximum x-coordinate.
        float maxX_;

        // The maximum y-coordinate.
        float maxY_;

        // The minimum x-coordinate.
        float minX_;

        // The minimum y-coordinate.
        float minY_;
    };

    // The maximum leaf size of a k-D tree.
    static const std::size_t HRVO_MAX_LEAF_SIZE = 10;

   public:
    /**
     * Constructor
     *
     * @param  simulator  The simulation.
     */
    explicit KdTree(HrvoSimulator *simulator);

    /**
     * Builds an agent k-D tree.
     */
    void build();

    /**
     * Recursive function to build a k-D tree.
     *
     * @param begin  The beginning k-D tree node.
     * @param end    The ending k-D tree node.
     * @param node   The current k-D tree node.
     */
    void buildRecursive(std::size_t begin, std::size_t end, std::size_t node);

    /**
     * Computes the neighbors of the specified agent.
     *
     * @param agent  A pointer to the agent for which neighbors are to be computed.
     * @param range  The range around the agent.
     */
    void query(HrvoAgent *agent, float range) const
    {
        float range_sq = range * range;
        queryRecursive(agent, range_sq, 0);
    }

    /**
     * Recursive function to compute the neighbors of the specified agent.
     *
     * @param agent    A pointer to the agent for which neighbors are to be computed.
     * @param range_sq  The squared range around the agent.
     * @param node     The current k-D tree node.
     */
    void queryRecursive(HrvoAgent *agent, float &range_sq, std::size_t node) const;

   private:
    HrvoSimulator *const simulator_;
    std::vector<std::size_t> agents_;
    std::vector<Node> nodes_;

    friend class HrvoAgent;
    friend class Simulator;
};
