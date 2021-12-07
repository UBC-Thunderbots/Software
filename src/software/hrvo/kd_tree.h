#pragma once

#include <cstddef>
#include <vector>

#include "vector2.h"

class Agent;
class Simulator;

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

    /**
     * Constructor
     *
     * @param  simulator  The simulation.
     */
    explicit KdTree(Simulator *simulator);

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
     * @param agent    A pointer to the agent for which neighbors are to be computed.
     * @param rangeSq  The squared range around the agent.
     */
    void query(Agent *agent, float rangeSq) const
    {
        queryRecursive(agent, rangeSq, 0);
    }

    /**
     * Recursive function to compute the neighbors of the specified agent.
     *
     * @param agent    A pointer to the agent for which neighbors are to be computed.
     * @param rangeSq  The squared range around the agent.
     * @param node     The current k-D tree node.
     */
    void queryRecursive(Agent *agent, float &rangeSq, std::size_t node) const;

    Simulator *const simulator_;
    std::vector<std::size_t> agents_;
    std::vector<Node> nodes_;

    friend class Agent;
    friend class Simulator;
};
