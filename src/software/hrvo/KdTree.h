#pragma once

#include <cstddef>
#include <vector>

#include "Vector2.h"

class Agent;
class Simulator;

/**
 * \class  KdTree
 * \brief  k-D trees for agents in the simulation.
 */
class KdTree {
private:
    /**
     * \class  Node
     * \brief  Defines a k-D tree node.
     */
    class Node {
    public:
        /**
         * \brief  Constructor.
         */
        Node() : begin_(0), end_(0), left_(0), right_(0), maxX_(0.0f), maxY_(0.0f), minX_(0.0f), minY_(0.0f) { }

        /**
         * \brief  The beginning node number.
         */
        std::size_t begin_;

        /**
         * \brief  The ending node number.
         */
        std::size_t end_;

        /**
         * \brief  The left node number.
         */
        std::size_t left_;

        /**
         * \brief  The right node number.
         */
        std::size_t right_;

        /**
         * \brief  The maximum x-coordinate.
         */
        float maxX_;

        /**
         * \brief  The maximum y-coordinate.
         */
        float maxY_;

        /**
         * \brief  The minimum x-coordinate.
         */
        float minX_;

        /**
         * \brief  The minimum y-coordinate.
         */
        float minY_;
    };

    /**
     * \brief  The maximum leaf size of a k-D tree.
     */
    static const std::size_t HRVO_MAX_LEAF_SIZE = 10;

    /**
     * \brief      Constructor.
     * \param[in]  simulator  The simulation.
     */
    explicit KdTree(Simulator *simulator);

    /**
     * \brief  Builds an agent k-D tree.
     */
    void build();

    /**
     * \brief  Recursive function to build a k-D tree.
     * \param  begin  The beginning k-D tree node.
     * \param  end    The ending k-D tree node.
     * \param  node   The current k-D tree node.
     */
    void buildRecursive(std::size_t begin, std::size_t end, std::size_t node);

    /**
     * \brief      Computes the neighbors of the specified agent.
     * \param[in]  agent    A pointer to the agent for which neighbors are to be computed.
     * \param[in]  rangeSq  The squared range around the agent.
     */
    void query(Agent *agent, float rangeSq) const
    {
        queryRecursive(agent, rangeSq, 0);
    }

    /**
     * \brief          Recursive function to compute the neighbors of the specified agent.
     * \param[in]      agent    A pointer to the agent for which neighbors are to be computed.
     * \param[in,out]  rangeSq  The squared range around the agent.
     * \param[in]      node     The current k-D tree node.
     */
    void queryRecursive(Agent *agent, float &rangeSq, std::size_t node) const;

    Simulator *const simulator_;
    std::vector<std::size_t> agents_;
    std::vector<Node> nodes_;

    friend class Agent;
    friend class Simulator;
};

