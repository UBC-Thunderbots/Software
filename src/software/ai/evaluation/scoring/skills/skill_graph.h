#pragma once

#include "software/ai/hl/stp/tactic/attacker/skill/skill.h"

class SkillGraph
{
   public:
    explicit SkillGraph(std::shared_ptr<Strategy> strategy);

    std::shared_ptr<Skill> getNextSkill(const Robot& robot, const World& world);

    void extendSequence(const std::shared_ptr<Skill>& skill);

    void completeSequence(double sequence_score);

   private:
    static constexpr double DEFAULT_TRANSITION_SCORE = 1.0;

    // The nodes in the graph.
    std::vector<std::shared_ptr<Skill>> nodes_;

    /**
     * Adjacency matrix representation of the graph.
     *
     * A non-zero value at adj_matrix_[i][j] indicates a directed edge from i to j,
     * where i and j are the IDs of the connected nodes.
     *
     * The weight of the edge is indicated by the value at adj_matrix_[i][j].
     */
    std::vector<std::vector<double>> adj_matrix_;

    /**
     * Tracks the sequence of skills chosen so far.
     *
     * Each pair {i, j} represents an edge from i to j that was travelled along,
     * where i and j are the IDs of the nodes that the edge connects.
     *
     * The first element of the sequence must always be {`nodes_.size()`,
     * `nodes_.size()`}. `nodes_.size()` represents the ID of a "start" node.
     */
    std::vector<std::pair<unsigned int, unsigned int>> sequence_;
};