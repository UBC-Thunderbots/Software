#pragma once

#include "software/ai/hl/stp/skill/skill.h"

/**
 * SkillGraph is used to produce sequences of Skills to execute based on the past
 * success of sequences generated.
 *
 * It is a weighted directed graph containing a node for every Skill, with each node
 * connected to every other node. Edge weights represent the success of a transition
 * from one Skill to the next.
 *
 * The SkillGraph records the current sequence of Skills as a walk through the graph,
 * and it extends the sequence by picking the most successful and viable Skill to
 * transition to from the last executed Skill in the sequence. The current sequence
 * can then be completed and scored based on the success of the Skills executed;
 * this will adjust the weights of the edges traversed over in the sequence, enabling
 * the SkillGraph to learn and remember which Skill transitions worked well or not.
 */
class SkillGraph
{
   public:
    explicit SkillGraph(std::shared_ptr<Strategy> strategy);

    /**
     * Returns the next Skill to execute in the current Skill sequence for a given robot.
     *
     * @note This method does not extend the current sequence with the returned Skill;
     * extendSequence must be called with the returned Skill to commit it to the sequence.
     *
     * @param robot the robot that will execute the returned Skill
     * @param world the World
     *
     * @return the next Skill in the current sequence for the given robot to execute
     */
    std::shared_ptr<Skill> getNextSkill(const Robot& robot, const World& world);

    /**
     * Extends the current Skill sequence with the given Skill.
     *
     * @param skill the skill to extend the current sequence with
     */
    void extendSequence(const std::shared_ptr<Skill>& skill);

    /**
     * Completes and scores the current Skill sequence.
     * A new sequence will be started after calling this method.
     *
     * @param sequence_score score between [-1, 1] rating the success of the sequence
     */
    void scoreSequence(double sequence_score);

   private:
    static constexpr double DEFAULT_EDGE_WEIGHT       = 1.0;
    static constexpr double MAX_EDGE_WEIGHT_MAGNITUDE = 5.0;

    // Constants used in calculating edge weight adjustments when
    // scoring a skill sequence
    static constexpr double ADJUSTMENT_SIGMOID_WIDTH = 2.0;
    static constexpr double ADJUSTMENT_RESISTANCE    = 20.0;
    static constexpr double ADJUSTMENT_SCALE         = 0.5;

    /**
     * Each node in the graph is represented by a Skill.
     * The ID of a node is the index of the corresponding Skill in nodes_.
     */
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
     * The first element of the sequence must always be {nodes_.size(),
     * nodes_.size()}. nodes_.size() represents the ID of a "start" node.
     */
    std::vector<std::pair<unsigned int, unsigned int>> sequence_;
};
