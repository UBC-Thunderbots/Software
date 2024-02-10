#include "software/ai/evaluation/scoring/skills/skill_graph.h"

#include "software/util/generic_factory/generic_factory.h"

SkillGraph::SkillGraph(std::shared_ptr<Strategy> strategy)
{
    auto all_skill_constructors =
        GenericFactory<std::string, Skill,
                       std::shared_ptr<Strategy>>::getRegisteredConstructors();

    CHECK(!all_skill_constructors.empty())
        << "No Skills registered in the Skill factory!";

    std::transform(all_skill_constructors.begin(), all_skill_constructors.end(),
                   std::back_inserter(nodes_), [&](auto skill_constructor) {
                       return std::move(skill_constructor(strategy));
                   });

    // The size of the adjacency matrix must be one more than the size of nodes_
    // since there is a "start" node not explicitly represented by any Skill
    // that needs to be accounted for
    adj_matrix_ = std::vector<std::vector<double>>(
        nodes_.size() + 1, std::vector<double>(nodes_.size() + 1, DEFAULT_EDGE_WEIGHT));

    // The first node of the sequence should always be the "start" node, which has
    // an ID of nodes_.size()
    sequence_ = {std::make_pair(nodes_.size(), nodes_.size())};
}

std::shared_ptr<Skill> SkillGraph::getNextSkill(const Robot& robot, const World& world)
{
    unsigned int last_node_id = sequence_.back().second;

    unsigned int best_next_node_id = 0;
    double best_transition_score   = std::numeric_limits<double>::min();

    for (unsigned int node_id = 0; node_id < nodes_.size(); ++node_id)
    {
        double viability_score = nodes_[node_id]->getViability(robot, world);

        // A skill with a viability score of 0 is considered inviable
        // and cannot be selected for execution
        if (viability_score == 0)
        {
            continue;
        }

        double edge_weight      = adj_matrix_[last_node_id][node_id];
        double transition_score = edge_weight + viability_score;

        if (transition_score > best_transition_score)
        {
            best_next_node_id     = node_id;
            best_transition_score = transition_score;
        }
    }

    return nodes_[best_next_node_id];
}

void SkillGraph::extendSequence(const std::shared_ptr<Skill>& skill)
{
    unsigned int last_node_id = sequence_.back().second;
    unsigned int next_node_id = static_cast<unsigned int>(
        std::distance(nodes_.begin(), std::find(nodes_.begin(), nodes_.end(), skill)));

    sequence_.emplace_back(last_node_id, next_node_id);
}

void SkillGraph::scoreSequence(double sequence_score)
{
    LOG_IF(WARNING, sequence_score < -1 || sequence_score > 1)
        << "Skill sequence being scored with score outside of range [-1, 1]";

    // Equation used to calculate the edge weight adjustment:
    // https://www.desmos.com/3d/3d2207d9d3

    double adjustment_sigmoid =
        sigmoid(sequence_score, 0, ADJUSTMENT_SIGMOID_WIDTH) - 0.5;

    double sequence_length = static_cast<double>(sequence_.size() - 1);

    while (sequence_.size() > 1)
    {
        auto [from, to] = sequence_.back();
        sequence_.pop_back();

        double weight = adj_matrix_[from][to];
        double adjustment_resistance =
            sequence_score * weight * weight / ADJUSTMENT_RESISTANCE;
        double adjustment =
            ADJUSTMENT_SCALE * (adjustment_sigmoid - adjustment_resistance);

        // Skills executed near the end of the sequence are considered as
        // having the most impact on the result of the play, so the weight adjustment
        // for edges traversed earlier in the sequence should be less substantial
        // than for those traversed later
        //
        // We multiply the adjustment by a "recency factor" so that edges earlier in
        // the sequence are adjusted by only a portion of the original adjustment
        double recency_factor = static_cast<double>(sequence_.size()) / sequence_length;
        adjustment *= recency_factor;

        double new_weight = std::clamp(weight + adjustment, -MAX_EDGE_WEIGHT_MAGNITUDE,
                                       MAX_EDGE_WEIGHT_MAGNITUDE);

        adj_matrix_[from][to] = new_weight;
    }
}
