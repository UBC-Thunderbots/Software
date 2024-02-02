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

    adj_matrix_ = std::vector<std::vector<double>>(
        nodes_.size() + 1,
        std::vector<double>(nodes_.size() + 1, DEFAULT_TRANSITION_SCORE));

    sequence_ = {std::make_pair(nodes_.size(), nodes_.size())};
}

std::shared_ptr<Skill> SkillGraph::getNextSkill(const Robot& robot, const World& world)
{
    unsigned int last_node_id = sequence_.back().second;

    unsigned int best_next_node_id;
    double best_transition_score = std::numeric_limits<double>::min();

    for (unsigned int node_id = 0; node_id < nodes_.size(); ++node_id)
    {
        double transition_score = adj_matrix_[last_node_id][node_id];
        transition_score *= nodes_[node_id]->getViability(robot, world);

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

void SkillGraph::completeSequence(double sequence_score)
{
    while (sequence_.size() > 1)
    {
        // TODO: Update sequence edge weights
        sequence_.pop_back();
    }
}
