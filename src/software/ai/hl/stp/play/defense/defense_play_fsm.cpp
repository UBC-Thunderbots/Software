#include "software/ai/hl/stp/play/defense/defense_play_fsm.h"

#include "software/ai/evaluation/defender_assignment.h"
#include "software/ai/evaluation/enemy_threat.h"

DefensePlayFSM::DefensePlayFSM(std::shared_ptr<Strategy> strategy)
    : DefensePlayFSMBase(strategy),
      shadow_enemy_tactic_(std::make_shared<ShadowEnemyTactic>())
{
}

void DefensePlayFSM::defendAgainstThreats(const Update& event)
{
    auto enemy_threats = getAllEnemyThreats(
        event.common.world_ptr->field(), event.common.world_ptr->friendlyTeam(),
        event.common.world_ptr->enemyTeam(), event.common.world_ptr->ball(), false);

    auto assignments = getAllDefenderAssignments(
        enemy_threats, event.common.world_ptr->field(), event.common.world_ptr->ball(),
        strategy->getAiConfig().defense_play_config().defender_assignment_config());

    if (assignments.size() == 0)
    {
        return;
    }

    PriorityTacticVector tactics_to_return = {{}};
    unsigned int num_tactics               = event.common.num_tactics;

    // Assign a ShadowEnemyTactic to defend the primary enemy threat
    // if we have more than 3 tactics to assign
    if (num_tactics >= 3)
    {
        tactics_to_return.push_back({shadow_enemy_tactic_});
        shadow_enemy_tactic_->updateControlParams(
            enemy_threats.front(),
            strategy->getAiConfig().defense_play_config().shadow_enemy_distance_meters());
        --num_tactics;
    }

    // Choose which defender assignments to assign defenders to based on number
    // of tactics available to set
    std::vector<DefenderAssignment> crease_defender_assignments;
    std::vector<DefenderAssignment> pass_defender_assignments;
    for (unsigned int i = 0; i < num_tactics; i++)
    {
        DefenderAssignment defender_assignment;
        if (i < assignments.size())
        {
            defender_assignment = assignments.at(i);
        }
        else
        {
            // If we have more tactics to set than determined defender assignments,
            // assign remaining defenders to the defender assignment with the
            // highest coverage rating
            defender_assignment = assignments.front();
        }

        if (defender_assignment.type == CREASE_DEFENDER)
        {
            crease_defender_assignments.emplace_back(defender_assignment);

            // If we have at least two available defenders, two defenders should
            // be assigned to the highest scoring crease defender assignment to better
            // block the shot cone of the most threatening enemy
            if (i == 0 && num_tactics >= 2)
            {
                crease_defender_assignments.emplace_back(defender_assignment);
                i++;
            }
        }
        else
        {
            pass_defender_assignments.emplace_back(defender_assignment);
        }
    }

    // Reset tactics if the number of crease defenders or pass defenders
    // we intend to assign has changed
    setUpCreaseDefenders(crease_defender_assignments.size());
    setUpPassDefenders(pass_defender_assignments.size());
    setAlignment(event, crease_defender_assignments, TbotsProto::BallStealMode::STEAL);
    updatePassDefenderControlParams(pass_defender_assignments,
                                    TbotsProto::BallStealMode::STEAL);

    TacticVector crease_defender_tactics;
    TacticVector pass_defender_tactics;
    crease_defender_tactics.insert(crease_defender_tactics.end(),
                                   crease_defenders.begin(), crease_defenders.end());
    pass_defender_tactics.insert(pass_defender_tactics.end(), pass_defenders.begin(),
                                 pass_defenders.end());
    tactics_to_return.push_back(std::move(crease_defender_tactics));
    tactics_to_return.push_back(std::move(pass_defender_tactics));

    event.common.set_tactics(tactics_to_return);
}
