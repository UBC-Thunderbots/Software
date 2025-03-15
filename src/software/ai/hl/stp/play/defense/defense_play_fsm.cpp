#include "software/ai/hl/stp/play/defense/defense_play_fsm.h"

#include "software/ai/evaluation/defender_assignment.h"
#include "software/ai/evaluation/enemy_threat.h"

#include "software/logger/logger.h"


DefensePlayFSM::DefensePlayFSM(TbotsProto::AiConfig ai_config)
    : DefensePlayFSMBase::DefensePlayFSMBase(ai_config)
{
}

bool DefensePlayFSM::shouldDefendAggressively(const Update& event)
{
   
  return true;
}

void DefensePlayFSM::blockShots(const Update& event)
{
    auto enemy_threats = getAllEnemyThreats(
        event.common.world_ptr->field(), event.common.world_ptr->friendlyTeam(),
        event.common.world_ptr->enemyTeam(), event.common.world_ptr->ball(), false);

    updateCreaseAndPassDefenders(event, enemy_threats);
    updateShadowers(event, {});

    setTactics(event);
}

void DefensePlayFSM::shadowAndBlockShots(const Update& event)
{
    auto enemy_threats = getAllEnemyThreats(
        event.common.world_ptr->field(), event.common.world_ptr->friendlyTeam(),
        event.common.world_ptr->enemyTeam(), event.common.world_ptr->ball(), false);


 updateCreaseAndPassDefenders(event, enemy_threats);

    if (pass_defenders.size() > 0)
    {

        pass_defenders.erase(pass_defenders.begin());
        updateShadowers(event, {enemy_threats.front()});
    }

    setTactics(event);
}

void DefensePlayFSM::updateCreaseAndPassDefenders(
    const Update& event, const std::vector<EnemyThreat>& enemy_threats)
{
    auto assignments = getAllDefenderAssignments(
        enemy_threats, event.common.world_ptr->field(), event.common.world_ptr->ball(),
    
        ai_config.defense_play_config().defender_assignment_config());

    if (assignments.size() == 0)
    {
        return;
    }

    // Choose which defender assignments to assign defenders to based on number
    // of tactics available to set
    std::vector<DefenderAssignment> crease_defender_assignments;
    std::vector<DefenderAssignment> pass_defender_assignments;
    for (unsigned int i = 0; i < event.common.num_tactics; i++)
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
            if (i == 0 && event.common.num_tactics >= 2)
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
    setUpCreaseDefenders(static_cast<unsigned int>(crease_defender_assignments.size()));
    setUpPassDefenders(static_cast<unsigned int>(pass_defender_assignments.size()));
    setAlignment(event, crease_defender_assignments, TbotsProto::BallStealMode::STEAL);
    updatePassDefenderControlParams(pass_defender_assignments,
                                    TbotsProto::BallStealMode::STEAL);

}

void DefensePlayFSM::updateShadowers(const Update& event,
                                     const std::vector<EnemyThreat>& threats_to_shadow)
{
    setUpShadowers(static_cast<unsigned int>(threats_to_shadow.size()));

    for (unsigned int i = 0; i < shadowers.size(); i++)
    {
        shadowers.at(i)->updateControlParams(threats_to_shadow.at(i),
                                             ROBOT_SHADOWING_DISTANCE_METERS);
    }
}


void DefensePlayFSM::setUpShadowers(unsigned int num_shadowers)
{
    if (num_shadowers == shadowers.size())
    {
        return;
    }

    shadowers = std::vector<std::shared_ptr<ShadowEnemyTactic>>(num_shadowers);
    std::generate(shadowers.begin(), shadowers.end(),
                  [this]() { return std::make_shared<ShadowEnemyTactic>(); });
}

void DefensePlayFSM::setTactics(const Update& event)
{
    PriorityTacticVector tactics_to_return = {{}, {}, {}};

    tactics_to_return[0].insert(tactics_to_return[0].end(), crease_defenders.begin(),
                                crease_defenders.end());
    tactics_to_return[1].insert(tactics_to_return[1].end(), pass_defenders.begin(),
                                pass_defenders.end());
    tactics_to_return[2].insert(tactics_to_return[2].end(), shadowers.begin(),
                                shadowers.end());

    event.common.set_tactics(tactics_to_return);
}
