#include "software/ai/hl/stp/play/defense/defense_play_fsm.h"

#include "software/ai/evaluation/defender_assignment.h"
#include "software/ai/evaluation/enemy_threat.h"
#include "software/logger/logger.h"


DefensePlayFSM::DefensePlayFSM(std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr)
    : DefensePlayFSMBase::DefensePlayFSMBase(ai_config_ptr)
{
}

bool DefensePlayFSM::shouldDefendAggressively(const Update& event)
{
    // If there is more attackers ahead of the ball than there
    // are our own defenders we won't press

    auto attackers = std::count_if(
        event.common.world_ptr->enemyTeam().getAllRobots().begin(),
        event.common.world_ptr->enemyTeam().getAllRobots().end(),
        [event](const Robot& robot)
        { return robot.position().x() < event.common.world_ptr->ball().position().x(); });

    auto defenders = std::count_if(
        event.common.world_ptr->friendlyTeam().getAllRobots().begin(),
        event.common.world_ptr->friendlyTeam().getAllRobots().end(),
        [event](const Robot& robot)
        { return robot.position().x() < event.common.world_ptr->ball().position().x(); });

    return defenders > attackers;
}

std::vector<EnemyThreat> DefensePlayFSM::getStableEnemyThreats(const Update& event)
{
    const auto& assignment_config =
        ai_config_ptr->defense_play_config().defender_assignment_config();
    auto enemy_threats = getAllEnemyThreats(
        event.common.world_ptr->field(), event.common.world_ptr->friendlyTeam(),
        event.common.world_ptr->enemyTeam(), event.common.world_ptr->ball(), false,
        previous_baller_id,
        Duration::fromSeconds(
            assignment_config.effective_possession_switch_margin_seconds()),
        &previous_baller_id);

    stabilizePrimaryThreat(
        enemy_threats, previous_primary_threat_id,
        Angle::fromDegrees(ai_config_ptr->defense_play_config()
                               .defender_assignment_config()
                               .primary_threat_hysteresis_degrees()));
    holdPrimaryThreat(enemy_threats, previous_primary_threat_id, primary_hold,
                      assignment_config.primary_threat_hold_updates());
    previous_primary_threat_id =
        enemy_threats.empty() ? std::nullopt
                              : std::make_optional(enemy_threats.front().robot.id());

    return enemy_threats;
}

void DefensePlayFSM::blockShots(const Update& event)
{
    auto enemy_threats = getStableEnemyThreats(event);

    updateCreaseAndPassDefenders(event, enemy_threats);

    setTactics(event);
}

void DefensePlayFSM::shadowAndBlockShots(const Update& event)
{
    auto enemy_threats = getStableEnemyThreats(event);

    updateCreaseAndPassDefenders(event, enemy_threats, true);

    setTactics(event);
}

void DefensePlayFSM::updateCreaseAndPassDefenders(
    const Update& event, const std::vector<EnemyThreat>& enemy_threats,
    bool shadow_primary_threat)
{
    auto defender_assignment_config =
        ai_config_ptr->defense_play_config().defender_assignment_config();
    auto assignments = getAllDefenderAssignments(
        enemy_threats, event.common.world_ptr->field(), event.common.world_ptr->ball(),
        defender_assignment_config);
    applyAssignmentHysteresis(
        assignments, previous_chosen_assignments,
        defender_assignment_config.assignment_hysteresis_multiplier(),
        ASSIGNMENT_MATCH_RADIUS_METERS);
    if (assignments.size() == 0)
    {
        previous_chosen_assignments.clear();
        setUpCreaseDefenders(0);
        setUpPassDefenders(0);
        updateShadowers(event, {});
        return;
    }

    unsigned int max_num_crease_defenders =
        defender_assignment_config.max_num_crease_defenders();

    // Choose which defender assignments to assign defenders to based on number
    // of tactics available to set
    std::vector<DefenderAssignment> crease_defender_assignments;
    std::vector<DefenderAssignment> pass_defender_assignments;
    const auto best_pass = std::find_if(assignments.begin(), assignments.end(),
                                        [](const auto& assignment)
                                        { return assignment.type == PASS_DEFENDER; });
    unsigned int used    = 0;
    for (const auto& assignment : assignments)
    {
        if (used >= event.common.num_tactics)
            break;
        if (assignment.type == CREASE_DEFENDER)
        {
            if (max_num_crease_defenders == 0)
                continue;
            crease_defender_assignments.push_back(assignment);
            --max_num_crease_defenders;
            ++used;
            // Double coverage for the first assignment, within the cap and budget.
            if (used == 1 && used < event.common.num_tactics &&
                max_num_crease_defenders > 0)
            {
                crease_defender_assignments.push_back(assignment);
                --max_num_crease_defenders;
                ++used;
            }
        }
        else
        {
            pass_defender_assignments.push_back(assignment);
            ++used;
        }
    }
    // Surplus robots can only repeat a genuine pass-blocking target.
    while (used < event.common.num_tactics && best_pass != assignments.end())
    {
        pass_defender_assignments.push_back(*best_pass);
        ++used;
    }

    // Allocate the shadow slot before setting up tactics so unchanged updates retain
    // their pass-defender FSMs instead of rebuilding them on every tick.
    if (shadow_primary_threat && !pass_defender_assignments.empty())
    {
        pass_defender_assignments.erase(pass_defender_assignments.begin());
        updateShadowers(event, {enemy_threats.front()});
    }
    else
    {
        updateShadowers(event, {});
    }

    // Reset tactics if the number of crease defenders or pass defenders
    // we intend to assign has changed
    // Remember what we chose so the next update prefers the same assignments
    previous_chosen_assignments = crease_defender_assignments;
    previous_chosen_assignments.insert(previous_chosen_assignments.end(),
                                       pass_defender_assignments.begin(),
                                       pass_defender_assignments.end());

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


void DefensePlayFSM::setUpShadowers(int num_shadowers)
{
    if (num_shadowers == static_cast<int>(shadowers.size()))
    {
        return;
    }

    shadowers = std::vector<std::shared_ptr<ShadowEnemyTactic>>(num_shadowers);
    std::generate(shadowers.begin(), shadowers.end(),
                  [this]()
                  { return std::make_shared<ShadowEnemyTactic>(ai_config_ptr); });
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
