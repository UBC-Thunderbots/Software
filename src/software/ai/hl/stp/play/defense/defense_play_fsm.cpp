#include "software/ai/hl/stp/play/defense/defense_play_fsm.h"

#include "software/ai/evaluation/defender_assignment.h"
#include "software/ai/evaluation/enemy_threat.h"

DefensePlayFSM ::DefensePlayFSM(TbotsProto::AiConfig ai_config)
    : ai_config(ai_config), crease_defenders({}), pass_defenders({})
{
}

void DefensePlayFSM::defendAgainstThreats(const Update& event)
{
    auto enemy_threats = getAllEnemyThreats(
        event.common.world.field(), event.common.world.friendlyTeam(),
        event.common.world.enemyTeam(), event.common.world.ball(), false);

    // Remove threats which are not near our side of the field.
    // We define being near our side as being within 3/4 of the field length
    // from our goal line (max_x_coordinate)
    double max_x_coordinate = 3 * event.common.world.field().xLength() / 4;
    enemy_threats.erase(std::remove_if(enemy_threats.begin(), enemy_threats.end(),
                                       [max_x_coordinate](const auto& threat) {
                                           return threat.robot.position().x() >
                                                  max_x_coordinate;
                                       }),
                        enemy_threats.end());

    if (enemy_threats.size() == 0)
    {
        return;
    }

    auto assignments = getAllDefenderAssignments(
        enemy_threats, event.common.world.field(), event.common.world.ball());

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
            // highest "coverage_rating"
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
    auto num_crease_defenders =
        static_cast<unsigned int>(crease_defender_assignments.size());
    auto num_pass_defenders = static_cast<unsigned int>(pass_defender_assignments.size());
    if (num_crease_defenders != crease_defenders.size())
    {
        setUpCreaseDefenders(num_crease_defenders);
    }
    if (num_pass_defenders != pass_defenders.size())
    {
        setUpPassDefenders(num_pass_defenders);
    }

    for (unsigned int i = 0; i < crease_defenders.size(); i++)
    {
        auto target = crease_defender_assignments.at(i).target;

        // Determine the number of crease defenders already assigned to the target
        auto defenders_with_target_count = std::count_if(
            crease_defender_assignments.begin(), crease_defender_assignments.begin() + i,
            [&target](const auto& assignment) { return assignment.target == target; });

        // Pick alignment based on how many crease defenders are already assigned to the
        // target
        TbotsProto::CreaseDefenderAlignment alignment;
        switch (defenders_with_target_count)
        {
            case 0:
                alignment = TbotsProto::CreaseDefenderAlignment::CENTRE;
                break;
            case 1:
                alignment = TbotsProto::CreaseDefenderAlignment::LEFT;
                break;
            case 2:
                alignment = TbotsProto::CreaseDefenderAlignment::RIGHT;
                break;
            default:
                alignment = TbotsProto::CreaseDefenderAlignment::CENTRE;
        }

        crease_defenders.at(i)->updateControlParams(
            target, alignment, event.control_params.max_allowed_speed_mode);
    }

    for (unsigned int i = 0; i < pass_defenders.size(); i++)
    {
        pass_defenders.at(i)->updateControlParams(pass_defender_assignments.at(i).target);
    }

    PriorityTacticVector tactics_to_return = {{}, {}};
    tactics_to_return[0].insert(tactics_to_return[0].end(), crease_defenders.begin(),
                                crease_defenders.end());
    tactics_to_return[1].insert(tactics_to_return[1].end(), pass_defenders.begin(),
                                pass_defenders.end());
    event.common.set_tactics(tactics_to_return);
}

void DefensePlayFSM::setUpCreaseDefenders(unsigned int num_crease_defenders)
{
    crease_defenders =
        std::vector<std::shared_ptr<CreaseDefenderTactic>>(num_crease_defenders);
    std::generate(crease_defenders.begin(), crease_defenders.end(), [this]() {
        return std::make_shared<CreaseDefenderTactic>(
            ai_config.robot_navigation_obstacle_config());
    });
}

void DefensePlayFSM::setUpPassDefenders(unsigned int num_pass_defenders)
{
    pass_defenders = std::vector<std::shared_ptr<PassDefenderTactic>>(num_pass_defenders);
    std::generate(pass_defenders.begin(), pass_defenders.end(),
                  [this]() { return std::make_shared<PassDefenderTactic>(); });
}
