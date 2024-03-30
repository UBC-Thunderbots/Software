#include "software/ai/hl/stp/play/enemy_free_kick/enemy_free_kick_play_fsm.h"

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/util/generic_factory/generic_factory.h"
#include "software/world/game_state.h"
#include "software/ai/evaluation/defender_assignment.h"
#include "software/ai/evaluation/enemy_threat.h"
#include "software/ai/hl/stp/tactic/crease_defender/crease_defender_tactic.h"
#include "software/ai/hl/stp/tactic/pass_defender/pass_defender_tactic.h"
#include "software/geom/algorithms/distance.h"

EnemyFreeKickPlayFSM::EnemyFreeKickPlayFSM(TbotsProto::AiConfig ai_config)
: ai_config(ai_config),
  enemy_free_kick_defenders({}),
  crease_defenders({}),
  pass_defenders({})
{
}

void EnemyFreeKickPlayFSM::setupEnemyKickerStrategy(const Update& event)
{
    unsigned int num_shadow_robots = 1;
    setTactics(event,
               num_shadow_robots,
               event.common.num_tactics - num_shadow_robots);
}

void EnemyFreeKickPlayFSM::setTactics(const Update& event, int num_shadow_robots,
                                              unsigned int num_defenders)
{
    PriorityTacticVector tactics_to_return = {{}, {}, {}};
    Point block_kick_point;

    /**
     * Stopgap redesign of DefensePlayFSM to suit the case of EnemyFreeKickPlayFSM.
     * This MUST be refactored along with DefensePlayFSM in a new PR
     *
     * Otherwise same behaviour as DefensePlayFSM, but:
     * 1) Standard 0.5m obstacle radius around ball per Free Kick rules,
     *      this implementation uses 0.6m to better avoid violation area
     * 2) Only 1 pass defender allowed to guard enemy kicker (Main Pass Defender)
     * 3) Skips additional pass defenders X meters around previously stated free kick pass defender,
     *      this is set to a radius of 3m of the Main Pass Defender.
     *
     * NOTE: First pass defender is NOT SKIPPED, as I have elected to instead just skip over all redundant
     *          pass defenders X meters around the main pass defender
     *
     * KNOWN BUGS:
     * - If the Main Pass Defender switches roles, it may cut through the foul region,
     *      this will be most likely fixed in the new trajectory planner PR
     */
    auto enemy_threats = getAllEnemyThreats(
            event.common.world_ptr->field(), event.common.world_ptr->friendlyTeam(),
            event.common.world_ptr->enemyTeam(), event.common.world_ptr->ball(), false);

    auto assignments = getAllDefenderAssignments(
            enemy_threats, event.common.world_ptr->field(), event.common.world_ptr->ball(),
            ai_config.defense_play_config().defender_assignment_config());

    if (assignments.size() == 0)
    {
        return;
    }

    if (num_shadow_robots > 0 && !enemy_threats.empty())
    {
        enemy_free_kick_defenders = std::vector<std::shared_ptr<PassDefenderTactic>>(num_shadow_robots);
        std::generate(enemy_free_kick_defenders.begin(), enemy_free_kick_defenders.end(),
                      [&block_kick_point, &event, &enemy_threats]() {
                          auto block_free_kicker = std::make_shared<PassDefenderTactic>();
                          Vector block_direction = Vector::createFromAngle(enemy_threats[0].robot.orientation());
                          block_kick_point = event.common.world_ptr->ball().position()
                                  + block_direction.normalize(0.6 + 2 * ROBOT_MAX_RADIUS_METERS);
                          block_free_kicker->updateControlParams(block_kick_point);
                          return block_free_kicker;
                      });
        tactics_to_return[0].insert(tactics_to_return[0].end(),
                                    enemy_free_kick_defenders.begin(),
                                    enemy_free_kick_defenders.end());
    }

    // Choose which defender assignments to assign defenders to based on number
    // of tactics available to set after assigning kick off defender
    std::vector<DefenderAssignment> crease_defender_assignments;
    std::vector<DefenderAssignment> pass_defender_assignments;
    unsigned int assigns_skipped = 0;
    for (unsigned int i = 0; i < num_defenders; i++)
    {
        DefenderAssignment defender_assignment;
        if (i < assignments.size())
        {
            defender_assignment = assignments.at(i+assigns_skipped);
            defender_assignment = assignments.at(i);

            while (defender_assignment.type == PASS_DEFENDER
            && distance(defender_assignment.target, block_kick_point) <= 0.5)
            {
                if (i+assigns_skipped+1 >= assignments.size()) {
                    defender_assignment = assignments.front();
                    break;
                }
                assigns_skipped++;
                defender_assignment = assignments.at(i+assigns_skipped);
            }

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
            if (i == 0 && num_defenders >= 2)
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

    for (unsigned int i = 0; i < crease_defenders.size(); i++)
    {
        auto target = crease_defender_assignments.at(i).target;

        // Determine the number of crease defenders already assigned to the target
        auto defenders_with_target_count = std::count_if(
                crease_defender_assignments.begin(), crease_defender_assignments.begin() + i,
                [&target](const auto& assignment) { return assignment.target == target; });

        // Pick alignment based on how many crease defenders are already assigned to the
        // target
        auto alignment = TbotsProto::CreaseDefenderAlignment::CENTRE;
        if (defenders_with_target_count == 1)
        {
            if (event.common.world_ptr->ball().position().y() > 0)
            {
                alignment = TbotsProto::CreaseDefenderAlignment::LEFT;
            }
            else
            {
                alignment = TbotsProto::CreaseDefenderAlignment::RIGHT;
            }
        }
        else if (defenders_with_target_count == 2)
        {
            if (event.common.world_ptr->ball().position().y() > 0)
            {
                alignment = TbotsProto::CreaseDefenderAlignment::RIGHT;
            }
            else
            {
                alignment = TbotsProto::CreaseDefenderAlignment::LEFT;
            }
        }

        crease_defenders.at(i)->updateControlParams(
                target, alignment, event.control_params.max_allowed_speed_mode);
    }

    for (unsigned int i = 0; i < pass_defenders.size(); i++)
    {
        pass_defenders.at(i)->updateControlParams(pass_defender_assignments.at(i).target);
    }


    tactics_to_return[1].insert(tactics_to_return[1].end(), crease_defenders.begin(),
                                crease_defenders.end());
    tactics_to_return[2].insert(tactics_to_return[2].end(), pass_defenders.begin(),
                                pass_defenders.end());
    event.common.set_tactics(tactics_to_return);

}

void EnemyFreeKickPlayFSM::setUpCreaseDefenders(unsigned int num_crease_defenders)
{
    if (num_crease_defenders == crease_defenders.size())
    {
        return;
    }

    crease_defenders =
            std::vector<std::shared_ptr<CreaseDefenderTactic>>(num_crease_defenders);
    std::generate(crease_defenders.begin(), crease_defenders.end(), [this]() {
        return std::make_shared<CreaseDefenderTactic>(
                ai_config.robot_navigation_obstacle_config());
    });
}

void EnemyFreeKickPlayFSM::setUpPassDefenders(unsigned int num_pass_defenders)
{
    if (num_pass_defenders == pass_defenders.size())
    {
        return;
    }

    pass_defenders = std::vector<std::shared_ptr<PassDefenderTactic>>(num_pass_defenders);
    std::generate(pass_defenders.begin(), pass_defenders.end(),
                  [this]() { return std::make_shared<PassDefenderTactic>(); });
}