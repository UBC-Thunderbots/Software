#include "software/ai/hl/stp/play/enemy_free_kick/enemy_free_kick_play_fsm.h"

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/ai/evaluation/defender_assignment.h"
#include "software/ai/evaluation/enemy_threat.h"
#include "software/ai/hl/stp/tactic/crease_defender/crease_defender_tactic.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/ai/hl/stp/tactic/pass_defender/pass_defender_tactic.h"
#include "software/geom/algorithms/distance.h"
#include "software/util/generic_factory/generic_factory.h"

EnemyFreeKickPlayFSM::EnemyFreeKickPlayFSM(TbotsProto::AiConfig ai_config)
    : ai_config(ai_config), crease_defenders({}), pass_defenders({})
{
}

void EnemyFreeKickPlayFSM::setupEnemyKickerStrategy(const Update& event)
{
    setTactics(event, event.common.num_tactics);
}

void EnemyFreeKickPlayFSM::setTactics(const Update& event, unsigned int num_tactics)
{
    // One tactic is always designated as the free kick defender
    unsigned int num_defenders             = num_tactics - 1;
    PriorityTacticVector tactics_to_return = {{}, {}, {}};
    Point block_kick_point;

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

    // Adds designated free kick defender to block the direction the kicker is facing
    if (!enemy_threats.empty())
    {
        auto block_free_kicker = std::make_shared<PassDefenderTactic>();
        Vector block_direction =
            Vector::createFromAngle(enemy_threats[0].robot.orientation());
        block_kick_point = event.common.world_ptr->ball().position() +
                           block_direction.normalize(
                                   STOP_COMMAND_BALL_AVOIDANCE_DISTANCE_M +
                                   2 * ROBOT_MAX_RADIUS_METERS
                                   );
        block_free_kicker->updateControlParams(block_kick_point);
        tactics_to_return[0].push_back(block_free_kicker);
    }

    // Choose which defender assignments to assign defenders to based on number
    // of tactics available to set after assigning kick off defender
    std::vector<DefenderAssignment> crease_defender_assignments;
    std::vector<DefenderAssignment> pass_defender_assignments;
    std::queue<DefenderAssignment> assignments_skipped;
    for (unsigned int i = 0; i < num_defenders; i++)
    {
        DefenderAssignment defender_assignment;
        if (i + assignments_skipped.size() < assignments.size() - 1)
        {
            defender_assignment = assignments.at(i + assignments_skipped.size());

            // Continuously skips pass defenders within half meter of free kick defender
            while (defender_assignment.type == PASS_DEFENDER &&
                   (i + assignments_skipped.size() < assignments.size() - 1) &&
                   distance(defender_assignment.target, block_kick_point) <=
                       HALF_METER_DISTANCE)
            {
                assignments_skipped.push(defender_assignment);
                defender_assignment = assignments.at(i + assignments_skipped.size());
            }
        }
        else
        {
            // If we have more tactics to set than determined defender assignments:
            // first assign remaining defenders to the previously skipped defender
            // assignments otherwise, assign a defender to stand between the kicker and
            // net if spacing permits else default to defender with the highest coverage
            // rating
            if (!assignments_skipped.empty())
            {
                defender_assignment = assignments_skipped.front();
                assignments_skipped.pop();
            }
            else if (tactics_to_return[0].size() < 2 &&
                     distance(event.common.world_ptr->field().friendlyGoalCenter(),
                              block_kick_point) >=
                         event.common.world_ptr->field().totalYLength() / 2)
            {
                auto mid_zone_defender = std::make_shared<PassDefenderTactic>();
                Point mid_point        = Point(
                    (event.common.world_ptr->ball().position().toVector() +
                     event.common.world_ptr->field().friendlyGoalCenter().toVector()) /
                    2);
                mid_zone_defender->updateControlParams(mid_point);
                tactics_to_return[0].push_back(mid_zone_defender);
            }
            else
            {
                defender_assignment = assignments.front();
            }
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
