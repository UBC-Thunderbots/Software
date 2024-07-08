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
    : DefensePlayFSMBase::DefensePlayFSMBase(ai_config)
{
}

void EnemyFreeKickPlayFSM::blockEnemyKicker(const Update& event)
{
    setTactics(event, event.common.num_tactics);
}

void EnemyFreeKickPlayFSM::setTactics(const Update& event, unsigned int num_tactics)
{
    if (num_tactics <= 0)
    {
        return;
    }
    const Point friendly_goal_center =
        event.common.world_ptr->field().friendlyGoalCenter();
    const double ball_far_threshold_m =
        event.common.world_ptr->field().totalYLength() / 2;


    // One tactic is always designated as the free kick defender
    int num_defenders                      = num_tactics - 1;
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
        auto block_free_kicker = std::make_shared<PassDefenderTactic>(ai_config);
        Vector block_direction =
            Vector::createFromAngle(enemy_threats[0].robot.orientation());
        block_kick_point =
            event.common.world_ptr->ball().position() +
            block_direction.normalize(STOP_COMMAND_BALL_AVOIDANCE_DISTANCE_M +
                                      2 * ROBOT_MAX_RADIUS_METERS);
        block_free_kicker->updateControlParams(block_kick_point,  TbotsProto::BallStealMode::IGNORE);
        tactics_to_return[0].push_back(block_free_kicker);
    }

    // Choose which defender assignments to assign defenders to based on number
    // of tactics available to set after assigning kick off defender
    std::vector<DefenderAssignment> crease_defender_assignments;
    std::vector<DefenderAssignment> pass_defender_assignments;
    std::queue<DefenderAssignment> assignments_skipped;
    for (int i = 0; i < num_defenders; i++)
    {
        DefenderAssignment defender_assignment;
        int assignment_index_with_skipped =
            static_cast<int>(i + assignments_skipped.size());
        int assignment_index_last_assignment = static_cast<int>(assignments.size()) - 1;

        if (assignment_index_with_skipped < assignment_index_last_assignment)
        {
            defender_assignment = assignments.at(assignment_index_with_skipped);

            // Continuously skips pass defenders within threshold meters of free kick
            // defender
            while (defender_assignment.type == PASS_DEFENDER &&
                   (assignment_index_with_skipped < assignment_index_last_assignment) &&
                   distance(defender_assignment.target, block_kick_point) <=
                       TOO_CLOSE_THRESHOLD_METERS)
            {
                assignments_skipped.push(defender_assignment);
                assignment_index_with_skipped =
                    static_cast<int>(i + assignments_skipped.size());
                defender_assignment = assignments.at(assignment_index_with_skipped);
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
                     distance(friendly_goal_center, block_kick_point) >=
                         ball_far_threshold_m)
            {
                auto mid_zone_defender = std::make_shared<PassDefenderTactic>(ai_config);
                Point mid_point =
                    Point((event.common.world_ptr->ball().position().toVector() +
                           friendly_goal_center.toVector()) /
                          2);
                mid_zone_defender->updateControlParams(mid_point,  TbotsProto::BallStealMode::IGNORE);
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
    setAlignment(event, crease_defender_assignments);
    updatePassDefenderControlParams(pass_defender_assignments, TbotsProto::BallStealMode::IGNORE);

    tactics_to_return[1].insert(tactics_to_return[1].end(), crease_defenders.begin(),
                                crease_defenders.end());
    tactics_to_return[2].insert(tactics_to_return[2].end(), pass_defenders.begin(),
                                pass_defenders.end());
    event.common.set_tactics(tactics_to_return);
}
