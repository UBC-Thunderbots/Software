#include "software/ai/hl/stp/play/defense/defense_play_fsm.h"

#include "software/ai/evaluation/defender_assignment.h"
#include "software/ai/evaluation/enemy_threat.h"

DefensePlayFSM::DefensePlayFSM(std::shared_ptr<Strategy> strategy)
    : DefensePlayFSMBase(strategy)
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
    PriorityTacticVector tactics_to_return = {{}, {}, {}};
    int num_defenders = event.common.num_tactics;

    // No need to check if we have possession since that would trigger OffensePlay automatically
    if (num_defenders >= 1)
    {
        Point current_ball_position = event.common.world_ptr->ball().position();
        const auto clock_time = std::chrono::system_clock::now();
        if (distance(enemy_possession_ball_position, current_ball_position)
            >= strategy->getAiConfig().defense_play_config().ball_stagnant_distance_threshold_m())
        {
            // Reset the ball's possible stagnant location when it moves out of its threshold radius
            // and reset the initial time for a possible stagnant ball
            enemy_possession_ball_position = current_ball_position;
            enemy_possession_epoch_time_s =
                    static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                            clock_time.time_since_epoch())
                            .count()) *
                    SECONDS_PER_MICROSECOND;
        }
        double epoch_time_in_seconds =
                static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                        clock_time.time_since_epoch())
                        .count()) *
                SECONDS_PER_MICROSECOND;

        double ball_displacement_m =
                distance(event.common.world_ptr->ball().position(), enemy_possession_ball_position);

        if (epoch_time_in_seconds - enemy_possession_epoch_time_s
            >= strategy->getAiConfig().defense_play_config().ball_stagnant_time_threshold_s()
            && ball_displacement_m <= strategy->getAiConfig().defense_play_config().ball_stagnant_distance_threshold_m())
        {
            // Ball is defined as "stagnant" due to no progress
            std::optional<Robot> nearest_enemy_to_ball =
                    event.common.world_ptr->enemyTeam().getNearestRobot(event.common.world_ptr->ball().position());
            Angle stealer_orientation;

            if (nearest_enemy_to_ball.has_value())
            {
                // We want to face opposing the enemy to effectively obtain the ball
                stealer_orientation = nearest_enemy_to_ball.value().orientation() + Angle::half();
            }
            DribbleSkillFSM::ControlParams dribble_control_params{
                    .dribble_destination       = event.common.world_ptr->ball().position(),
                    .final_dribble_orientation = stealer_orientation,
                    .excessive_dribbling_mode  = TbotsProto::ExcessiveDribblingMode::LOSE_BALL
            };
            num_defenders--;
            auto ball_stealer = std::make_shared<AssignedSkillTactic<DribbleSkill>>(strategy);
            ball_stealer->updateControlParams(dribble_control_params);
            tactics_to_return[0].push_back(ball_stealer);
        }
    }

    // Choose which defender assignments to assign defenders to based on number
    // of tactics available to set
    std::vector<DefenderAssignment> crease_defender_assignments;
    std::vector<DefenderAssignment> pass_defender_assignments;
    for (int i = 0; i < num_defenders; i++)
    {
        DefenderAssignment defender_assignment;
        if (i < static_cast<int>(assignments.size()))
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

    tactics_to_return[1].insert(tactics_to_return[1].end(), crease_defenders.begin(),
                                crease_defenders.end());
    tactics_to_return[2].insert(tactics_to_return[2].end(), pass_defenders.begin(),
                                pass_defenders.end());
    event.common.set_tactics(tactics_to_return);
}