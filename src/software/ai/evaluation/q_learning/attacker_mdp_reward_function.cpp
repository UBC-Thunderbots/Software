#include "software/ai/evaluation/q_learning/attacker_mdp_reward_function.h"

#include "software/geom/algorithms/contains.h"
#include "software/logger/logger.h"
#include "software/math/math_functions.h"

AttackerMdpRewardFunction::AttackerMdpRewardFunction(std::shared_ptr<Strategy> strategy)
    : strategy_(strategy)
{
}

void AttackerMdpRewardFunction::startStepObservation(WorldPtr world_ptr)
{
    step_start_world_ptr_ = std::move(world_ptr);
}

double AttackerMdpRewardFunction::endStepObservation(WorldPtr world_ptr)
{
    CHECK(step_start_world_ptr_ != nullptr)
        << "Tried to end step observation for AttackerMdpRewardFunction, "
        << "but no step observation was started";

    const TbotsProto::AttackerRewardFunctionConfig& reward_function_config =
        strategy_->getAiConfig().attacker_tactic_config().reward_function_config();

    double reward = 0;

    // Reward friendly team scoring
    if (contains(world_ptr->field().enemyGoal(), world_ptr->ball().position()))
    {
        reward += reward_function_config.friendly_team_scoring_reward();
    }

    // Penalize enemy team scoring
    if (contains(world_ptr->field().friendlyGoal(), world_ptr->ball().position()))
    {
        reward += reward_function_config.enemy_team_scoring_penalty();
    }

    // Reward getting the ball near the enemy goal (i.e. in the enemy defense area)
    // This accounts for missed shots, saves, and close calls
    if (contains(world_ptr->field().enemyDefenseArea(), world_ptr->ball().position()))
    {
        reward += reward_function_config.ball_near_enemy_goal_reward();
    }

    // Reward keeping friendly possession
    if (world_ptr->getTeamWithPossession() == TeamPossession::FRIENDLY)
    {
        reward += reward_function_config.kept_possession_reward();
    }

    // Penalize losing friendly possession
    if (step_start_world_ptr_->getTeamWithPossession() == TeamPossession::FRIENDLY &&
        world_ptr->getTeamWithPossession() != TeamPossession::FRIENDLY)
    {
        reward += reward_function_config.lost_possession_penalty();
    }

    // Reward moving the ball up the field
    Point ball_prev_pos = step_start_world_ptr_->ball().position();
    Point ball_curr_pos = world_ptr->ball().position();
    reward += normalizeValueToRange(std::max(ball_curr_pos.x() - ball_prev_pos.x(), 0.0),
                                    0.0, world_ptr->field().xLength(), 0.0,
                                    reward_function_config.forward_progress_reward());

    // TODO (#3247): Reward enemy team fouls, penalize friendly team fouls

    return reward;
}
