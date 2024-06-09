#include "software/ai/evaluation/q_learning/attacker_mdp_reward_function.h"

#include "software/geom/algorithms/contains.h"
#include "software/logger/logger.h"
#include "software/math/math_functions.h"

void AttackerMdpRewardFunction::startStepObservation(WorldPtr world_ptr)
{
    step_start_world_ptr_ = std::move(world_ptr);
}

double AttackerMdpRewardFunction::endStepObservation(WorldPtr world_ptr)
{
    CHECK(step_start_world_ptr_ != nullptr)
        << "Tried to end step observation for AttackerMdpRewardFunction, "
        << "but no step observation was started";

    double reward = 0;

    // Reward friendly team scoring
    if (contains(world_ptr->field().enemyGoal(), world_ptr->ball().position()))
    {
        reward += 0.5;
    }

    // Penalize enemy team scoring
    if (contains(world_ptr->field().friendlyGoal(), world_ptr->ball().position()))
    {
        reward -= 0.5;
    }

    // Reward keeping possession
    if (world_ptr->getTeamWithPossession() == TeamPossession::FRIENDLY)
    {
        reward += 0.1;
    }

    // Reward moving the ball up the field
    // Point ball_prev_pos = step_start_world_ptr_->ball().position();
    // Point ball_curr_pos = world_ptr->ball().position();
    // reward += normalizeValueToRange(std::max(ball_curr_pos.x() - ball_prev_pos.x(), 0.0),
    //                                 0.0, world_ptr->field().xLength(), 0.0, 0.1);

    // TODO: Reward enemy team fouls, penalize friendly team fouls

    return reward;
}
