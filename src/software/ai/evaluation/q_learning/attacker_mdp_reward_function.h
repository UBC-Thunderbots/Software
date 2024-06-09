#pragma once

#include "software/world/world.h"

/**
 * AttackerMdpRewardFunction is the reward function for the Markov decision process (MDP)
 * modelling the Attacker agent's gameplay decision making.
 *
 * It analyzes the state of the World at the start and end of each "step" (the period of
 * time between actions taken by the agent) and computes a reward representing the success
 * of the agent's gameplay during the step.
 */
class AttackerMdpRewardFunction
{
   public:
    /**
     * Starts a new step.
     *
     * @param world_ptr the World at the start of the step
     */
    void startStepObservation(WorldPtr world_ptr);

    /**
     * Ends the current step and returns a reward representing the success
     * of the agent's gameplay during the step.
     *
     * @param world_ptr the World at the end of the step
     *
     * @return the reward for the step, normalized to range [-1, 1]
     */
    double endStepObservation(WorldPtr world_ptr);

   private:
    // The World at the start of the current step
    WorldPtr step_start_world_ptr_;
};
