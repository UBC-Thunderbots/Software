#pragma once

#include "software/world/world.h"

/**
 * GameplayMonitor analyzes the state of the World at the start and end of each "step" 
 * (the period of time between actions taken by the AI agent) and computes a reward
 * representing the success of the AI's gameplay during the step. 
 * 
 * It is the reward function for reinforcement learning algorithms used in our AI.  
 */
class GameplayMonitor
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
     * of the AI's gameplay during the step. 
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