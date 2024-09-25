#pragma once

#include "software/ai/strategy.h"

/**
 * State representation of the Markov decision process (MDP) modelling
 * the Attacker agent's gameplay decision making.
 */
struct AttackerMdpState
{
    WorldPtr world_ptr;
    std::shared_ptr<Strategy> strategy;
};
