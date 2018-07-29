#pragma once

#include "ai/intent.h"
#include "ai/world/world.h"

/**
 * An abstraction for the high-level logic of our AI. The high-level logic is responsible
 * for the major
 * strategic gameplay decisions of the AI.
 *
 * This is an Abstract. It is meant to define the interace that all HL modules must
 * follow.
 * Other classes should inherit from this class and implement the methods to create a
 * useable HL class. This
 * allows us to potentially have multiple different high-level decision making modules,
 * that we can swap out or
 * combine at runtime.
 */
class HL
{
   public:
    // TODO: Will need timestamps

    /**
    * Given the state of the world, returns the intent that each available Robot should be
    * running.
    *
    * @param world The current state of the world
    * @return A vector of pairs, where each pair contains a robot id and the Intent that
    * robot should run
    */
    virtual std::vector<std::pair<unsigned int, Intent>> getIntentAssignment(
        const World &world);
};
