#pragma once

#include <memory>
#include <vector>

#include "ai/intent/intent.h"
#include "ai/world/world.h"

/**
 * An abstraction for the high-level logic of our AI. The high-level logic is responsible
 * for the major strategic gameplay decisions of the AI.
 *
 * This is an Abstract class meant to define the interface that all HL modules must
 * follow. Other classes should inherit from this class and implement the methods to
 * create a concrete implementation of a HL module. This allows us to potentially have
 * multiple different high-level decision making modules, that we can swap out or combine
 * at runtime.
 */
class HL
{
   public:
    /**
     * Given the state of the world, returns the Intent that each available Robot should
     * be running.
     *
     * @param world The current state of the world
     *
     * @return A vector of unique pointers to the Intents our friendly robots should be
     * running
     */
    virtual std::vector<std::unique_ptr<Intent>> getIntents(const World &world) = 0;

    virtual ~HL() = default;
};
