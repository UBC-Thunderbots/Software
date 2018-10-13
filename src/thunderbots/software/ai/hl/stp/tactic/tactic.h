#pragma once

#include <memory>

#include "ai/intent/intent.h"
#include "ai/world/world.h"

/**
 * In the STP framework, a Tactic represents a role or objective for a single robot. For
 * example, being the goalie, shooting on the opposing goal, moving to receive a pass,
 * or acting as a defender. Tactics are where most of the complicated logic takes place,
 * and they tend to rely a lot on our Evaluation functions. Ultimately, Tactics will
 * return the next Intent that the Robot assigned to this Tactic should run in order
 * to work towards its objective.
 */
class Tactic
{
   public:
    /**
     * Returns the next Intent that should be run by the Robot assigned to this Tactic
     * in order to work towards achieving the Tactic's objective
     * @param world The current state of the world
     * @param robot The Robot assigned to this Tactic
     * @return The Intent that the Robot should run in order to work towards the Tactic's
     * objective
     */
    virtual std::unique_ptr<Intent> getNextIntent(const World& world,
                                                  const Robot& robot) = 0;

    /**
     * Returns the Robot that the Tactic would prefer to use
     * @param available_robots The list of available Robots that the Tactic can choose
     * from
     * @return The Robot that the Tactic prefers to use
     */
    virtual Robot selectRobot(const World& world,
                              const std::vector<Robot>& available_robots) = 0;

    virtual ~Tactic() = default;
};
