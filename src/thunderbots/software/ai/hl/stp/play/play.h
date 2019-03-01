#pragma once

#include <memory>
#include <vector>

#include "ai/hl/stp/tactic/tactic.h"
#include "ai/world/world.h"

/**
 * In the STP framework, a Play is essentially a collection of tactics that represent some
 * "team-wide" goal. It can be thought of like a traditional play in soccer, such as an
 * offensive play, defensive play, or a specific play used for corner kicks.
 *
 * When we are running autonomously, different Plays are selected at different times
 * based on the state of the world, and this switching of Plays is what allows our AI to
 * play a full game of soccer, as long a we provide a Play for any given scenario.
 */
class Play
{
   public:
    /**
     * Returns whether or not this Play can be started. For example, the Enemy Team
     * must have the ball for a defensive play to be applicable.
     *
     * @param world The current state of the world
     * @return true if this Play can be started, and false otherwise
     */
    virtual bool isApplicable(const World& world) = 0;

    /**
     * Returns whether or not the invariant for this Play holds (is true). The invariant
     * is a set of conditions that must remain true for the Play to continue running
     * (not necessarily the same as the Applicable conditions). For example, the Invariant
     * for an Offensive Play might be that our team must have possession of the ball. If
     * our team no longer has possession of the ball (ie. the invariant no longer holds),
     * then we should probably run a different Play.
     *
     * @param world The current state of the world
     * @return true if this Play's invariant holds (is true), and false otherwise
     */
    virtual bool invariantHolds(const World& world) = 0;

    /**
     * Returns whether or not the Play has failed. This is kept as a separate condition
     * from the Invariant because it is easier to reason about. For example, a Play that
     * is supposed to retrieve a loose ball would fail if the enemy team got possession
     * of the ball
     *
     * @param world The current state of the world
     * @return true if this Play has failed, and false otherwise
     */
    virtual bool hasFailed(const World& world) = 0;

    /**
     * Gets the tactics that make up this Play. The tactics are returned in decreasing
     * order of priority (the first tactic has the highest priority, the last one has the
     * least). This means that if there are not enough available robots to run all of the
     * tactics, the tactics with the least priority will not be assigned/run
     *
     * @param world The current state of the world
     * @return A list of tactics that should be run as part of this Play, in decreasing
     * order of priority
     */
    virtual std::vector<std::unique_ptr<Tactic>> getTactics(const World& world) = 0;

    /**
     * Returns the name of this Play
     *
     * @return the name of this Play
     */
    virtual std::string name() = 0;

    virtual ~Play() = default;
};
