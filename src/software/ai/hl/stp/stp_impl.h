#include "software/ai/hl/stp/play/play.h"

/**
 * Given a list of tactics and the current World, assigns robots from the friendly
 * team to each tactic
 *
 * Some tactics may not be assigned a robot, depending on if there is a robot
 * capable of performing that tactic
 *
 * This will clear all assigned robots from all tactics
 *
 * The order of the given tactics determines their priority, with the tactics as the
 * beginning of the vector being a higher priority than those at the end. The priority
 * determines which tactics will NOT be assigned if there are not enough robots on the
 * field to assign them all. For example, if a Play returned 6 Tactics but there were
 * only 4 robots on the field at the time, only the first 4 Tactics in the vector
 * would be assigned to robots and run.
 *
 * @param world The state of the world, which contains the friendly Robots that will
 * be assigned to each tactic
 * @param [in/out] tactics The list of tactics that should be assigned a robot. Note
 * that this function modifies tactics to make the correct assignments, because we need to
 * modify the individual tactics _and_ possibly add/remove tactics
 *
 * @return The list of tactics that were assigned to the robots
 */
void assignRobotsToTactics(const World &world,
                           std::vector<std::shared_ptr<Tactic>> &tactics);

/**
 * Assigns non goalie robots to each non goalie tactic
 *
 * Some tactics may not be assigned a robot, depending on if there is a robot
 * capable of performing that tactic
 *
 * This will clear all assigned robots from all tactics
 *
 * The order of the given tactics determines their priority, with the tactics as the
 * beginning of the vector being a higher priority than those at the end. The priority
 * determines which tactics will NOT be assigned if there are not enough robots on the
 * field to assign them all. For example, if a Play returned 6 Tactics but there were
 * only 4 robots on the field at the time, only the first 4 Tactics in the vector
 * would be assigned to robots and run.
 *
 * @param world The state of the world for calculating robot costs
 * @param non_goalie_robots The non goalie robots to assign to tactics
 * @param [in/out] non_goalie_tactics The list of tactics that should be assigned a
 * robot. Note that this function modifies non_goalie_tactics to make the correct
 * assignments, because we need to modify the individual tactics _and_ possibly add/remove
 * tactics
 *
 * @return The list of tactics that were assigned to the robots
 */
void assignNonGoalieRobotsToTactics(
    const World &world, const std::vector<Robot> &non_goalie_robots,
    std::vector<std::shared_ptr<Tactic>> &non_goalie_tactics);
