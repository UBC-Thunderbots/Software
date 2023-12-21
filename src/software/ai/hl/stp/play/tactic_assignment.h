#include "software/world/world.h"
#include "software/world/robot.h"
#include "software/ai/navigator/path_planner/global_path_planner_factory.h"
#include "proto/primitive.pb.h"

using TacticVector              = std::vector<std::shared_ptr<Tactic>>;


/**
 * Assigns the given tactics to as many of the given robots
 *
 * @param path_planner_factory The path planner factory
 * @param world The world
 * @param tactic_vector The tactic vector
 * @param robots_to_assign The robots to assign to
 *
 * @return the remaining unassigned robots, the new primitives to assign, and robot to
 * tactic assignment
 */
std::tuple<std::vector<Robot>, std::unique_ptr<TbotsProto::PrimitiveSet>,
           std::map<std::shared_ptr<const Tactic>, RobotId>>
           assignTactics(const GlobalPathPlannerFactory &path_planner_factory,
                    const World &world, TacticVector tactic_vector,
                    const std::vector<Robot> robots_to_assign);

/**
 * Gets Primitives from a Tactic given the path planner factory, the world, and the
 * tactic
 *
 * @param path_planner_factory The path planner factory
 * @param world The updated world
 * @param tactic the Tactic
 * @param motion_constraints the motion constraints to use
 *
 * @return the PrimitiveSet to execute
 */
std::unique_ptr<TbotsProto::PrimitiveSet> getPrimitivesFromTactic(
    const GlobalPathPlannerFactory &path_planner_factory, const World &world,
    std::shared_ptr<Tactic> tactic,
    std::set<TbotsProto::MotionConstraint> motion_constraints);