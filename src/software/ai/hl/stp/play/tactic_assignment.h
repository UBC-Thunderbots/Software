#include "proto/primitive.pb.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/motion_constraint/motion_constraint_set_builder.h"
#include "software/ai/navigator/trajectory/trajectory_planner.h"
#include "software/world/robot.h"
#include "software/world/world.h"

using TacticVector = std::vector<std::shared_ptr<Tactic>>;


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
assignTactics(const WorldPtr& world, TacticVector tactic_vector,
              const std::vector<Robot>& robots_to_assign,
              RobotNavigationObstacleFactory obstacle_factory,
              TbotsProto::ObstacleList& obstacle_list,
              TbotsProto::PathVisualization& path_visualization);
