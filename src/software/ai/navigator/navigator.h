#pragma once

#include <unordered_set>

#include "shared/proto/tbots_software_msgs.pb.h"
#include "software/ai/intent/all_intents.h"
#include "software/ai/intent/intent.h"
#include "software/ai/navigator/obstacle/obstacle.h"
#include "software/ai/navigator/obstacle/robot_navigation_obstacle_factory.h"
#include "software/ai/navigator/path_manager/path_manager.h"
#include "software/parameter/dynamic_parameters.h"
#include "software/primitive/all_primitives.h"
#include "software/primitive/primitive.h"
#include "software/world/world.h"

/**
 * This Navigator converts the given Intents into their respective Primitives
 * and navigate around obstacles
 */
class Navigator
{
   public:
    /**
     * Create a Navigator
     * @param path_manager The path manager that will be used for path creation for all
     *                     the robots
     * @param robot_navigation_obstacle_factory Will be used to generate obstacles from
     * various constructs
     * @param config The navigator config
     */
    explicit Navigator(std::unique_ptr<PathManager> path_manager,
                       RobotNavigationObstacleFactory robot_navigation_obstacle_factory,
                       std::shared_ptr<const NavigatorConfig> config);

    /**
     * Get assigned primitives for given assigned intents
     *
     * @param world World to navigate around
     * @assigned_intents intents to process into primitives
     *
     * @return vector of primitives for the given intents
     */
    std::vector<std::unique_ptr<Primitive>> getAssignedPrimitives(
        const World &world, const std::vector<std::unique_ptr<Intent>> &assigned_intents);

    /**
     * Get PrimitiveSetMsg for given assigned intents
     *
     * @param world World to navigate around
     * @assignedIntents intents to process into primitives
     *
     * @return PrimitiveSetMsg
     */
    std::unique_ptr<PrimitiveSetMsg> getAssignedPrimitiveSetMsg(
        const World &world, const std::vector<std::unique_ptr<Intent>> &assignedIntents);

    /**
     * Get the planned paths for navigation
     *
     * @return planned paths
     */
    std::vector<std::vector<Point>> getPlannedPathPoints();

    /**
     * Get the obstacles for navigation
     *
     * @return obstacles
     */
    std::vector<ObstaclePtr> getObstacles();

    /**
     * Calculates the transition speed for the robot between two line segments
     *
     * Calculates the speed that the robot should be at when it is at the end of a
     * given line segment in order to smoothly transition to another given line segment,
     * given a final speed at the end of the two line segments
     *
     * This is only public so it is testable.
     *
     * @param p1, p2, p3 are 3 points that define two line segments that form a path
     * @param final_speed is the intended final speed at the end of the path
     * @return the first segment's final speed after travelling from p1 to p2
     * for a smooth transition to the p2 to p3 path, scaled by the final speed at the end
     * of the path
     */
    static double calculateTransitionSpeedBetweenSegments(const Point &p1,
                                                          const Point &p2,
                                                          const Point &p3,
                                                          double final_speed);

   private:
    /**
     * Generates path objectives from intents
     *
     * @param intents intents to make into path objectives
     * @param world World to navigate around
     * @param friendly_non_navigating_robot_obstacles The obstacles representing friendly
     * non-navigating robots
     *
     * @return set of PathObjectives
     */
    std::unordered_set<PathObjective> getPathObjectivesFromIntents(
        const std::vector<std::unique_ptr<Intent>> &intents, const World &world,
        std::vector<ObstaclePtr> friendly_non_navigating_robot_obstacles);

    /**
     * Creates the final speed and destination given the navigator params, the path, the
     * intent, and the world
     *
     * @param navigator_params NavigatorParams
     * @param path path to make primitive for
     * @param intent intent to update
     * @param world World to navigate around
     *
     * @return the final destination and speed
     */
    std::pair<Point, double> calculateDestinationAndFinalSpeed(
        NavigatorParams navigator_params, Path path,
        const std::unique_ptr<Intent> &intent, const World &world);

    /**
     * Calculates a factor for how close p is to an enemy obstacle.
     * 0 = touching or inside
     * 1 = greater than/equal to EnemyRobotProximityLimit (dynamic parameter) away
     * scaled linearly between these values
     *
     * @param p point to evaluate
     * @param enemy_team enemy team
     *
     * @return A factor from 0 to 1 for how close p is to an enemy obstacle
     */
    double getEnemyObstacleProximityFactor(const Point &p, const Team &enemy_team);

    std::shared_ptr<const NavigatorConfig> config;
    RobotNavigationObstacleFactory robot_navigation_obstacle_factory;
    std::unique_ptr<PathManager> path_manager;
    std::vector<std::vector<Point>> planned_paths;
};
