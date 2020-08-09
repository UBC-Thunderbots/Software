#pragma once

#include <unordered_set>

#include "shared/proto/tbots_software_msgs.pb.h"
#include "software/ai/intent/all_intents.h"
#include "software/ai/intent/intent.h"
#include "software/ai/intent/intent_visitor.h"
#include "software/ai/navigator/obstacle/obstacle.h"
#include "software/ai/navigator/obstacle/robot_navigation_obstacle_factory.h"
#include "software/ai/navigator/path_manager/path_manager.h"
#include "software/parameter/dynamic_parameters.h"
#include "software/world/world.h"

/**
 * This Navigator converts the given Intents into their respective Primitives
 * and navigate around obstacles
 */
class Navigator : public IntentVisitor
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
     * Get Primitives for given assigned intents
     *
     * @param world The World to navigate around
     * @param intents The intents to process into primitives
     *
     * @return Primitives
     */
    std::unique_ptr<PrimitiveSetMsg> getAssignedPrimitives(
        const World &world, const std::vector<std::unique_ptr<Intent>> &intents);

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
     * Registers the given Intent for navigation
     *
     * @param The Intent to register
     */
    void visit(const DirectPrimitiveIntent &intent) override;
    void visit(const NavigatingIntent &intent) override;
    void visit(const MoveIntent &intent) override;

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
     * Generates path objectives
     *
     * @param intents intents to make into path objectives
     * @param world World to navigate around
     *
     * @return vector of PathObjectives
     */
    std::vector<PathObjective> createPathObjectives(
        const std::vector<std::unique_ptr<Intent>> &intents, const World &world);

    /**
     * Creates the final speed and destination given the final speed and the path
     *
     * @param final_speed The final speed
     * @param path path to make primitive for
     *
     * @return the final destination and speed
     */
    std::pair<Point, double> calculateDestinationAndFinalSpeed(double final_speed,
                                                               Path path);

    std::shared_ptr<const NavigatorConfig> config;
    RobotNavigationObstacleFactory robot_navigation_obstacle_factory;
    std::unique_ptr<PathManager> path_manager;
    std::vector<std::vector<Point>> planned_paths;
    std::optional<PrimitiveMsg> current_primitive;
    std::map<RobotId, std::optional<Path>> robot_id_to_path;
};
