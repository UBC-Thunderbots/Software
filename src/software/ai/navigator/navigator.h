#pragma once

#include "shared/parameter/cpp_dynamic_parameters.h"
#include "shared/proto/tbots_software_msgs.pb.h"
#include "software/ai/intent/all_intents.h"
#include "software/ai/intent/intent.h"
#include "software/ai/intent/intent_visitor.h"
#include "software/ai/navigator/obstacle/obstacle.h"
#include "software/ai/navigator/obstacle/robot_navigation_obstacle_factory.h"
#include "software/ai/navigator/path_manager/path_manager.h"
#include "software/world/world.h"

/**
 * The Navigator converts the given Intents into their respective Primitives
 * and navigate around obstacles
 */
class Navigator : public IntentVisitor
{
   public:
    Navigator() = delete;
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
    std::unique_ptr<TbotsProto::PrimitiveSet> getAssignedPrimitives(
        const World &world, const std::vector<std::unique_ptr<Intent>> &intents) const;

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
    void visit(const MoveIntent &intent) override;

   private:
    /**
     * Generates path objectives
     *
     * @param world World to navigate around
     *
     * @return set of PathObjectives
     */
    std::unordered_set<PathObjective> createPathObjectives(const World &world);

    std::shared_ptr<const NavigatorConfig> config;
    RobotNavigationObstacleFactory robot_navigation_obstacle_factory;
    std::unique_ptr<PathManager> path_manager;

    std::vector<std::vector<Point>> planned_paths;
    std::vector<std::shared_ptr<NavigatingIntent>> navigating_intents;
    // These are the robots that were assigned direct primitive intents.
    // When navigating intents are processed to path plan, we can avoid these
    // non-navigating robots
    std::vector<RobotId> direct_primitive_intent_robots;
    std::unique_ptr<TbotsProto::PrimitiveSet> primitive_set_msg;
};
