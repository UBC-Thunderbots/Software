#pragma once

#include <unordered_set>

#include "software/ai/intent/all_intents.h"
#include "software/ai/intent/intent.h"
#include "software/ai/intent/intent_visitor.h"
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
     * Get assigned primitives for given assigned intents
     *
     * @param world World to navigate around
     * @assignedIntents intents to process into primitives
     *
     * @return vector of primitives for the given intents
     */
    std::vector<std::unique_ptr<Primitive>> getAssignedPrimitives(
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
     * Registers the given Intent for navigation
     *
     * @param The Intent to register
     */
    void visit(const CatchIntent &catch_intent) override;
    void visit(const ChipIntent &chip_intent) override;
    void visit(const DirectVelocityIntent &direct_velocity_intent) override;
    void visit(const DirectWheelsIntent &direct_wheels_intent) override;
    void visit(const DribbleIntent &dribble_intent) override;
    void visit(const KickIntent &kick_intent) override;
    void visit(const MoveIntent &move_intent) override;
    void visit(const MoveSpinIntent &move_spin_intent) override;
    void visit(const PivotIntent &pivot_intent) override;
    void visit(const StopIntent &stop_intent) override;

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
     * Generates path objectives from move intents
     *
     * @param move_intents intents to make into path objectives
     * @param world World to navigate around
     *
     * @return set of PathObjectives
     */
    std::unordered_set<PathObjective> getPathObjectivesFromMoveIntents(
        const std::vector<MoveIntent> &move_intents, const World &world);

    /**
     * Creates a list primitives for the list of MoveIntents
     *
     * @param move_intents intents to make into primitives
     * @param world World to navigate around
     *
     * @return list of primitives
     */
    std::vector<std::unique_ptr<Primitive>> getPrimitivesFromMoveIntents(
        const std::vector<MoveIntent> &move_intents, const World &world);

    /**
     * Creates a primitive for a given path and move intent
     *
     * @param path path to make primitive for
     * @param intent intent to make primitive
     * @param world World to navigate around
     *
     * @return unique pointer to the primitive
     */
    std::unique_ptr<Primitive> getPrimitiveFromPathAndMoveIntent(std::optional<Path> path,
                                                                 MoveIntent intent,
                                                                 const World &world);

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

    // The current Primitive the navigator has created from an Intent.
    // This variable is set by each `visit` function
    std::unique_ptr<Primitive> current_primitive;
    std::optional<RobotId> current_robot_id;

    // These are obstacles that represent robots that aren't
    // assigned move intents
    // When move intents are processed to path plan,
    // we can avoid these non-"moving" robots
    std::vector<ObstaclePtr> friendly_non_move_intent_robot_obstacles;

    // intents that need path planning
    std::vector<MoveIntent> move_intents_for_path_planning;
};
