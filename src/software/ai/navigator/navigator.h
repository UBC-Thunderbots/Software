#pragma once

#include "software/ai/intent/all_intents.h"
#include "software/ai/intent/intent.h"
#include "software/ai/intent/intent_visitor.h"
#include "software/ai/navigator/navigator.h"
#include "software/ai/navigator/obstacle/obstacle.h"
#include "software/ai/navigator/path_manager/path_manager.h"
#include "software/ai/navigator/util.h"
#include "software/ai/primitive/all_primitives.h"
#include "software/ai/primitive/primitive.h"
#include "software/util/parameter/dynamic_parameters.h"
#include "software/world/world.h"


/**
 * This Navigator converts the given Intents into their respective Primitives
 * and navigate around obstacles
 */
class Navigator : public IntentVisitor
{
   public:
    explicit Navigator(std::unique_ptr<PathManager> path_manager);

    /**
     * Get assigned primitives for given assigned intents
     *
     * @param world World to navigate around
     * @assignedIntents intents to navigate into primitives
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
     * Visits a CatchIntent to perform an operation.
     *
     * @param catch_intent The CatchIntent to visit
     */
    void visit(const CatchIntent &catch_intent) override;

    /**
     * Visits a ChipIntent to perform an operation.
     *
     * @param chip_intent The ChipIntent to visit
     */
    void visit(const ChipIntent &chip_intent) override;

    /**
     * Visits a DirectVelocityIntent to perform an operation.
     *
     * @param direct_velocity_intent The DirectVelocityIntent to visit
     */
    void visit(const DirectVelocityIntent &direct_velocity_intent) override;

    /**
     * Visits a DirectWheelsIntent to perform an operation.
     *
     * @param direct_wheels_intent The DirectWheelsIntent to visit
     */
    void visit(const DirectWheelsIntent &direct_wheels_intent) override;

    /**
     * Visits a DribbleIntent to perform an operation.
     *
     * @param dribble The DribbleIntent to visit
     */
    void visit(const DribbleIntent &dribble_intent) override;

    /**
     * Visits a KickIntent to perform an operation.
     *
     * @param kick_intent The KickIntent to visit
     */
    void visit(const KickIntent &kick_intent) override;

    /**
     * Visits a MoveIntent to perform an operation.
     *
     * @param move_intent The MoveIntent to visit
     */
    void visit(const MoveIntent &move_intent) override;

    /**
     * Visits a MoveSpinIntent to perform an operation.
     *
     * @param move_spin_intent The MoveSpinIntent to visit
     */
    void visit(const MoveSpinIntent &move_spin_intent) override;

    /**
     * Visits a PivotIntent to perform an operation.
     *
     * @param pivot_intent The PivotIntent to visit
     */
    void visit(const PivotIntent &pivot_intent) override;

    /**
     * Visits a StopIntent to perform an operation.
     *
     * @param stop_intent The StopIntent to visit
     */
    void visit(const StopIntent &stop_intent) override;

   private:
    // Path manager used to navigate around obstacles
    std::unique_ptr<PathManager> path_manager;

    // This navigators knowledge / state of the world
    World world;

    // The current Primitive the navigator has created from an Intent.
    // This variable is set by each `visit` function
    std::unique_ptr<Primitive> current_primitive;

    // This is used by the visualizer to see the planned paths
    std::vector<std::vector<Point>> planned_paths;

    // These are obstacles that represent robots that aren't
    // assigned move intents
    // When move intents are processed to path plan,
    // we can avoid these non-"moving" robots
    std::vector<Obstacle> friendly_non_move_intent_robot_obstacles;

    // intents that need path planning
    std::vector<MoveIntent> move_intents;

    /**
     * Create obstacles for the given avoid areas, with a buffer such that the edge
     * of the robot does not protrude into the area
     *
     * @param avoid_areas The areas to convert into obstacles
     *
     * @return Obstacles representing the given avoid areas
     */
    std::vector<Obstacle> getObstaclesFromAvoidAreas(
        const std::vector<AvoidArea> &avoid_areas);

    /**
     * Calculates a factor for how close p is to an enemy obstacle.
     * 0 = touching or inside
     * 1 = greater than/equal to 2m away
     * scaled linearly between these values
     *
     * @param p point to evaluate
     *
     * @return A factor from 0 to 1 for how close p is to an enemy obstacle
     */
    double getCloseToEnemyObstacleFactor(const Point &p);

    /**
     * Convert paths into primitives and add them to assigned_primitives
     *
     * @param paths paths to convert
     * @param assigned_primitives list of primitives to add to
     */
    void addPathsToAssignedPrimitives(
        const std::map<PathObjective, std::optional<Path>> &paths,
        std::vector<std::unique_ptr<Primitive>> &assigned_primitives);

    /**
     * Get Obstacles from a Team
     *
     * @return vector of obstacles
     */
    std::vector<Obstacle> getObstaclesFromTeam(const Team &team);

    /**
     * Registers this robot id as a robot that is not assigned a MoveIntent
     *
     * @param RobotId
     *
     */
    void registerNonMoveIntentRobotId(RobotId id);

    /**
     * Generates a map from path objectives to move intents
     *
     * @return map from path objectives to move intents
     */
    std::set<PathObjective> generatePathObjectives(void);

    /**
     * Adds primitives associated with all MoveIntents into assigned_primitives
     *
     * @param assigned_primitives primitives to add MoveIntent primitives to
     */
    void addMoveIntentsToAssignedPrimitives(
        std::vector<std::unique_ptr<Primitive>> &assigned_primitives);

    /**
     * Set current_primitive for a given path and intent
     *
     * @param path path to make primitive for
     * @param intent intent to make primitive
     */
    void processPathIntoCurrentPrimitive(std::optional<Path> path, MoveIntent intent);
};
