#pragma once

#include <unordered_set>

#include "software/ai/intent/all_intents.h"
#include "software/ai/intent/intent.h"
#include "software/ai/intent/intent_visitor.h"
#include "software/ai/navigator/obstacle/obstacle.h"
#include "software/ai/navigator/obstacle/obstacle_generation.h"
#include "software/ai/navigator/path_manager/path_manager.h"
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

   protected:
    /**
     * Calculates the transition speed for the robot between two line segments
     *
     * Calculates the speed that the robot should be at when it is at the end of a
     * given line segment in order to smoothly transition to another given line segment,
     * given a final speed at the end of the two line segments
     *
     * @param p1, p2, p3 are 3 points that define two line segments that form a path
     * @param final_speed is the intended final speed at the end of the path
     * @return the first segment's final speed after travelling from p1 to p2
     * for a smooth transition to the p2 to p3 path, scaled by the final speed at the end
     * of the path
     */
    double calculateTransitionSpeedBetweenSegments(const Point &p1, const Point &p2,
                                                   const Point &p3, double final_speed);

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
    std::vector<MoveIntent> move_intents_for_path_planning;

    /**
     * Registers this robot id as a robot that is not assigned a MoveIntent
     *
     * @param id RobotId to register
     *
     */
    void registerNonMoveIntentRobotId(RobotId id);

    /**
     * Generates path objectives from move intents
     *
     * @param move_intents intents to make into path objectives
     *
     * @return set of PathObjectives
     */
    std::unordered_set<PathObjective> getPathObjectivesFromMoveIntents(
        const std::vector<MoveIntent> &move_intents);

    /**
     * Creates a list primitives for the list of MoveIntents
     *
     * @param move_intents intents to make into primitives
     *
     * @return list of primitives
     */
    std::vector<std::unique_ptr<Primitive>> getPrimitivesFromMoveIntents(
        const std::vector<MoveIntent> &move_intents);

    /**
     * Creates a primitive for a given path and move intent
     *
     * @param path path to make primitive for
     * @param intent intent to make primitive
     *
     * @return unique pointer to the primitive
     */
    std::unique_ptr<Primitive> getPrimitiveFromPathAndMoveIntent(std::optional<Path> path,
                                                                 MoveIntent intent);

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
};
