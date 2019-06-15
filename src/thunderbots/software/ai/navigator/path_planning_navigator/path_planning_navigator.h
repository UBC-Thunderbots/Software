#pragma once

#include "ai/intent/intent.h"
#include "ai/intent/visitor/intent_visitor.h"
#include "ai/navigator/navigator.h"
#include "ai/navigator/obstacle/obstacle.h"
#include "ai/navigator/path_planner/straight_line_path_planner.h"
#include "ai/navigator/path_planner/theta_star_path_planner.h"
#include "ai/navigator/path_planning_navigator/path_planning_navigator.h"
#include "ai/primitive/primitive.h"

/**
 * This PathPlanningNavigator is a simple navigator that will convert the given Intents
 * into their respective Primitives, without doing any intelligent navigation. It is meant
 * to provide an example of how to implement a navigator and act as a starting point for
 * future navigators to build off of.
 */
class PathPlanningNavigator : public Navigator, public IntentVisitor
{
   public:
    explicit PathPlanningNavigator(){

    };

    std::vector<std::unique_ptr<Primitive>> getAssignedPrimitives(
        const World &world,
        const std::vector<std::unique_ptr<Intent>> &assignedIntents) override;

    static double place_holder_violation_func(const Point &input_point);
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
    // This navigators knowledge / state of the world
    World world;
    // The current Primitive the navigator has created from an Intent.
    // This variable is set by each `visit` function
    std::unique_ptr<Primitive> current_primitive;
};
