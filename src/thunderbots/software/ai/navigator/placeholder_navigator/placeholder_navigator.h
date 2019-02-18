#pragma once

#include "ai/navigator/navigator.h"
#include "ai/intent/visitor/intent_visitor.h"
#include "ai/intent/intent.h"
#include "ai/primitive/primitive.h"

class PlaceholderNavigator : public Navigator, public IntentVisitor {
public:
    explicit PlaceholderNavigator() = default;

    std::vector<std::unique_ptr<Primitive>> getAssignedPrimitives(
            const World &world,
            const std::vector<std::unique_ptr<Intent>> &assignedIntents) override;

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
    World world;
    std::vector<std::unique_ptr<Intent>> assignedIntents;
    std::unique_ptr<Primitive> curr_prim;
};
