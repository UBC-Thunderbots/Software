#pragma once

#include <variant>

#include "software/backend/output/grsim/command_primitive_visitor/motion_controller.h"
#include "software/primitive/primitive_visitor.h"
#include "software/world/ball.h"
#include "software/world/robot.h"

using MotionControllerCommand =
    std::variant<MotionController::PositionCommand, MotionController::VelocityCommand>;

/**
 * This class implements a Visitor that simulates the motion of our Primitives in grSim
 * by calculating and returning commands that are sent to our simulated MotionController
 */
class GrsimCommandPrimitiveVisitor : public PrimitiveVisitor
{
   public:
    /**
     * Creates a new GrsimCommandPrimitiveVisitor
     *
     * @param robot The robot that will be performing the Primitive. This is the robot
     * that the resultant MotionControllerCommand will be sent to.
     */
    GrsimCommandPrimitiveVisitor(const Robot &robot, const Ball &ball);

    /**
     * Generates and stores the MotionControllerCommand the robot should perform at the
     * current moment in time in order to simulate the given Primitive
     *
     * @param primitive The Primitive to simulate
     */
    void visit(const CatchPrimitive &catch_primitive) override;
    void visit(const ChipPrimitive &chip_primitive) override;
    void visit(const DirectVelocityPrimitive &direct_velocity_primitive) override;
    void visit(const DirectWheelsPrimitive &direct_wheels_primitive) override;
    void visit(const KickPrimitive &kick_primitive) override;
    void visit(const MovePrimitive &move_primitive) override;
    void visit(const DribblePrimitive &dribble_primitive) override;
    void visit(const MoveSpinPrimitive &move_spin_primitive) override;
    void visit(const PivotPrimitive &pivot_primitive) override;
    void visit(const StopPrimitive &stop_primitive) override;

    /**
     * Returns the most recent MotionControllerCommand created by this
     * GrSimCommandPrimitiveVisitor. This is the MotionControllerCommand created by
     * one of the 'visit' functions.
     *
     * Calling this function before this Visitor has been called at least once has
     * undefined behavior
     *
     * @return The most recent MotionControllerCommand created by this
     * GrSimCommandPrimitiveVisitor
     */
    MotionControllerCommand getMotionControllerCommand();

   private:
    static constexpr double STANDARD_INTERCEPT_MARGIN = 1.0;

    // The robot that the Primitive calculations will be performed for
    Robot robot;
    // The MotionControllerCommand created by the 'visit' functions that is to be sent
    // to the MotionController. THis can be either a position or velocity command.
    MotionControllerCommand motion_controller_command;
    Ball ball;
};
