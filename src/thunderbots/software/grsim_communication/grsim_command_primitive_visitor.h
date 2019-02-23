#pragma once

#include <ai/world/ball.h>

#include "ai/primitive/visitor/primitive_visitor.h"
#include "ai/world/robot.h"
#include "motion_controller.h"

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
     * current moment in time in order to simulate the CatchPrimitive
     *
     * @param catch_primitive The CatchPrimitive to simulate
     */
    void visit(const CatchPrimitive &catch_primitive) override;

    /**
     * Generates and stores the MotionControllerCommand the robot should perform at the
     * current moment in time in order to simulate the ChipPrimitive
     *
     * @param chip_primitive The ChipPrimitive to simulate
     */
    void visit(const ChipPrimitive &chip_primitive) override;

    /**
     * Generates and stores the MotionControllerCommand the robot should perform at the
     * current moment in time in order to simulate the DirectVelocityPrimitive
     *
     * @param direct_velocity_primitive The DirectVelocityPrimitive to simulate
     */
    void visit(const DirectVelocityPrimitive &direct_velocity_primitive) override;



    /**
     * Generates and stores the MotionControllerCommand the robot should perform at the
     * current moment in time in order to simulate the DirectWheelsPrimitive
     *
     * @param direct_wheels_primitive The DirectWheelsPrimitive to simulate
     */
    void visit(const DirectWheelsPrimitive &direct_wheels_primitive) override;

    /**
     * Generates and stores the MotionControllerCommand the robot should perform at the
     * current moment in time in order to simulate the KickPrimitive
     *
     * @param kick_primitive The KickPrimitive to simulate
     */
    void visit(const KickPrimitive &kick_primitive) override;

    /**
     * Generates and stores the MotionControllerCommand the robot should perform at the
     * current moment in time in order to simulate the MovePrimitive
     *
     * @param move_primitive The MovePrimitive to simulate
     */
    void visit(const MovePrimitive &move_primitive) override;

    /**
     * Generates and stores the MotionControllerCommand the robot should perform at the
     * current moment in time in order to simulate the DribblePrimitive
     *
     * @param move_primitive The DribblePrimitive to simulate
     */
    void visit(const DribblePrimitive &dribble_primitive) override;


    /**
     * Generates and stores the MotionControllerCommand the robot should perform at the
     * current moment in time in order to simulate the MovePrimitive
     *
     * @param move_spin_primitive The MoveSpinPrimitive to simulate
     */
    void visit(const MoveSpinPrimitive &move_spin_primitive) override;

    /**
     * Generates and stores the MotionControllerCommand the robot should perform at the
     * current moment in time in order to simulate the PivotPrimitive
     *
     * @param pivot_primitive The PivotPrimitive to simulate
     */
    void visit(const PivotPrimitive &pivot_primitive) override;

    /**
     * Generates and stores the MotionControllerCommand the robot should perform at the
     * current moment in time in order to simulate the StopPrimitive
     *
     * @param stop_primitive The StopPrimitive to simulate
     */
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
    MotionController::MotionControllerCommand getMotionControllerCommand();

   private:
    static constexpr double STANDARD_INTERCEPT_MARGIN = 1.0;

    // The robot that the Primitive calculations will be performed for
    Robot robot;
    // The MotionControllerCommand created by the 'visit' functions that is to be sent
    // to the MotionController
    MotionController::MotionControllerCommand motion_controller_command;
    Ball ball;
};
