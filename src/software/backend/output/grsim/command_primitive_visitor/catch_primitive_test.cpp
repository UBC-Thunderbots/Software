#include "software/primitive/catch_primitive.h"

#include <gtest/gtest.h>

#include "shared/constants.h"
#include "software/backend/output/grsim/command_primitive_visitor/grsim_command_primitive_visitor.h"
#include "software/backend/output/grsim/command_primitive_visitor/motion_controller.h"
#include "software/new_geom/angle.h"
#include "software/world/ball.h"
#include "software/world/robot.h"

#define POSITION_TOLERANCE 0.01
#define VELOCITY_BASE_TOLERANCE 0.015
#define VELOCITY_TOLERANCE_SCALE_FACTOR 0.01

double calculateVelocityTolerance(double velocity)
{
    return VELOCITY_BASE_TOLERANCE + VELOCITY_TOLERANCE_SCALE_FACTOR * velocity;
}

TEST(GrsimCommandPrimitiveVisitorTest,
     visit_catch_primitive_robot_stationary_meets_ball_on_x_axis)
{
    Robot robot = Robot(1, Point(2, 2), Vector(0, 0), Angle::fromRadians(0.0),
                        AngularVelocity::fromRadians(0.0), Timestamp::fromSeconds(0));

    CatchPrimitive primitive = CatchPrimitive(1, 0, 10, 0.3);

    Ball ball = Ball(Point(0, 0), Vector(1, 0), Timestamp::fromSeconds(0));

    GrsimCommandPrimitiveVisitor grsim_command_primitive_visitor =
        GrsimCommandPrimitiveVisitor(robot, ball);
    primitive.accept(grsim_command_primitive_visitor);

    MotionController::PositionCommand motionCommand =
        std::get<MotionController::PositionCommand>(
            grsim_command_primitive_visitor.getMotionControllerCommand());

    EXPECT_NEAR(2.3, motionCommand.global_destination.x(), POSITION_TOLERANCE);
    EXPECT_NEAR(0, motionCommand.global_destination.y(), 0.1);
}

TEST(GrsimCommandPrimitiveVisitorTest,
     visit_catch_primitive_robot_moving_away_from_final_dest)
{
    Robot robot = Robot(1, Point(2, 2), Vector(-1, 1), Angle::fromRadians(0.0),
                        AngularVelocity::fromRadians(0.0), Timestamp::fromSeconds(0));

    CatchPrimitive primitive = CatchPrimitive(1, 0, 10, 0.3);

    Ball ball = Ball(Point(0, 0), Vector(1, 0), Timestamp::fromSeconds(0));

    GrsimCommandPrimitiveVisitor grsim_command_primitive_visitor =
        GrsimCommandPrimitiveVisitor(robot, ball);
    primitive.accept(grsim_command_primitive_visitor);

    MotionController::PositionCommand motionCommand =
        std::get<MotionController::PositionCommand>(
            grsim_command_primitive_visitor.getMotionControllerCommand());

    EXPECT_NEAR(2.3, motionCommand.global_destination.x(), POSITION_TOLERANCE);
    EXPECT_NEAR(0, motionCommand.global_destination.y(), POSITION_TOLERANCE);
}

TEST(GrsimCommandPrimitiveVisitorTest,
     visit_catch_primitive_robot_moving_towards_final_dest)
{
    Robot robot = Robot(1, Point(2, 2), Vector(0, -1), Angle::fromRadians(0.0),
                        AngularVelocity::fromRadians(0.0), Timestamp::fromSeconds(0));

    CatchPrimitive primitive = CatchPrimitive(1, 0, 10, 0.3);

    Ball ball = Ball(Point(0, 0), Vector(1, 0), Timestamp::fromSeconds(0));

    GrsimCommandPrimitiveVisitor grsim_command_primitive_visitor =
        GrsimCommandPrimitiveVisitor(robot, ball);
    primitive.accept(grsim_command_primitive_visitor);

    MotionController::PositionCommand motionCommand =
        std::get<MotionController::PositionCommand>(
            grsim_command_primitive_visitor.getMotionControllerCommand());

    EXPECT_NEAR(2.3, motionCommand.global_destination.x(), POSITION_TOLERANCE);
    EXPECT_NEAR(0, motionCommand.global_destination.y(), POSITION_TOLERANCE);
}

TEST(GrsimCommandPrimitiveVisitorTest,
     visit_catch_primitive_robot_already_in_final_dest_not_moving)
{
    Robot robot = Robot(1, Point(2.3, 0), Vector(0, 0), Angle::fromRadians(0.0),
                        AngularVelocity::fromRadians(0.0), Timestamp::fromSeconds(0));

    CatchPrimitive primitive = CatchPrimitive(1, 0, 10, 0.3);

    Ball ball = Ball(Point(0, 0), Vector(1, 0), Timestamp::fromSeconds(0));

    GrsimCommandPrimitiveVisitor grsim_command_primitive_visitor =
        GrsimCommandPrimitiveVisitor(robot, ball);
    primitive.accept(grsim_command_primitive_visitor);

    MotionController::PositionCommand motionCommand =
        std::get<MotionController::PositionCommand>(
            grsim_command_primitive_visitor.getMotionControllerCommand());

    EXPECT_NEAR(2.3, motionCommand.global_destination.x(), POSITION_TOLERANCE);
    EXPECT_NEAR(0, motionCommand.global_destination.y(), POSITION_TOLERANCE);
}

TEST(GrsimCommandPrimitiveVisitorTest, visit_catch_primitive_robot_close_to_ball)
{
    Robot robot = Robot(1, Point(0.2, 0), Vector(1, 0), Angle::fromRadians(0.0),
                        AngularVelocity::fromRadians(0.0), Timestamp::fromSeconds(0));

    CatchPrimitive primitive = CatchPrimitive(1, 0.2, 10, 0.3);

    Ball ball = Ball(Point(0, 0), Vector(1, 0), Timestamp::fromSeconds(0));

    GrsimCommandPrimitiveVisitor grsim_command_primitive_visitor =
        GrsimCommandPrimitiveVisitor(robot, ball);
    primitive.accept(grsim_command_primitive_visitor);

    MotionController::PositionCommand motionCommand =
        std::get<MotionController::PositionCommand>(
            grsim_command_primitive_visitor.getMotionControllerCommand());

    EXPECT_NEAR(0, motionCommand.global_destination.x(), POSITION_TOLERANCE);
    EXPECT_NEAR(0, motionCommand.global_destination.y(), POSITION_TOLERANCE);
    EXPECT_NEAR(0.2, motionCommand.final_speed_at_destination, POSITION_TOLERANCE);
}
