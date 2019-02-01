#include "ai/world/ball.h"
#include "grsim_communication/visitor/grsim_command_primitive_visitor.h"
#include "grsim_communication/motion_controller/motion_controller.h"

#include "geom/angle.h"
#include "gtest/gtest.h"
#include "shared/constants.h"
#include "software/ai/primitive/catch_primitive.h"
#include "software/ai/world/robot.h"

using namespace std::chrono;

#define POSITION_TOLERANCE 0.01
#define VELOCITY_BASE_TOLERANCE 0.015
#define VELOCITY_TOLERANCE_SCALE_FACTOR 0.01

// set up test class to keep deterministic time
class CatchPrimitiveTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        auto epoch       = time_point<std::chrono::steady_clock>();
        auto since_epoch = std::chrono::seconds(10000);

        // An arbitrary fixed point in time. 10000 seconds after the epoch.
        // We use this fixed point in time to make the tests deterministic.
        current_time = epoch + since_epoch;
    }

    steady_clock::time_point current_time;

    double calculateVelocityTolerance(double velocity)
    {
        return VELOCITY_BASE_TOLERANCE + VELOCITY_TOLERANCE_SCALE_FACTOR * velocity;
    }
};

TEST_F(CatchPrimitiveTest, robot_stationary_meets_ball_on_x_axis)
{
    Robot robot = Robot(1, Point(2, 2), Vector(0,0), Angle::ofRadians(0.0),
                        AngularVelocity::ofRadians(0.0), current_time);

    CatchPrimitive primitive = CatchPrimitive(1, 0, 10, 0.3);

    Ball ball = Ball(Point(0,0), Vector(1,0), current_time);

    GrsimCommandPrimitiveVisitor grsim_command_primitive_visitor =
            GrsimCommandPrimitiveVisitor(robot, ball);
    primitive.accept(grsim_command_primitive_visitor);

    MotionController::MotionControllerCommand motionCommand =  grsim_command_primitive_visitor.getMotionControllerCommand();

    EXPECT_NEAR(2.3, motionCommand.global_destination.x(), POSITION_TOLERANCE);
    EXPECT_NEAR(0, motionCommand.global_destination.y(), 0.1);
}

TEST_F(CatchPrimitiveTest, robot_moving_away_from_final_dest)
{
    Robot robot = Robot(1, Point(2, 2), Vector(-1, 1), Angle::ofRadians(0.0),
                        AngularVelocity::ofRadians(0.0), current_time);

    CatchPrimitive primitive = CatchPrimitive(1, 0, 10, 0.3);

    Ball ball = Ball(Point(0,0), Vector(1,0), current_time);

    GrsimCommandPrimitiveVisitor grsim_command_primitive_visitor =
            GrsimCommandPrimitiveVisitor(robot, ball);
    primitive.accept(grsim_command_primitive_visitor);

    MotionController::MotionControllerCommand motionCommand =  grsim_command_primitive_visitor.getMotionControllerCommand();

    EXPECT_NEAR(2.3, motionCommand.global_destination.x(), POSITION_TOLERANCE);
    EXPECT_NEAR(0, motionCommand.global_destination.y(), POSITION_TOLERANCE);
}

TEST_F(CatchPrimitiveTest, robot_moving_towards_final_dest)
{
    Robot robot = Robot(1, Point(2, 2), Vector(0,-1), Angle::ofRadians(0.0),
                        AngularVelocity::ofRadians(0.0), current_time);

    CatchPrimitive primitive = CatchPrimitive(1, 0, 10, 0.3);

    Ball ball = Ball(Point(0,0), Vector(1,0), current_time);

    GrsimCommandPrimitiveVisitor grsim_command_primitive_visitor =
            GrsimCommandPrimitiveVisitor(robot, ball);
    primitive.accept(grsim_command_primitive_visitor);

    MotionController::MotionControllerCommand motionCommand =  grsim_command_primitive_visitor.getMotionControllerCommand();

    EXPECT_NEAR(2.3, motionCommand.global_destination.x(), POSITION_TOLERANCE);
    EXPECT_NEAR(0, motionCommand.global_destination.y(), POSITION_TOLERANCE);
}

TEST_F(CatchPrimitiveTest, robot_already_in_final_dest_not_moving)
{
    Robot robot = Robot(1, Point(2.3, 0), Vector(0,0), Angle::ofRadians(0.0),
                        AngularVelocity::ofRadians(0.0), current_time);

    CatchPrimitive primitive = CatchPrimitive(1, 0, 10, 0.3);

    Ball ball = Ball(Point(0,0), Vector(1,0), current_time);

    GrsimCommandPrimitiveVisitor grsim_command_primitive_visitor =
            GrsimCommandPrimitiveVisitor(robot, ball);
    primitive.accept(grsim_command_primitive_visitor);

    MotionController::MotionControllerCommand motionCommand =  grsim_command_primitive_visitor.getMotionControllerCommand();

    EXPECT_NEAR(2.3, motionCommand.global_destination.x(), POSITION_TOLERANCE);
    EXPECT_NEAR(0, motionCommand.global_destination.y(), POSITION_TOLERANCE);
}

TEST_F(CatchPrimitiveTest, robot_close_to_ball)
{
    Robot robot = Robot(1, Point(0.2, 0), Vector(1,0), Angle::ofRadians(0.0),
                        AngularVelocity::ofRadians(0.0), current_time);

    CatchPrimitive primitive = CatchPrimitive(1, 0.2, 10, 0.3);

    Ball ball = Ball(Point(0,0), Vector(1,0), current_time);

    GrsimCommandPrimitiveVisitor grsim_command_primitive_visitor =
            GrsimCommandPrimitiveVisitor(robot, ball);
    primitive.accept(grsim_command_primitive_visitor);

    MotionController::MotionControllerCommand motionCommand =  grsim_command_primitive_visitor.getMotionControllerCommand();



    EXPECT_NEAR(0, motionCommand.global_destination.x(), POSITION_TOLERANCE);
    EXPECT_NEAR(0, motionCommand.global_destination.y(), POSITION_TOLERANCE);
    EXPECT_NEAR(0.2, motionCommand.final_speed_at_destination, POSITION_TOLERANCE);
}



int main(int argc, char **argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
