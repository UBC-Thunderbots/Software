#include "software/primitive/pivot_primitive.h"

#include <gtest/gtest.h>

#include "shared/constants.h"
#include "software/backend/output/grsim/command_primitive_visitor/grsim_command_primitive_visitor.h"
#include "software/backend/output/grsim/command_primitive_visitor/motion_controller.h"
#include "software/new_geom/angle.h"
#include "software/world/ball.h"
#include "software/world/robot.h"

/*
 * Pivot GrSim Primitive Tests
 *
 * Functionality of pivot includes:
 *  - pivoting around a point at a fixed radius (integration test, covered implicitly
 * here)
 *  - choosing the shortest path to pivot
 *  - stopping consistently at the final position
 *
 *  This file will attempt to test all the cases
 */
class GrsimCommandPrimitiveVisitorParameterizedTest
    : public ::testing::TestWithParam<Point>
{
};

/*
 * Takes a robot and pivot primitive object references and creates
 * a grsim primitive visitor, visits the primitive, gets the motion controller command,
 * and returns it. Pivot doesn't need a ball, so a ball is created in this util function.
 *
 * @param robot, reference to the robot object that will be assigned the primitive
 * @param test_primitive, reference to the primitive object, to visit and get the motion
 * command
 * @returns motionCommand, the motion command obtained after visiting the primitive
 *
 */
MotionController::PositionCommand get_motion_command(Robot& robot,
                                                     PivotPrimitive& test_primitive)
{
    // create a test ball, pivot doesn't need a ball
    Ball ball = Ball(Point(0, 0), Vector(0, 0), Timestamp::fromSeconds(0));

    // create primitive visitor
    GrsimCommandPrimitiveVisitor grsim_command_primitive_visitor =
        GrsimCommandPrimitiveVisitor(robot, ball);

    // visit primitive
    test_primitive.accept(grsim_command_primitive_visitor);

    // get motion command
    MotionController::PositionCommand motionCommand =
        std::get<MotionController::PositionCommand>(
            grsim_command_primitive_visitor.getMotionControllerCommand());

    return motionCommand;
}

// robot needs to pick the shortest path to rotate, based on where it is and the magnitude
// of the vector from its next position to the final position.

// Test picking a clockwise path
TEST_P(GrsimCommandPrimitiveVisitorParameterizedTest, visit_pivot_primtive_clockwise)
{
    // asked to pivot to 1/4 pi below x axis, with radius of 10, from point (10, -10)
    // (chosen far so all values pivot cw) which should result in a cw rotation on all
    // parameterized pivot points
    PivotPrimitive primitive =
        PivotPrimitive(1, Point(10, -10), Angle::fromRadians(-1 / 4 * M_PI),
                       Angle::fromRadians(1.24), false);

    // place the robot in the first quadrant, should rotate CW
    Robot robot = Robot(1, GetParam(), Vector(0, 0), Angle::fromRadians(0.0),
                        AngularVelocity::fromRadians(0.0), Timestamp::fromSeconds(0));

    auto motion_command = get_motion_command(robot, primitive);

    // figure out if the tangential vector is clockwise, take a cross product of a 2d
    // vector in 3d plane
    // https://math.stackexchange.com/questions/74307/two-2d-vector-angle-clockwise-predicate

    // this vector connects the robot to the pivot point, the tangential vectors
    // that look at both directions will be perpendicular to this vector
    Vector radial_vector = robot.position() - primitive.getPivotPoint();
    Vector direction_vector =
        motion_command.global_destination - primitive.getPivotPoint();

    // if c is less than 0, then direction vector is on the clockwise side of radial
    // vector, which means the robot is rotating clockwise
    float c = static_cast<float>(radial_vector.x() * direction_vector.y() -
                                 radial_vector.y() * direction_vector.x());
    EXPECT_LE(c, 0);
}

// Test picking a counter-clockwise path
TEST_P(GrsimCommandPrimitiveVisitorParameterizedTest,
       visit_pivot_primtive_counter_clockwise)
{
    // asked to pivot to 3/4 pi above x axis, with radius of 10, from point (-10, 10)
    // (chosen far so all values pivot ccw) which should result in a cw rotation on all
    // parameterized pivot points
    PivotPrimitive primitive =
        PivotPrimitive(1, Point(-10, 10), Angle::fromRadians(3 / 4 * M_PI),
                       Angle::fromRadians(1.24), false);

    // place the robot in the first quadrant, should rotate CW
    Robot robot = Robot(1, GetParam(), Vector(0, 0), Angle::fromRadians(0.0),
                        AngularVelocity::fromRadians(0.0), Timestamp::fromSeconds(0));

    auto motion_command = get_motion_command(robot, primitive);

    // figure out if the tangential vector is clockwise, take a cross product of a 2d
    // vector in 3d plane
    // https://math.stackexchange.com/questions/74307/two-2d-vector-angle-clockwise-predicate

    // this vector connects the robot to the pivot point, the tangential vectors
    // that look at both directions will be perpendicular to this vector
    Vector radial_vector = robot.position() - primitive.getPivotPoint();
    Vector direction_vector =
        motion_command.global_destination - primitive.getPivotPoint();

    // if c is greater than 0, then direction vector is on the counter-clockwise side of
    // radial vector, which means the robot is rotating counter-clockwise
    float c = static_cast<float>(radial_vector.x() * direction_vector.y() -
                                 radial_vector.y() * direction_vector.x());
    EXPECT_GE(c, 0);
}

// Create the parameterize test case for the tests above
//
// Robot should not move test, uses it for both robot position and pivot point position
// The following two tests use it to place the pivot point at different locations, and
// then the tests make sure that the robot choses the right direction to move.
//
// If stopping, and rotating work, pivot should work
INSTANTIATE_TEST_CASE_P(Positions, GrsimCommandPrimitiveVisitorParameterizedTest,
                        ::testing::Values(Point(1, 0), Point(0, 1), Point(-1, 0),
                                          Point(0, -1), Point(4, 2), Point(-4, 2)));
