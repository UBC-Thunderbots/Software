#include <gtest/gtest.h>

#include "shared/constants.h"
#include "software/ai/hl/stp/tactic/receiver/receiver_tactic.h"
#include "software/geom/algorithms/distance.h"
#include "software/test_util/test_util.h"

class OneTouchShotDirectionTest
    : public ::testing::TestWithParam<std::tuple<Point, Point, double, double>>
{
};
TEST_P(OneTouchShotDirectionTest, test_shot_towards_enemy_net)
{
    Point robot_location     = std::get<0>(GetParam());
    Point ball_location      = std::get<1>(GetParam());
    double min_angle_degrees = std::get<2>(GetParam());
    double max_angle_degrees = std::get<3>(GetParam());

    // Create a ball traveling from the specified position towards the robot
    Ball ball(ball_location, robot_location - ball_location, Timestamp::fromSeconds(0));

    // Create a shot towards the enemy net
    Ray shot(robot_location, Vector(4.5, 0) - robot_location.toVector());

    Angle robot_angle = ReceiverFSM::getOneTouchShotDirection(shot, ball);

    EXPECT_GT(robot_angle.toDegrees(), min_angle_degrees);
    EXPECT_LT(robot_angle.toDegrees(), max_angle_degrees);
}
// Since the exact direction for one touch shots is highly variable and depends a lot on
// physical tests, we can't check the exact angles, but we can at least test that they're
// in the right range
INSTANTIATE_TEST_CASE_P(
    All, OneTouchShotDirectionTest,
    ::testing::Values(
        // Robot at the origin, ball coming at it from different directions
        std::make_tuple<Point, Point, double, double>({0, 0}, {1, 1}, 1, 20),
        std::make_tuple<Point, Point, double, double>({0, 0}, {3, 1}, 1, 20),
        std::make_tuple<Point, Point, double, double>({0, 0}, {3, -1}, -20, -1),
        std::make_tuple<Point, Point, double, double>({0, 0}, {1, -1}, -20, -1),
        std::make_tuple<Point, Point, double, double>({0, 0}, {0, 1}, 1, 40),
        std::make_tuple<Point, Point, double, double>({0, 0}, {0, -1}, -40, -1),
        // Corner kicks, robot is roughly in the opposite corner of the goal crease to
        // where the corner kick is coming from
        std::make_tuple<Point, Point, double, double>({3, 1}, {4.5, -3}, -45, -5),
        std::make_tuple<Point, Point, double, double>({3, -1}, {4.5, 3}, 5, 45),
        // Corner kicks, robot is roughly in the same corner of the goal crease to
        // where the corner kick is coming from
        std::make_tuple<Point, Point, double, double>({3, -1}, {4.5, -3}, 0, 45),
        std::make_tuple<Point, Point, double, double>({3, 1}, {4.5, 3}, -45, 0),
        // Corner kick, robot is close to the goal and directly in front of it
        std::make_tuple<Point, Point, double, double>({4, 0}, {4.5, -3}, -45, -1)));

class OneTouchShotPositionTest
    : public ::testing::TestWithParam<std::tuple<double, double, double, double, double>>
{
};
/**
 * This test was added because during field testing we observed that in rare cases, the
 * receiver robot would move away from the ball's trajectory rather than towards it, and
 * therefore completely miss the pass. The behaviour did not seem to be caused by bad
 * vision data, and seemed more like a logic issue, so this test was added to try catch
 * any strange edge cases in the logic that would cause the robot to move to the wrong
 * position
 */
TEST_P(OneTouchShotPositionTest, test_receiver_moves_to_correct_one_touch_shot_position)
{
    Point robot_position(std::get<0>(GetParam()), std::get<1>(GetParam()));
    Point ball_position(std::get<2>(GetParam()), std::get<3>(GetParam()));
    // We just choose a moderate speed for these tests. Varying the speed won't change
    // the results since it's treated as an "ideal ball trajectory" anyway
    double ball_speed                 = 4;
    Angle ball_velocity_vector_offset = Angle::fromDegrees(std::get<4>(GetParam()));
    // We apply angular "noise" to the ball velocity vector to simulate imperfect passes
    Vector ball_velocity_vector = Vector::createFromAngle(
        (robot_position - ball_position).orientation() + ball_velocity_vector_offset);

    // Create a ball traveling from the specified position towards the robot
    Ball ball(ball_position, ball_velocity_vector.normalize(ball_speed),
              Timestamp::fromSeconds(0));

    // Create a robot at the robot location with no velocity. The initial orientation
    // does not matter.
    Robot robot(0, robot_position, Vector(0, 0), Angle::zero(), AngularVelocity::zero(),
                Timestamp::fromSeconds(0));

    // Create a best shot towards the center of the enemy goal
    Point best_shot_target = Point(4.5, 0);

    Shot shot =
        ReceiverFSM::getOneTouchShotPositionAndOrientation(robot, ball, best_shot_target);
    Point ideal_position    = shot.getPointToShootAt();
    Angle ideal_orientation = shot.getOpenAngle();

    // The position where the ball should make contact with the receiver robot
    Point ball_contact_position =
        ideal_position +
        Vector::createFromAngle(ideal_orientation)
            .normalize(DIST_TO_FRONT_OF_ROBOT_METERS + BALL_MAX_RADIUS_METERS);

    // We check that the position the receiver tries to move to will cause the
    // ball contact point to intersect with the ball's trajectory, meaning that we are
    // in the correct position to make contact with the ball
    Line ball_path = Line(ball.position(), ball.position() + ball.velocity());
    double dist_to_ball_passthrough = distance(ball_path, ball_contact_position);

    EXPECT_TRUE(dist_to_ball_passthrough < 0.001);
}

INSTANTIATE_TEST_CASE_P(
    All, OneTouchShotPositionTest,
    ::testing::Combine(testing::Values(-0.2),           // Robot x coordinate
                       testing::Values(0.0),            // Robot y coordinate
                       testing::Range(-3.0, 3.0, 0.5),  // Ball x coordinate
                       testing::Range(-4.5, 0.0, 0.5),  // Ball y coordinate
                       testing::Range(-5.0, 5.0, 2.5)  // Angle deviation from ideal pass,
                                                       // in degrees
                       ));
