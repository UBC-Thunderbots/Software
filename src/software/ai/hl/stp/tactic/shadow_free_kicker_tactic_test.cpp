#include <gtest/gtest.h>

#include "shared/constants.h"
#include "software/ai/hl/stp/tactic/shadow_free_kicker_tactic.h"
#include "software/ai/intent/move_intent.h"
#include "software/geom/algorithms/distance.h"
#include "software/geom/line.h"
#include "software/test_util/test_util.h"

class ShadowFreekickerTacticTest : public testing::Test
{
   protected:
    /**
     * The fixture function to help to create world with enemy robots and calculate
     * distances needed for unit tests
     * @param ball_position: the position of the ball in Point format
     * @param enemy_robot_positions: a vector of Points to represent the position of
     * robots
     * @param left_or_right: a enum argument in the shadowFreekickerTactic to represent
     * whether the robot is  blocking the free_kick from left or right
     */
    void CreateSceneWithEnemyRobot(Point ball_position,
                                   std::vector<Point> enemy_robot_positions,
                                   ShadowFreekickerTactic::FreekickShadower left_or_right)
    {
        // set up world, robots, balls
        world = ::TestUtil::createBlankTestingWorld();
        world = ::TestUtil::setEnemyRobotPositions(world, enemy_robot_positions,
                                                   Timestamp::fromSeconds(0));

        world =
            ::TestUtil::setBallPosition(world, ball_position, Timestamp::fromSeconds(0));
        world =
            ::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(0));

        Robot friendly_robot(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                             AngularVelocity::zero(), Timestamp::fromSeconds(0));
        world.updateFriendlyTeamState(Team({friendly_robot}));

        // setup tactic and acquire move action from it
        ShadowFreekickerTactic tactic = ShadowFreekickerTactic(
            left_or_right, world.enemyTeam(), world.ball(), world.field(), false);
        tactic.updateRobot(friendly_robot);

        auto action_ptr = tactic.getNextAction();
        ASSERT_TRUE(action_ptr);

        auto move_action = std::dynamic_pointer_cast<MoveAction>(action_ptr);
        ASSERT_NE(nullptr, move_action);

        // Calculate distance and angle for comparison
        ball_to_friendly_robot_distance =
            (world.ball().position() - move_action->getDestination()).length();

        Line enemy_to_ball_line = Line(world.enemyTeam().getAllRobots().at(0).position(),
                                       world.ball().position());

        middle_line_to_friendly_robot_distance =
            distance(enemy_to_ball_line, move_action->getDestination());

        // assume that only 1 enemy robot is on the field for angle calculation
        enemy_to_ball_angle = (world.ball().position() -
                               world.enemyTeam().getAllRobots().front().position())
                                  .orientation();
        enemy_to_dest_angle = (move_action->getDestination() -
                               world.enemyTeam().getAllRobots().front().position())
                                  .orientation();
    }

    /**
     * The fixture function to help to create world without enemy robots and calculate
     * distances needed for unit tests
     * @param ball_position: the position of the ball in Point format
     * @param left_or_right: a enum argument in the shadowFreekickerTactic to represent
     * whether the robot is  blocking the free_kick from left or right
     */
    void CreateSceneWithoutEnemyRobot(
        Point ball_position, ShadowFreekickerTactic::FreekickShadower left_or_right)
    {
        // set up world, robot, balls
        world = ::TestUtil::createBlankTestingWorld();

        world =
            ::TestUtil::setBallPosition(world, ball_position, Timestamp::fromSeconds(0));
        world =
            ::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(0));
        Robot friendly_robot(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                             AngularVelocity::zero(), Timestamp::fromSeconds(0));
        world.updateFriendlyTeamState(Team({friendly_robot}));

        // setup tactic and acquire move action from it
        ShadowFreekickerTactic tactic = ShadowFreekickerTactic(
            left_or_right, world.enemyTeam(), world.ball(), world.field(), false);
        tactic.updateWorldParams(world);
        EXPECT_EQ(tactic.getBall().position(), world.ball().position());

        tactic.updateRobot(friendly_robot);
        auto action_ptr = tactic.getNextAction();
        ASSERT_TRUE(action_ptr);
        auto move_action = std::dynamic_pointer_cast<MoveAction>(action_ptr);
        ASSERT_NE(nullptr, move_action);

        // Calculate distance and angle for comparison
        ball_to_friendly_robot_distance =
            (world.ball().position() - move_action->getDestination()).length();

        Line ball_to_goal_line =
            Line(world.ball().position(), world.field().friendlyGoalCenter());

        middle_line_to_friendly_robot_distance =
            distance(ball_to_goal_line, move_action->getDestination());

        goal_to_ball_angle =
            (world.ball().position() - world.field().friendlyGoalCenter()).orientation();
        goal_to_dest_angle =
            (move_action->getDestination() - world.field().friendlyGoalCenter())
                .orientation();
    }

    World world = ::TestUtil::createBlankTestingWorld();

    /**
     *
     *                                      +
     *                                     + +
     *                                      +
     *
     *                                         Friendly robot
     *
     *       Enemy
     *                       ball
     *              X
     *             X X   - -   O - - - - - - - - -  - - - - - - - - - +-----+
     *              X            .          |
     *                             .
     *                             ^ .      |
     *                             |   .         Middle line to friendly distance
     *                +------------+     .  |                            0.09*1.1
     *   ball to friendly robot distance    +
     *                                     + +                        +-----+
     *                         +            +
     *                         |            | Friendly robot
     *                         |            |
     *                         +            +
     *
     *                           0.5*1.05+0.09
     */


    double ball_to_friendly_robot_distance;
    const double BALL_TO_FRIENDLY_ROBOT_DISTANCE_ABS_ERROR = 0.05;
    // To make the edge of the robot to be slightly more than 0.5m from the ball, the
    // center of the robot should be 0.62 m away from the ball
    const double EXPECTED_BALL_TO_FRIENDLY_ROBOT_DISTANCE = 0.62;

    double middle_line_to_friendly_robot_distance;
    const double MIDDLE_LINE_TO_FRIENDLY_ROBOT_DISTANCE_ABS_ERROR = 0.01;
    // The distance between robot and the middle line is 1.1 times of the robot
    // radius(0.09m)
    const double EXPECTED_MIDDLE_LINE_TO_FRIENDLY_ROBOT_DISTANCE = 0.09 * 1.1;

    // used when there are enemy robots
    // angle calculated from the vector connecting enemy robot position and ball position
    Angle enemy_to_ball_angle;
    // angle calculated from the vector connecting enemy robot position and move action
    // destination (friendly robot position)
    Angle enemy_to_dest_angle;

    // used when there is no enemy robot
    // angle calculated from the vector connecting friendly goal and ball position
    Angle goal_to_ball_angle;
    // angle calculated from the vector connecting friendly goal and move action
    // destination
    Angle goal_to_dest_angle;
};


/**
 * The two tests immediate below has the scene shown in the graph below
 *            +-----------+-------------+-----------+
 *            |           |             |           |
 *            |           |             |           |
 *            |           |             |           |
 *            |           +-------------+           |
 *            |                                     |
 *            |                                     |
 *            |                                     |
 *            |                                     |
 *            |                                     |
 *            |                                     |
 *            |                  ^                  |
 *            |                  |                  |
 *            |                  |                  |
 *      enemy |                  |                  |
 *            x   ball    <------+                  |
 *            | o                 (0,0)             |
 *            |    + friendly                       |
 *            | +                                   |
 *            |                                     |
 *            | friendly                            |
 *            |                                     |
 *            |                                     |
 *            |                                     |
 *            |                                     |
 *            |                                     |
 *            |           +-------------+           |
 *            |           |             |           |
 *            |           |             |           |
 *            |           |             |           |
 *            +-----------+-------------+-----------+
 */


TEST_F(ShadowFreekickerTacticTest,
       test_shadow_free_kicker_left_side_with_enemy_robot_at_middle_left_on_field)
{
    std::vector<Point> enemy_robot_positions = {
        Point(0, world.field().friendlyCornerPos().y())};
    Point ball_position(-0.09, world.field().friendlyCornerPos().y() - 0.09);
    ShadowFreekickerTactic::FreekickShadower left_or_right = ShadowFreekickerTactic::LEFT;

    CreateSceneWithEnemyRobot(ball_position, enemy_robot_positions, left_or_right);

    // The edge of the robot should be slightly more than 0.5m from the ball
    EXPECT_NEAR(ball_to_friendly_robot_distance, EXPECTED_BALL_TO_FRIENDLY_ROBOT_DISTANCE,
                BALL_TO_FRIENDLY_ROBOT_DISTANCE_ABS_ERROR);

    // The robot should be just to the right of the line between the enemy robot and
    // the ball (from the POV of the enemy robot)
    EXPECT_NEAR(middle_line_to_friendly_robot_distance,
                EXPECTED_MIDDLE_LINE_TO_FRIENDLY_ROBOT_DISTANCE,
                MIDDLE_LINE_TO_FRIENDLY_ROBOT_DISTANCE_ABS_ERROR);

    EXPECT_LT(enemy_to_dest_angle, enemy_to_ball_angle);
}

TEST_F(ShadowFreekickerTacticTest,
       test_shadow_free_kicker_right_side_with_enemy_robot_at_middle_left_on_field)
{
    std::vector<Point> enemy_robot_positions = {
        Point(0, world.field().friendlyCornerPos().y())};
    Point ball_position(-0.09, world.field().friendlyCornerPos().y() - 0.09);
    ShadowFreekickerTactic::FreekickShadower left_or_right =
        ShadowFreekickerTactic::RIGHT;

    CreateSceneWithEnemyRobot(ball_position, enemy_robot_positions, left_or_right);

    // The edge of the robot should be slightly more than 0.5m from the ball
    EXPECT_NEAR(ball_to_friendly_robot_distance, EXPECTED_BALL_TO_FRIENDLY_ROBOT_DISTANCE,
                BALL_TO_FRIENDLY_ROBOT_DISTANCE_ABS_ERROR);

    // The robot should be just to the left of the line between the enemy robot and
    // the ball (from the POV of the enemy robot)
    EXPECT_NEAR(middle_line_to_friendly_robot_distance,
                EXPECTED_MIDDLE_LINE_TO_FRIENDLY_ROBOT_DISTANCE,
                MIDDLE_LINE_TO_FRIENDLY_ROBOT_DISTANCE_ABS_ERROR);

    EXPECT_GT(enemy_to_dest_angle, enemy_to_ball_angle);
}


/**
 *  The two tests immediate below has the scene shown in the graph below
 *            +-----------+-------------+-----------+
 *            |           |             |           |
 *            |           |             |           |
 *            |           |             |           |
 *            |           +-------------+           |
 *            |                                     |
 *            |                                     |
 *            |                                     |
 *            |                                     |
 *            |                                     |
 *            |                                     |
 *            |                  ^                  |
 *            |                  |                  |
 *            |                  |                  |
 *            |                  |                  |   enemy
 *            |           <------+           ball   x
 *            |                   (0,0)           o |
 *            |                      friendly +     |
 *            |                                  +  |
 *            |                           friendly  |
 *            |                                     |
 *            |                                     |
 *            |                                     |
 *            |                                     |
 *            |                                     |
 *            |                                     |
 *            |           +-------------+           |
 *            |           |             |           |
 *            |           |             |           |
 *            |           |             |           |
 *            +-----------+-------------+-----------+
 */


TEST_F(ShadowFreekickerTacticTest,
       test_shadow_free_kicker_left_side_with_enemy_robot_at_middle_right_on_field)
{
    std::vector<Point> enemy_robot_positions = {
        Point(0, world.field().friendlyCornerNeg().y())};
    Point ball_position(-0.09, world.field().friendlyCornerNeg().y() + 0.09);
    ShadowFreekickerTactic::FreekickShadower left_or_right = ShadowFreekickerTactic::LEFT;

    CreateSceneWithEnemyRobot(ball_position, enemy_robot_positions, left_or_right);

    // The edge of the robot should be slightly more than 0.5m from the ball
    EXPECT_NEAR(ball_to_friendly_robot_distance, EXPECTED_BALL_TO_FRIENDLY_ROBOT_DISTANCE,
                BALL_TO_FRIENDLY_ROBOT_DISTANCE_ABS_ERROR);

    // The robot should be just to the right of the line between the enemy robot and
    // the ball (from the POV of the enemy robot)
    EXPECT_NEAR(middle_line_to_friendly_robot_distance,
                EXPECTED_MIDDLE_LINE_TO_FRIENDLY_ROBOT_DISTANCE,
                MIDDLE_LINE_TO_FRIENDLY_ROBOT_DISTANCE_ABS_ERROR);

    EXPECT_LT(enemy_to_dest_angle, enemy_to_ball_angle);
}

TEST_F(ShadowFreekickerTacticTest,
       test_shadow_free_kicker_right_side_with_enemy_robot_at_middle_right_on_field)
{
    std::vector<Point> enemy_robot_positions = {
        Point(0, world.field().friendlyCornerNeg().y())};
    Point ball_position(-0.09, world.field().friendlyCornerNeg().y() + 0.09);
    ShadowFreekickerTactic::FreekickShadower left_or_right =
        ShadowFreekickerTactic::RIGHT;

    CreateSceneWithEnemyRobot(ball_position, enemy_robot_positions, left_or_right);
    // The edge of the robot should be slightly more than 0.5m from the ball
    EXPECT_NEAR(ball_to_friendly_robot_distance, EXPECTED_BALL_TO_FRIENDLY_ROBOT_DISTANCE,
                BALL_TO_FRIENDLY_ROBOT_DISTANCE_ABS_ERROR);


    // The robot should be just to the left of the line between the enemy robot and
    // the ball (from the POV of the enemy robot)
    EXPECT_NEAR(middle_line_to_friendly_robot_distance,
                EXPECTED_MIDDLE_LINE_TO_FRIENDLY_ROBOT_DISTANCE,
                MIDDLE_LINE_TO_FRIENDLY_ROBOT_DISTANCE_ABS_ERROR);

    EXPECT_GT(enemy_to_dest_angle, enemy_to_ball_angle);
}

/**
 *  The two tests immediate below has the scene shown in the graph below
 *
 *
 *           enemy
 *
 *            x-----------+-------------+-----------+
 *            |           |             |           |
 *            | o ball    |             |           |
 *            |           |             |           |
 *            |    +      +-------------+           |
 *            | +    friendly                       |
 *            |                                     |
 *            | friendly                            |
 *            |                                     |
 *            |                                     |
 *            |                                     |
 *            |                  ^                  |
 *            |                  |                  |
 *            |                  |                  |
 *            |                  |                  |
 *            |           <------+                  |
 *            |                   (0,0)             |
 *            |                                     |
 *            |                                     |
 *            |                                     |
 *            |                                     |
 *            |                                     |
 *            |                                     |
 *            |                                     |
 *            |                                     |
 *            |                                     |
 *            |           +-------------+           |
 *            |           |             |           |
 *            |           |             |           |
 *            |           |             |           |
 *            +-----------+-------------+-----------+
 */


TEST_F(ShadowFreekickerTacticTest,
       test_shadow_free_kicker_left_side_with_enemy_robot_at_top_left_on_field)
{
    std::vector<Point> enemy_robot_positions = {Point(
        world.field().friendlyCornerPos().x(), world.field().friendlyCornerPos().y())};
    Point ball_position(world.field().friendlyCornerPos().x() - 0.09,
                        world.field().friendlyCornerPos().y() - 0.09);
    ShadowFreekickerTactic::FreekickShadower left_or_right = ShadowFreekickerTactic::LEFT;

    CreateSceneWithEnemyRobot(ball_position, enemy_robot_positions, left_or_right);

    // The edge of the robot should be slightly more than 0.5m from the ball
    EXPECT_NEAR(ball_to_friendly_robot_distance, EXPECTED_BALL_TO_FRIENDLY_ROBOT_DISTANCE,
                BALL_TO_FRIENDLY_ROBOT_DISTANCE_ABS_ERROR);


    // The robot should be just to the right of the line between the enemy robot and
    // the ball (from the POV of the enemy robot)
    EXPECT_NEAR(middle_line_to_friendly_robot_distance,
                EXPECTED_MIDDLE_LINE_TO_FRIENDLY_ROBOT_DISTANCE,
                MIDDLE_LINE_TO_FRIENDLY_ROBOT_DISTANCE_ABS_ERROR);

    EXPECT_LT(enemy_to_dest_angle, enemy_to_ball_angle);
}



TEST_F(ShadowFreekickerTacticTest,
       test_shadow_free_kicker_right_side_with_enemy_robot_at_top_left_on_field)
{
    std::vector<Point> enemy_robot_positions = {Point(
        world.field().friendlyCornerPos().x(), world.field().friendlyCornerPos().y())};
    Point ball_position(world.field().friendlyCornerPos().x() - 0.09,
                        world.field().friendlyCornerPos().y() - 0.09);
    ShadowFreekickerTactic::FreekickShadower left_or_right =
        ShadowFreekickerTactic::RIGHT;

    CreateSceneWithEnemyRobot(ball_position, enemy_robot_positions, left_or_right);

    // The edge of the robot should be slightly more than 0.5m from the ball
    EXPECT_NEAR(ball_to_friendly_robot_distance, EXPECTED_BALL_TO_FRIENDLY_ROBOT_DISTANCE,
                BALL_TO_FRIENDLY_ROBOT_DISTANCE_ABS_ERROR);

    // The robot should be just to the left of the line between the enemy robot and
    // the ball (from the POV of the enemy robot)
    EXPECT_NEAR(middle_line_to_friendly_robot_distance,
                EXPECTED_MIDDLE_LINE_TO_FRIENDLY_ROBOT_DISTANCE,
                MIDDLE_LINE_TO_FRIENDLY_ROBOT_DISTANCE_ABS_ERROR);

    EXPECT_GT(enemy_to_dest_angle, enemy_to_ball_angle);
}


/**
 *  The two tests immediate below has the scene shown in the graph below
 *
 *
 *                                              enemy
 *
 *            +-----------+-------------+-----------x
 *            |           |             |           |
 *            |           |             |    ball o |
 *            |           |             |           |
 *            |           +-------------+    +      |
 *            |                      friendly     + |
 *            |                                     |
 *            |                           friendly  |
 *            |                                     |
 *            |                                     |
 *            |                                     |
 *            |                  ^                  |
 *            |                  |                  |
 *            |                  |                  |
 *            |                  |                  |
 *            |           <------+                  |
 *            |                   (0,0)             |
 *            |                                     |
 *            |                                     |
 *            |                                     |
 *            |                                     |
 *            |                                     |
 *            |                                     |
 *            |                                     |
 *            |                                     |
 *            |                                     |
 *            |           +-------------+           |
 *            |           |             |           |
 *            |           |             |           |
 *            |           |             |           |
 *            +-----------+-------------+-----------+
 */


TEST_F(ShadowFreekickerTacticTest,
       test_shadow_free_kicker_left_side_with_enemy_robot_at_top_right_on_field)
{
    std::vector<Point> enemy_robot_positions = {Point(
        world.field().friendlyCornerNeg().x(), world.field().friendlyCornerPos().y())};
    Point ball_position(world.field().friendlyCornerNeg().x() + 0.09,
                        world.field().friendlyCornerPos().y() - 0.09);
    ShadowFreekickerTactic::FreekickShadower left_or_right = ShadowFreekickerTactic::LEFT;

    CreateSceneWithEnemyRobot(ball_position, enemy_robot_positions, left_or_right);

    // The edge of the robot should be slightly more than 0.5m from the ball
    EXPECT_NEAR(ball_to_friendly_robot_distance, EXPECTED_BALL_TO_FRIENDLY_ROBOT_DISTANCE,
                BALL_TO_FRIENDLY_ROBOT_DISTANCE_ABS_ERROR);


    // The robot should be just to the right of the line between the enemy robot and
    // the ball (from the POV of the enemy robot)
    EXPECT_NEAR(middle_line_to_friendly_robot_distance,
                EXPECTED_MIDDLE_LINE_TO_FRIENDLY_ROBOT_DISTANCE,
                MIDDLE_LINE_TO_FRIENDLY_ROBOT_DISTANCE_ABS_ERROR);

    EXPECT_LT(enemy_to_dest_angle, enemy_to_ball_angle);
}



TEST_F(ShadowFreekickerTacticTest,
       test_shadow_free_kicker_right_side_with_enemy_robot_at_top_right_on_field)
{
    std::vector<Point> enemy_robot_positions = {Point(
        world.field().friendlyCornerNeg().x(), world.field().friendlyCornerPos().y())};
    Point ball_position(world.field().friendlyCornerNeg().x() + 0.09,
                        world.field().friendlyCornerPos().y() - 0.09);
    ShadowFreekickerTactic::FreekickShadower left_or_right =
        ShadowFreekickerTactic::RIGHT;

    CreateSceneWithEnemyRobot(ball_position, enemy_robot_positions, left_or_right);
    // The edge of the robot should be slightly more than 0.5m from the ball
    EXPECT_NEAR(ball_to_friendly_robot_distance, EXPECTED_BALL_TO_FRIENDLY_ROBOT_DISTANCE,
                BALL_TO_FRIENDLY_ROBOT_DISTANCE_ABS_ERROR);

    // The robot should be just to the left of the line between the enemy robot and
    // the ball (from the POV of the enemy robot)
    EXPECT_NEAR(middle_line_to_friendly_robot_distance,
                EXPECTED_MIDDLE_LINE_TO_FRIENDLY_ROBOT_DISTANCE,
                MIDDLE_LINE_TO_FRIENDLY_ROBOT_DISTANCE_ABS_ERROR);

    EXPECT_GT(enemy_to_dest_angle, enemy_to_ball_angle);
}


/**
 *  The two tests immediate below has the scene shown in the graph below
 *
 *            +-----------+-------------+-----------+
 *            |           |             |           |
 *            |           |             |           |
 *            |           |             |           |
 *            |           +-------------+           |
 *            |                                     |
 *            |                                     |
 *            |                                     |
 *            |                                     |
 *            |                                     |
 *            |                                     |
 *            |                  ^                  |
 *            |                  |                  |
 *            |                  |                  |
 *            |                  |                  |
 *            |           <------+                  |
 *            |                   (0,0)             |
 *            |                                     |
 *            |                                     |
 *            |                                     |
 *            |                                     |
 *            |                                     |
 *            |                                     |
 *            |                                     |
 *            | friendly                            |
 *            |  +    friendly                      |
 *            |      +    +-------------+           |
 *            |           |             |           |
 *            |  o  ball  |             |           |
 *            |           |             |           |
 *    enemy   x-----------+-------------+-----------+
 */


TEST_F(ShadowFreekickerTacticTest,
       test_shadow_free_kicker_left_side_with_enemy_robot_at_bottom_left_on_field)
{
    std::vector<Point> enemy_robot_positions = {Point(
        world.field().friendlyCornerPos().x(), world.field().friendlyCornerNeg().y())};
    Point ball_position(world.field().friendlyCornerPos().x() - 0.09,
                        world.field().friendlyCornerNeg().y() + 0.09);
    ShadowFreekickerTactic::FreekickShadower left_or_right = ShadowFreekickerTactic::LEFT;

    CreateSceneWithEnemyRobot(ball_position, enemy_robot_positions, left_or_right);

    // The edge of the robot should be slightly more than 0.5m from the ball
    EXPECT_NEAR(ball_to_friendly_robot_distance, EXPECTED_BALL_TO_FRIENDLY_ROBOT_DISTANCE,
                BALL_TO_FRIENDLY_ROBOT_DISTANCE_ABS_ERROR);

    // The robot should be just to the right of the line between the enemy robot and
    // the ball (from the POV of the enemy robot)
    EXPECT_NEAR(middle_line_to_friendly_robot_distance,
                EXPECTED_MIDDLE_LINE_TO_FRIENDLY_ROBOT_DISTANCE,
                MIDDLE_LINE_TO_FRIENDLY_ROBOT_DISTANCE_ABS_ERROR);

    EXPECT_LT(enemy_to_dest_angle, enemy_to_ball_angle);
}



TEST_F(ShadowFreekickerTacticTest,
       test_shadow_free_kicker_right_side_with_enemy_robot_at_bottom_left_on_field)
{
    std::vector<Point> enemy_robot_positions = {Point(
        world.field().friendlyCornerPos().x(), world.field().friendlyCornerNeg().y())};
    Point ball_position(world.field().friendlyCornerPos().x() - 0.09,
                        world.field().friendlyCornerNeg().y() + 0.09);
    ShadowFreekickerTactic::FreekickShadower left_or_right =
        ShadowFreekickerTactic::RIGHT;

    CreateSceneWithEnemyRobot(ball_position, enemy_robot_positions, left_or_right);

    // The edge of the robot should be slightly more than 0.5m from the ball
    EXPECT_NEAR(ball_to_friendly_robot_distance, EXPECTED_BALL_TO_FRIENDLY_ROBOT_DISTANCE,
                BALL_TO_FRIENDLY_ROBOT_DISTANCE_ABS_ERROR);

    // The robot should be just to the left of the line between the enemy robot and
    // the ball (from the POV of the enemy robot)
    EXPECT_NEAR(middle_line_to_friendly_robot_distance,
                EXPECTED_MIDDLE_LINE_TO_FRIENDLY_ROBOT_DISTANCE,
                MIDDLE_LINE_TO_FRIENDLY_ROBOT_DISTANCE_ABS_ERROR);

    EXPECT_GT(enemy_to_dest_angle, enemy_to_ball_angle);
}


/**
 *  The two tests immediate below has the scene shown in the graph below
 *
 *
 *            +-----------+-------------+-----------+
 *            |           |             |           |
 *            |           |             |           |
 *            |           |             |           |
 *            |           +-------------+           |
 *            |                                     |
 *            |                                     |
 *            |                                     |
 *            |                                     |
 *            |                                     |
 *            |                                     |
 *            |                  ^                  |
 *            |                  |                  |
 *            |                  |                  |
 *            |                  |                  |
 *            |           <------+                  |
 *            |                   (0,0)             |
 *            |                                     |
 *            |                                     |
 *            |                                     |
 *            |                                     |
 *            |                                     |
 *            |                                     |
 *            |                                     |
 *            |                           friendly  |
 *            |                      friendly   +   |
 *            |           +-------------+   +       |
 *            |           |             |           |
 *            |           |             | ball  o   |
 *            |           |             |           |
 *            +-----------+-------------+-----------x enemy
 */


TEST_F(ShadowFreekickerTacticTest,
       test_shadow_free_kicker_left_side_with_enemy_robot_at_botom_right_on_field)
{
    std::vector<Point> enemy_robot_positions = {Point(
        world.field().friendlyCornerNeg().x(), world.field().friendlyCornerNeg().y())};
    Point ball_position(world.field().friendlyCornerNeg().x() + 0.09,
                        world.field().friendlyCornerNeg().y() + 0.09);
    ShadowFreekickerTactic::FreekickShadower left_or_right = ShadowFreekickerTactic::LEFT;

    CreateSceneWithEnemyRobot(ball_position, enemy_robot_positions, left_or_right);

    // The edge of the robot should be slightly more than 0.5m from the ball
    EXPECT_NEAR(ball_to_friendly_robot_distance, EXPECTED_BALL_TO_FRIENDLY_ROBOT_DISTANCE,
                BALL_TO_FRIENDLY_ROBOT_DISTANCE_ABS_ERROR);

    // The robot should be just to the right of the line between the enemy robot and
    // the ball (from the POV of the enemy robot)
    EXPECT_NEAR(middle_line_to_friendly_robot_distance,
                EXPECTED_MIDDLE_LINE_TO_FRIENDLY_ROBOT_DISTANCE,
                MIDDLE_LINE_TO_FRIENDLY_ROBOT_DISTANCE_ABS_ERROR);

    EXPECT_LT(enemy_to_dest_angle, enemy_to_ball_angle);
}

TEST_F(ShadowFreekickerTacticTest,
       test_shadow_free_kicker_right_side_with_enemy_robot_at_bottom_right_on_field)
{
    std::vector<Point> enemy_robot_positions = {Point(
        world.field().friendlyCornerNeg().x(), world.field().friendlyCornerNeg().y())};
    Point ball_position(world.field().friendlyCornerNeg().x() + 0.09,
                        world.field().friendlyCornerNeg().y() + 0.09);
    ShadowFreekickerTactic::FreekickShadower left_or_right =
        ShadowFreekickerTactic::RIGHT;

    CreateSceneWithEnemyRobot(ball_position, enemy_robot_positions, left_or_right);

    // The edge of the robot should be slightly more than 0.5m from the ball
    EXPECT_NEAR(ball_to_friendly_robot_distance, EXPECTED_BALL_TO_FRIENDLY_ROBOT_DISTANCE,
                BALL_TO_FRIENDLY_ROBOT_DISTANCE_ABS_ERROR);

    // The robot should be just to the left of the line between the enemy robot and
    // the ball (from the POV of the enemy robot)
    EXPECT_NEAR(middle_line_to_friendly_robot_distance,
                EXPECTED_MIDDLE_LINE_TO_FRIENDLY_ROBOT_DISTANCE,
                MIDDLE_LINE_TO_FRIENDLY_ROBOT_DISTANCE_ABS_ERROR);

    EXPECT_GT(enemy_to_dest_angle, enemy_to_ball_angle);
}


/**
 *  The two tests immediate below has the scene shown in the graph below
 *                                                                                                                                                     X
 *                                                                                                                                                    X+X-------
 *            +-----------+-------------+-----------+
 *            |           |             |           |
 *            |           |             |           |
 *            |           |             |           |
 *            |           +-------------+           |
 *            |                                     |
 *            |                                     |
 *            |                                     |
 *            |                                     |
 *            |                                     |
 *            |                                     |
 *            |                  ^                  |
 *            |                  |                  |
 *            |                  |                  |
 *            |             (0,0)| enemy            |
 *            |           <------x                  |
 *            |                                     |
 *            |            ball  o                  |
 *            |                                     |
 *            |                +   +                |
 *            |         friendly    friendly        |
 *            |                                     |
 *            |                                     |
 *            |                                     |
 *            |                                     |
 *            |                                     |
 *            |           +-------------+           |
 *            |           |             |           |
 *            |           |             |           |
 *            |           |             |           |
 *            +-----------+-------------+-----------+
 */


TEST_F(ShadowFreekickerTacticTest,
       test_shadow_free_kicker_left_side_with_enemy_robot_at_origin_on_field)
{
    std::vector<Point> enemy_robot_positions = {Point(0, 0)};
    Point ball_position(0, -0.09);
    ShadowFreekickerTactic::FreekickShadower left_or_right = ShadowFreekickerTactic::LEFT;

    CreateSceneWithEnemyRobot(ball_position, enemy_robot_positions, left_or_right);

    // The edge of the robot should be slightly more than 0.5m from the ball
    EXPECT_NEAR(ball_to_friendly_robot_distance, EXPECTED_BALL_TO_FRIENDLY_ROBOT_DISTANCE,
                BALL_TO_FRIENDLY_ROBOT_DISTANCE_ABS_ERROR);

    // The robot should be just to the right of the line between the enemy robot and
    // the ball (from the POV of the enemy robot)
    EXPECT_NEAR(middle_line_to_friendly_robot_distance,
                EXPECTED_MIDDLE_LINE_TO_FRIENDLY_ROBOT_DISTANCE,
                MIDDLE_LINE_TO_FRIENDLY_ROBOT_DISTANCE_ABS_ERROR);

    EXPECT_LT(enemy_to_dest_angle, enemy_to_ball_angle);
}

TEST_F(ShadowFreekickerTacticTest,
       test_shadow_free_kicker_right_side_with_enemy_robot_at_origin_on_field)
{
    std::vector<Point> enemy_robot_positions = {Point(0, 0)};
    Point ball_position(0, -0.09);
    ShadowFreekickerTactic::FreekickShadower left_or_right =
        ShadowFreekickerTactic::RIGHT;

    CreateSceneWithEnemyRobot(ball_position, enemy_robot_positions, left_or_right);

    // The edge of the robot should be slightly more than 0.5m from the ball
    EXPECT_NEAR(ball_to_friendly_robot_distance, EXPECTED_BALL_TO_FRIENDLY_ROBOT_DISTANCE,
                BALL_TO_FRIENDLY_ROBOT_DISTANCE_ABS_ERROR);

    // The robot should be just to the left of the line between the enemy robot and
    // the ball (from the POV of the enemy robot)
    EXPECT_NEAR(middle_line_to_friendly_robot_distance,
                EXPECTED_MIDDLE_LINE_TO_FRIENDLY_ROBOT_DISTANCE,
                MIDDLE_LINE_TO_FRIENDLY_ROBOT_DISTANCE_ABS_ERROR);

    EXPECT_GT(enemy_to_dest_angle, enemy_to_ball_angle);
}

/**
 *  The two tests immediate below has the scene shown in the graph below
 *
 *
 *            +-----------+-------------+-----------+
 *            |           |             |           |
 *            |           |             |           |
 *            |           |             |           |
 *            |           +-------------+           |
 *            |                                     |
 *            |                                     |
 *            |                                     |
 *            |                                     |
 *            |                                     |
 *            |                                     |
 *            |                  ^                  |
 *            |                  |                  |
 *            |                  |                  |
 *            |             (0,0)|                  |
 *            |           <------+                  |
 *            |                                     |
 *            |            ball  o                  |
 *            |                                     |
 *            |                +   +                |
 *            |         friendly    friendly        |
 *            |                                     |
 *            |                                     |
 *            |                                     |
 *            |                                     |
 *            |                                     |
 *            |           +-------------+           |
 *            |           |             |           |
 *            |           |             |           |
 *            |           |             |           |
 *            +-----------+-------------+-----------+
 */

TEST_F(ShadowFreekickerTacticTest,
       test_shadow_free_kicker_left_side_without_robot_in_enemy_team)
{
    Point ball_position(-0.09, world.field().friendlyCornerPos().y() - 0.09);
    ShadowFreekickerTactic::FreekickShadower left_or_right = ShadowFreekickerTactic::LEFT;

    CreateSceneWithoutEnemyRobot(ball_position, left_or_right);

    // The edge of the robot should be slightly more than 0.5m from the ball
    EXPECT_NEAR(ball_to_friendly_robot_distance, EXPECTED_BALL_TO_FRIENDLY_ROBOT_DISTANCE,
                BALL_TO_FRIENDLY_ROBOT_DISTANCE_ABS_ERROR);

    // The robot should be just to the left of the line between the friendly net and
    // the ball (from the POV of the friendly net)
    EXPECT_NEAR(middle_line_to_friendly_robot_distance,
                EXPECTED_MIDDLE_LINE_TO_FRIENDLY_ROBOT_DISTANCE,
                MIDDLE_LINE_TO_FRIENDLY_ROBOT_DISTANCE_ABS_ERROR);

    EXPECT_GT(goal_to_dest_angle, goal_to_ball_angle);
}

TEST_F(ShadowFreekickerTacticTest,
       test_shadow_free_kicker_right_side_without_robot_in_enemy_team)
{
    Point ball_position(-0.09, world.field().friendlyCornerPos().y() - 0.09);
    ShadowFreekickerTactic::FreekickShadower left_or_right =
        ShadowFreekickerTactic::RIGHT;

    CreateSceneWithoutEnemyRobot(ball_position, left_or_right);

    // The edge of the robot should be slightly more than 0.5m from the ball
    EXPECT_NEAR(ball_to_friendly_robot_distance, EXPECTED_BALL_TO_FRIENDLY_ROBOT_DISTANCE,
                BALL_TO_FRIENDLY_ROBOT_DISTANCE_ABS_ERROR);

    // The robot should be just to the right of the line between the friendly net and
    // the ball (from the POV of the friendly net)
    EXPECT_NEAR(middle_line_to_friendly_robot_distance,
                EXPECTED_MIDDLE_LINE_TO_FRIENDLY_ROBOT_DISTANCE,
                MIDDLE_LINE_TO_FRIENDLY_ROBOT_DISTANCE_ABS_ERROR);

    EXPECT_LT(goal_to_dest_angle, goal_to_ball_angle);
}
