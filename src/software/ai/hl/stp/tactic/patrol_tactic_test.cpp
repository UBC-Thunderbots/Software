#include "software/ai/hl/stp/tactic/patrol_tactic.h"

#include <gtest/gtest.h>

#include "software/ai/hl/stp/action/move_action.h"
#include "software/test_util/test_util.h"

class PatrolTacticTest : public testing::Test
{
   protected:
    std::vector<TimestampedRobotState> robot_state_to_complete_actions;
    std::vector<std::shared_ptr<MoveAction>> expected_actions;
    const double COST_OF_ASSIGNED_ROBOT                      = 0.0;
    const double COST_OF_NONASSIGNED_ROBOT_UNASSIGNED_TACTIC = 1.0;

    /**
     * populates the expected_actions test variable with move_actions in the order of
     * points in patrol_points vector
     * @param robot: expected robot that should be assigned to action
     * @param patrol_points: points that the robot visits in the patrol tactic
     * @param angle: expected angle of action
     * @param speed_at_patrol_points : expected speed of action
     */
    void generateExpectedActions(Robot robot, std::vector<Point> patrol_points,
                                 Angle angle, double speed_at_patrol_points)
    {
        for (size_t i = 0; i < patrol_points.size(); i++)
        {
            auto expected_action = std::make_shared<MoveAction>(false);
            expected_action->updateControlParams(
                robot, patrol_points[i], angle, speed_at_patrol_points,
                DribblerEnable::OFF, MoveType::NORMAL, AutokickType::NONE,
                BallCollisionType::AVOID);
            expected_actions.push_back(expected_action);
        }
    }

    /**
     * populates the robot_state_to_complete_actions test variable in the order of points
     * in patrol_points
     * @param patrol_points: points that the robot visits in the patrol tactic
     * @param velocity : desired velocity of the robot after action in new state
     */
    void generateRobotStatesToCompleteAction(std::vector<Point> patrol_points,
                                             Vector velocity)
    {
        for (size_t i = 0; i < patrol_points.size(); i++)
        {
            TimestampedRobotState robotState = TimestampedRobotState(
                patrol_points[i], velocity, Angle::zero(), AngularVelocity::zero(),
                Timestamp::fromSeconds(static_cast<double>(std::time(nullptr))));
            robot_state_to_complete_actions.push_back(robotState);
        }
    }

    /**
     * Simulates the running of an action until it is done, allowing the patrol tactic
     * to move on to the next point
     * @param robot: robot that is assigned the tactic
     * @param new_robot_state: A state that satisfies the patrol tactics's conditions of
     * moving on to the next point
     * @param action_ptr: the last action returned by the tactic
     * @param tactic: the patrol tactic
     */
    void compareMoveActions(std::shared_ptr<MoveAction> expected_move_action,
                            std::shared_ptr<MoveAction> move_action)
    {
        EXPECT_EQ(expected_move_action->getDestination(), move_action->getDestination());
        EXPECT_EQ(expected_move_action->getFinalOrientation(),
                  move_action->getFinalOrientation());
        EXPECT_EQ(expected_move_action->getFinalSpeed(), move_action->getFinalSpeed());
        EXPECT_EQ(expected_move_action->getAutoKickType(),
                  move_action->getAutoKickType());
        EXPECT_EQ(expected_move_action->getDribblerEnabled(),
                  move_action->getDribblerEnabled());
    }

    /**
     * Simulates the running of an action until it is done, allowing the patrol tactic
     * to move on to the next point
     * @param robot: robot that is assigned the tactic
     * @param new_robot_state: A state that satisfies the patrol tactics's conditions of
     * moving on to the next point
     * @param action_ptr: the last action returned by the tactic
     * @param tactic: the patrol tactic
     */
    void simulateActionToCompletion(Robot &robot, TimestampedRobotState new_robot_state,
                                    std::shared_ptr<Action> action_ptr,
                                    PatrolTactic &tactic)
    {
        // actions need to yield at least once before being able to complete
        std::unique_ptr<Intent> intent = action_ptr->getNextIntent();

        // update state of the robot and tactic
        robot.updateState(new_robot_state);
        tactic.updateRobot(robot);

        // complete the action
        // tactic will update the action with latest robot status
        action_ptr = tactic.getNextAction();

        // Now that the action has the latest robot status, it is able to set the action
        // to complete
        intent = action_ptr->getNextIntent();
        ASSERT_TRUE(action_ptr->done());
    }
};

TEST_F(PatrolTacticTest, patrol_tactic_constructor)
{
    Point patrol_point1              = Point(3, 5);
    double at_patrol_point_tolerance = 1.0;
    double speed_at_patrol_points    = 1.0;

    PatrolTactic tactic =
        PatrolTactic(std::vector<Point>({patrol_point1}), at_patrol_point_tolerance,
                     Angle::zero(), speed_at_patrol_points);
    ASSERT_NE(nullptr, &tactic);
    ASSERT_EQ("Patrol Tactic", tactic.getName());
}

TEST_F(PatrolTacticTest, patrol_one_point)
{
    // Setup
    int robot_id               = 0;
    Point robot_starting_point = Point(0, 0);
    Vector initial_velocity    = Vector(1, 0);

    Point patrol_point1              = Point(3, 5);
    double at_patrol_point_tolerance = 1.0;
    double speed_at_patrol_points    = 1.0;

    Robot robot = Robot(robot_id, robot_starting_point, initial_velocity, Angle::zero(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));

    PatrolTactic tactic =
        PatrolTactic(std::vector<Point>({patrol_point1}), at_patrol_point_tolerance,
                     Angle::zero(), speed_at_patrol_points);

    tactic.updateRobot(robot);
    generateExpectedActions(robot, std::vector<Point>({patrol_point1}), Angle::zero(),
                            speed_at_patrol_points);

    // Act
    std::shared_ptr<Action> action_ptr = tactic.getNextAction();
    ASSERT_NE(nullptr, action_ptr);
    std::shared_ptr<MoveAction> move_action =
        std::dynamic_pointer_cast<MoveAction>(action_ptr);

    // Assert
    compareMoveActions(expected_actions[0], move_action);
}

TEST_F(PatrolTacticTest, patrol_three_points)
{
    // setup
    int robot_id               = 0;
    Point robot_starting_point = Point(3, 3);
    Vector initial_velocity    = Vector(3, 1);

    Point patrol_point1 = Point(3, -3);
    Point patrol_point2 = Point(-3, -3);
    Point patrol_point3 = Point(3, 3);
    std::vector<Point> patrol_points{patrol_point1, patrol_point2, patrol_point3};

    double at_patrol_point_tolerance = 1.0;
    double speed_at_patrol_points    = 1.0;

    Robot robot = Robot(robot_id, robot_starting_point, initial_velocity, Angle::zero(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));

    PatrolTactic tactic = PatrolTactic(patrol_points, at_patrol_point_tolerance,
                                       Angle::zero(), speed_at_patrol_points);

    tactic.updateRobot(robot);

    generateExpectedActions(robot, patrol_points, Angle::zero(), speed_at_patrol_points);
    generateRobotStatesToCompleteAction(patrol_points, initial_velocity);

    // Act and Assert

    for (size_t i = 0; i < patrol_points.size(); i++)
    {
        std::shared_ptr<Action> action_ptr = tactic.getNextAction();
        ASSERT_NE(nullptr, action_ptr);

        std::shared_ptr<MoveAction> move_action =
            std::dynamic_pointer_cast<MoveAction>(action_ptr);
        compareMoveActions(expected_actions[i], move_action);

        simulateActionToCompletion(robot, robot_state_to_complete_actions[i], action_ptr,
                                   tactic);
    }
}

TEST_F(PatrolTacticTest, patrol_two_points_and_restart)
{
    // setup
    int robot_id               = 0;
    Point robot_starting_point = Point(-9, 5);
    Vector initial_velocity    = Vector(0, 0);

    Point patrol_point1 = Point(7, 11);
    Point patrol_point2 = Point(6, 9);
    std::vector<Point> patrol_points{patrol_point1, patrol_point2};

    double at_patrol_point_tolerance = 0.5;
    double speed_at_patrol_points    = 2.0;

    Robot robot = Robot(robot_id, robot_starting_point, initial_velocity, Angle::zero(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));

    PatrolTactic tactic = PatrolTactic(patrol_points, at_patrol_point_tolerance,
                                       Angle::zero(), speed_at_patrol_points);

    tactic.updateRobot(robot);

    generateExpectedActions(robot, patrol_points, Angle::zero(), speed_at_patrol_points);
    generateRobotStatesToCompleteAction(patrol_points, initial_velocity);

    // Act and Assert

    for (int j = 0; j < 2; j++)
    {
        for (size_t i = 0; i < patrol_points.size(); i++)
        {
            std::shared_ptr<Action> action_ptr = tactic.getNextAction();
            ASSERT_NE(nullptr, action_ptr);

            std::shared_ptr<MoveAction> move_action =
                std::dynamic_pointer_cast<MoveAction>(action_ptr);
            compareMoveActions(expected_actions[i], move_action);

            simulateActionToCompletion(robot, robot_state_to_complete_actions[i],
                                       action_ptr, tactic);
        }
    }
}

TEST_F(PatrolTacticTest, cost_of_non_assigned_robot_when_tactic_is_assigned)
{
    // setup
    Point robot_starting_point = Point(1, 1);
    Vector initial_velocity    = Vector(0, 1);

    Robot assigned_robot = Robot(0, robot_starting_point, initial_velocity, Angle::zero(),
                                 AngularVelocity::zero(), Timestamp::fromSeconds(0));

    Robot non_assigned_robot =
        Robot(1, robot_starting_point, initial_velocity, Angle::zero(),
              AngularVelocity::zero(), Timestamp::fromSeconds(0));

    std::vector<Point> patrol_points{Point(2, 3), Point(4, 3)};
    World world = ::TestUtil::createBlankTestingWorld();

    PatrolTactic tactic = PatrolTactic(patrol_points, 1.0, Angle::zero(), 2.0);

    tactic.updateRobot(assigned_robot);

    double cost = tactic.calculateRobotCost(non_assigned_robot, world);

    EXPECT_EQ(COST_OF_NONASSIGNED_ROBOT_UNASSIGNED_TACTIC, cost);
}

TEST_F(PatrolTacticTest, cost_of_non_assigned_robot_when_tactic_is_not_assigned)
{
    // setup
    Point robot_starting_point = Point(1, 1);
    Vector initial_velocity    = Vector(0, 1);

    Robot non_assigned_robot =
        Robot(1, robot_starting_point, initial_velocity, Angle::zero(),
              AngularVelocity::zero(), Timestamp::fromSeconds(0));

    std::vector<Point> patrol_points{Point(2, 2), Point(4, 3)};
    World world = ::TestUtil::createBlankTestingWorld();

    PatrolTactic tactic = PatrolTactic(patrol_points, 1.0, Angle::zero(), 2.0);

    double cost = tactic.calculateRobotCost(non_assigned_robot, world);

    EXPECT_NEAR(0.15, cost, 0.05);
}

TEST_F(PatrolTacticTest, cost_of_already_assigned_robot)
{
    // setup
    int robot_id               = 0;
    Point robot_starting_point = Point(1, 1);
    Vector initial_velocity    = Vector(0, 1);

    Robot assigned_robot =
        Robot(robot_id, robot_starting_point, initial_velocity, Angle::zero(),
              AngularVelocity::zero(), Timestamp::fromSeconds(0));

    std::vector<Point> patrol_points{Point(2, 3), Point(4, 3)};
    World world = ::TestUtil::createBlankTestingWorld();

    PatrolTactic tactic = PatrolTactic(patrol_points, 1.0, Angle::zero(), 2.0);

    tactic.updateRobot(assigned_robot);

    double cost = tactic.calculateRobotCost(assigned_robot, world);

    EXPECT_EQ(COST_OF_ASSIGNED_ROBOT, cost);
}
