#include "software/ai/hl/stp/tactic/patrol_tactic.h"

#include <gtest/gtest.h>

#include "software/ai/hl/stp/action/move_action.h"
#include "software/test_util/test_util.h"

class PatrolTacticTest : public testing::Test
{
   protected:
    std::vector<RobotState> robotStateToCompleteActions;
    std::vector<std::shared_ptr<MoveAction>> expectedActions;
    const double COST_OF_ASSIGNED_ROBOT                      = 0.0;
    const double COST_OF_NONASSIGNED_ROBOT_UNASSIGNED_TACTIC = 1.0;

    /*
     * populates the expectedActions test variable in the order of the patrol points
     */
    void GenerateExpectedActions(Robot robot, std::vector<Point> patrolPoints,
                                 Angle angle, double speedAtPatrolPoints)
    {
        for (int i = 0; i < patrolPoints.size(); i++)
        {
            auto expected_action = std::make_shared<MoveAction>(false);
            expected_action->updateControlParams(
                robot, patrolPoints[i], angle, speedAtPatrolPoints, DribblerEnable::OFF,
                MoveType::NORMAL, AutokickType::NONE, BallCollisionType::AVOID);
            expectedActions.push_back(expected_action);
        }
    }

    /*
     *  populates the robotStateToCompleteActions test variable in the order of patrol
     * points
     */
    void GenerateRobotStatesToCompleteAction(std::vector<Point> patrolPoints,
                                             Vector velocity)
    {
        for (int i = 0; i < patrolPoints.size(); i++)
        {
            RobotState robotState = RobotState(
                patrolPoints[i], velocity, Angle::zero(), AngularVelocity::zero(),
                Timestamp::fromSeconds(std::time(nullptr)));
            robotStateToCompleteActions.push_back(robotState);
        }
    }

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
     * @param newRobotState: A state that satisfies the patrol tactics's conditions of
     * moving on on to the next point
     * @param action_ptr: the last action returned by the tactic
     * @param tactic: the patrol tactic
     */
    void simulateActionToCompletion(Robot &robot, RobotState newRobotState,
                                    std::shared_ptr<Action> action_ptr,
                                    PatrolTactic &tactic)
    {
        // actions need to yield at least once before being able to complete
        std::unique_ptr<Intent> intent = action_ptr->getNextIntent();

        // update state of the robot and tactic
        robot.updateState(newRobotState);
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
    Point patrolPoint1            = Point(3, 5);
    double atPatrolPointTolerance = 1.0;
    double speedAtPatrolPoints    = 1.0;

    PatrolTactic tactic =
        PatrolTactic(std::vector<Point>({patrolPoint1}), atPatrolPointTolerance,
                     Angle::zero(), speedAtPatrolPoints);
    ASSERT_NE(nullptr, &tactic);
    ASSERT_EQ("Patrol Tactic", tactic.getName());
}

TEST_F(PatrolTacticTest, patrol_one_point)
{
    // Setup
    int robotId              = 0;
    Point robotStartingPoint = Point(0, 0);
    Vector initialVelocity   = Vector(1, 0);

    Point patrolPoint1            = Point(3, 5);
    double atPatrolPointTolerance = 1.0;
    double speedAtPatrolPoints    = 1.0;

    Robot robot = Robot(robotId, robotStartingPoint, initialVelocity, Angle::zero(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));

    PatrolTactic tactic =
        PatrolTactic(std::vector<Point>({patrolPoint1}), atPatrolPointTolerance,
                     Angle::zero(), speedAtPatrolPoints);

    tactic.updateRobot(robot);
    GenerateExpectedActions(robot, std::vector<Point>({patrolPoint1}), Angle::zero(),
                            speedAtPatrolPoints);

    // Act
    std::shared_ptr<Action> action_ptr = tactic.getNextAction();
    ASSERT_NE(nullptr, action_ptr);
    std::shared_ptr<MoveAction> move_action =
        std::dynamic_pointer_cast<MoveAction>(action_ptr);

    // Assert
    compareMoveActions(expectedActions[0], move_action);
}

TEST_F(PatrolTacticTest, patrol_three_points)
{
    // setup
    int robotId              = 0;
    Point robotStartingPoint = Point(3, 3);
    Vector initialVelocity   = Vector(3, 1);

    Point patrolPoint1 = Point(3, -3);
    Point patrolPoint2 = Point(-3, -3);
    Point patrolPoint3 = Point(3, 3);
    std::vector<Point> patrolPoints{patrolPoint1, patrolPoint2, patrolPoint3};

    double atPatrolPointTolerance = 1.0;
    double speedAtPatrolPoints    = 1.0;

    Robot robot = Robot(robotId, robotStartingPoint, initialVelocity, Angle::zero(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));

    PatrolTactic tactic = PatrolTactic(patrolPoints, atPatrolPointTolerance,
                                       Angle::zero(), speedAtPatrolPoints);

    tactic.updateRobot(robot);

    GenerateExpectedActions(robot, patrolPoints, Angle::zero(), speedAtPatrolPoints);
    GenerateRobotStatesToCompleteAction(patrolPoints, initialVelocity);

    // Act and Assert

    for (int i = 0; i < patrolPoints.size(); i++)
    {
        std::shared_ptr<Action> action_ptr = tactic.getNextAction();
        ASSERT_NE(nullptr, action_ptr);

        std::shared_ptr<MoveAction> move_action =
            std::dynamic_pointer_cast<MoveAction>(action_ptr);
        compareMoveActions(expectedActions[i], move_action);

        simulateActionToCompletion(robot, robotStateToCompleteActions[i], action_ptr,
                                   tactic);
    }
}

TEST_F(PatrolTacticTest, patrol_two_points_and_restart)
{
    // setup
    int robotId              = 0;
    Point robotStartingPoint = Point(-9, 5);
    Vector initialVelocity   = Vector(0, 0);

    Point patrolPoint1 = Point(7, 11);
    Point patrolPoint2 = Point(6, 9);
    std::vector<Point> patrolPoints{patrolPoint1, patrolPoint2};

    double atPatrolPointTolerance = 0.5;
    double speedAtPatrolPoints    = 2.0;

    Robot robot = Robot(robotId, robotStartingPoint, initialVelocity, Angle::zero(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));

    PatrolTactic tactic = PatrolTactic(patrolPoints, atPatrolPointTolerance,
                                       Angle::zero(), speedAtPatrolPoints);

    tactic.updateRobot(robot);

    GenerateExpectedActions(robot, patrolPoints, Angle::zero(), speedAtPatrolPoints);
    GenerateRobotStatesToCompleteAction(patrolPoints, initialVelocity);

    // Act and Assert

    for (int j = 0; j < 2; j++)
    {
        for (int i = 0; i < patrolPoints.size(); i++)
        {
            std::shared_ptr<Action> action_ptr = tactic.getNextAction();
            ASSERT_NE(nullptr, action_ptr);

            std::shared_ptr<MoveAction> move_action =
                std::dynamic_pointer_cast<MoveAction>(action_ptr);
            compareMoveActions(expectedActions[i], move_action);

            simulateActionToCompletion(robot, robotStateToCompleteActions[i], action_ptr,
                                       tactic);
        }
    }
}

TEST_F(PatrolTacticTest, cost_of_non_assigned_robot_when_tactic_is_assigned)
{
    // setup
    Point robotStartingPoint = Point(1, 1);
    Vector initialVelocity   = Vector(0, 1);

    Robot assigned_robot = Robot(0, robotStartingPoint, initialVelocity, Angle::zero(),
                                 AngularVelocity::zero(), Timestamp::fromSeconds(0));

    Robot non_assigned_robot =
        Robot(1, robotStartingPoint, initialVelocity, Angle::zero(),
              AngularVelocity::zero(), Timestamp::fromSeconds(0));

    std::vector<Point> patrolPoints{Point(2, 3), Point(4, 3)};
    World world = ::Test::TestUtil::createBlankTestingWorld();

    PatrolTactic tactic = PatrolTactic(patrolPoints, 1.0, Angle::zero(), 2.0);

    tactic.updateRobot(assigned_robot);

    double cost = tactic.calculateRobotCost(non_assigned_robot, world);

    EXPECT_EQ(COST_OF_NONASSIGNED_ROBOT_UNASSIGNED_TACTIC, cost);
}

TEST_F(PatrolTacticTest, cost_of_non_assigned_robot_when_tactic_is_not_assigned)
{
    // setup
    Point robotStartingPoint = Point(1, 1);
    Vector initialVelocity   = Vector(0, 1);

    Robot non_assigned_robot =
        Robot(1, robotStartingPoint, initialVelocity, Angle::zero(),
              AngularVelocity::zero(), Timestamp::fromSeconds(0));

    std::vector<Point> patrolPoints{Point(2, 2), Point(4, 3)};
    World world = ::Test::TestUtil::createBlankTestingWorld();

    PatrolTactic tactic = PatrolTactic(patrolPoints, 1.0, Angle::zero(), 2.0);

    double cost = tactic.calculateRobotCost(non_assigned_robot, world);

    EXPECT_NEAR(0.15, cost, 0.05);
}

TEST_F(PatrolTacticTest, cost_of_already_assigned_robot)
{
    // setup
    int robotId              = 0;
    Point robotStartingPoint = Point(1, 1);
    Vector initialVelocity   = Vector(0, 1);

    Robot assigned_robot =
        Robot(robotId, robotStartingPoint, initialVelocity, Angle::zero(),
              AngularVelocity::zero(), Timestamp::fromSeconds(0));

    std::vector<Point> patrolPoints{Point(2, 3), Point(4, 3)};
    World world = ::Test::TestUtil::createBlankTestingWorld();

    PatrolTactic tactic = PatrolTactic(patrolPoints, 1.0, Angle::zero(), 2.0);

    tactic.updateRobot(assigned_robot);

    double cost = tactic.calculateRobotCost(assigned_robot, world);

    EXPECT_EQ(COST_OF_ASSIGNED_ROBOT, cost);
}
