#include "software/ai/hl/stp/tactic/patrol_tactic.h"

#include <gtest/gtest.h>

#include "software/ai/hl/stp/action/move_action.h"
#include "software/test_util/test_util.h"

void compareMoveActions(std::shared_ptr<MoveAction> expected_move_action,
                        std::shared_ptr<MoveAction> move_action)
{

    EXPECT_EQ(expected_move_action->getDestination(),  move_action->getDestination());
    EXPECT_EQ(expected_move_action->getFinalOrientation(),  move_action->getFinalOrientation());
    EXPECT_EQ(expected_move_action->getFinalSpeed(),  move_action->getFinalSpeed());
    EXPECT_EQ(expected_move_action->getAutoKickType(),  move_action->getAutoKickType());
    EXPECT_EQ(expected_move_action->getDribblerEnabled(),  move_action->getDribblerEnabled());

}

void simulateActionToCompletion(Robot &robot, RobotState newRobotState, std::shared_ptr<Action> action_ptr, PatrolTactic &tactic) {

    //actions need to yield at least once before being able to complete
    std::unique_ptr<Intent> intent = action_ptr->getNextIntent();

    //update state of the robot and tactic
    robot.updateState(newRobotState);
    tactic.updateRobot(robot);

    //complete the action
    //tactic will update the action with latest robot status
    action_ptr = tactic.getNextAction();

    //Now that the action has the latest robot status, it is able to set the action to complete
    intent = action_ptr->getNextIntent();
    ASSERT_TRUE(action_ptr->done());
}

TEST(MoveTacticTest, patrol_one_point_should_return_one_acion)
{
    //Setup
    int robotId = 0;
    Point robotStartingPoint = Point(0,0);
    Vector initialVelocity = Vector(1,0);

    Point patrolPoint1 = Point(3,5);
    double atPatrolPointTolerance = 1.0;
    double speedAtPatrolPoints = 1.0;

    Robot robot = Robot(robotId, robotStartingPoint, initialVelocity, Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));

    PatrolTactic tactic = PatrolTactic(std::vector<Point>({patrolPoint1}), atPatrolPointTolerance, Angle::zero(), speedAtPatrolPoints);

    tactic.updateRobot(robot);

    auto expected_action = std::make_shared<MoveAction>(false);
    expected_action->updateControlParams(robot, patrolPoint1, Angle::zero(),speedAtPatrolPoints, DribblerEnable::OFF, MoveType::NORMAL,
                                        AutokickType::NONE, BallCollisionType::AVOID);
    //Act
    std::shared_ptr<Action> action_ptr = tactic.getNextAction();
    ASSERT_NE(nullptr, action_ptr);
    std::shared_ptr<MoveAction> move_action = std::dynamic_pointer_cast<MoveAction>(action_ptr);

    //Assert
    compareMoveActions(expected_action, move_action);
}

TEST(MoveTacticTest, patrol_multiple_points_should_return_multiple_actions)
{
    //setup
    int robotId = 0;
    Point robotStartingPoint = Point(3,3);
    Vector initialVelocity = Vector(3,1);

    Point patrolPoint1 = Point(3,-3);
    Point patrolPoint2 = Point(-3,-3);
    Point patrolPoint3 = Point(3,3);
    std::vector<Point> patrolPoints{patrolPoint1, patrolPoint2, patrolPoint3};

    double atPatrolPointTolerance = 1.0;
    double speedAtPatrolPoints = 1.0;

    Robot robot = Robot(robotId, robotStartingPoint, initialVelocity, Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));

    PatrolTactic tactic = PatrolTactic(patrolPoints, atPatrolPointTolerance, Angle::zero(), speedAtPatrolPoints);

    tactic.updateRobot(robot);

    auto expected_action1 = std::make_shared<MoveAction>(false);
    expected_action1->updateControlParams(robot, patrolPoint1, Angle::zero(),speedAtPatrolPoints, DribblerEnable::OFF, MoveType::NORMAL,
                                         AutokickType::NONE, BallCollisionType::AVOID);

    auto expected_action2 = std::make_shared<MoveAction>(false);
    expected_action2->updateControlParams(robot, patrolPoint2, Angle::zero(),speedAtPatrolPoints, DribblerEnable::OFF, MoveType::NORMAL,
                                         AutokickType::NONE, BallCollisionType::AVOID);

    auto expected_action3 = std::make_shared<MoveAction>(false);
    expected_action3->updateControlParams(robot, patrolPoint3, Angle::zero(),speedAtPatrolPoints, DribblerEnable::OFF, MoveType::NORMAL,
                                         AutokickType::NONE, BallCollisionType::AVOID);

    RobotState robotStateToCompleteFirstAction  = RobotState(patrolPoint1, initialVelocity, Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(1));
    RobotState robotStateToCompleteSecondAction = RobotState(patrolPoint2, initialVelocity, Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(2));

    //Act and Assert

    //first patrol point
    std::shared_ptr<Action> action_ptr1 = tactic.getNextAction();
    ASSERT_NE(nullptr, action_ptr1);
    std::shared_ptr<MoveAction> move_action1 = std::dynamic_pointer_cast<MoveAction>(action_ptr1);
    compareMoveActions(expected_action1, move_action1);

    simulateActionToCompletion(robot, robotStateToCompleteFirstAction, action_ptr1, tactic);

    //second patrol point
    std::shared_ptr<Action> action_ptr2 = tactic.getNextAction();
    ASSERT_NE(nullptr, action_ptr2);
    std::shared_ptr<MoveAction> move_action2 = std::dynamic_pointer_cast<MoveAction>(action_ptr2);

    compareMoveActions(expected_action2, move_action2);

    simulateActionToCompletion(robot, robotStateToCompleteSecondAction, action_ptr2, tactic);

    //third patrol point
    std::shared_ptr<Action> action_ptr3 = tactic.getNextAction();
    ASSERT_NE(nullptr, action_ptr3);
    std::shared_ptr<MoveAction> move_action3 = std::dynamic_pointer_cast<MoveAction>(action_ptr3);
    compareMoveActions(expected_action3, move_action3);

}



