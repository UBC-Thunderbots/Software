#include "software/ai/hl/stp/tactic/patrol_tactic.h"

#include <gtest/gtest.h>

#include "software/ai/hl/stp/action/move_action.h"
#include "software/test_util/test_util.h"

void compareMoveActions(std::shared_ptr<MoveAction> move_action,
                        std::shared_ptr<MoveAction> expected_move_action) {

    EXPECT_EQ(expected_move_action->getDestination(),  move_action->getDestination());
    EXPECT_EQ(expected_move_action->getFinalOrientation(),  move_action->getFinalOrientation());
    EXPECT_EQ(expected_move_action->getFinalSpeed(),  move_action->getFinalSpeed());
    EXPECT_EQ(expected_move_action->getAutoKickType(),  move_action->getAutoKickType());
    EXPECT_EQ(expected_move_action->getDribblerEnabled(),  move_action->getDribblerEnabled());

}

void PrintMoveActions(std::shared_ptr<MoveAction> move_action)
{
    std::cout << "destination: " << move_action->getDestination() << std::endl;
    std::cout << "final orientation: " << move_action->getFinalOrientation() << std::endl;
    std::cout << "final speed: " << move_action->getFinalSpeed() << std::endl << std::endl;

}

void simulateActionToCompletion(RobotState newRobotState, std::shared_ptr<Action> action_ptr, PatrolTactic tactic) {
    //    //complete action

    //make new state and call updatestate
    Robot robot = *(tactic.getAssignedRobot());
    (robot).updateState(RobotState(Point(3, 3), Vector(0, 1), Angle::zero(),
                                   AngularVelocity::zero(),

                                   Timestamp::fromSeconds(3)));
    tactic.updateRobot(robot);

    //   action_ptr = tactic.getNextAction();

    //complete the action

    //need to call getnextintent at least once for it to set done in action later
    action_ptr->getNextIntent();

    //will update the action with latest robot status
    action_ptr = tactic.getNextAction();

    //sets action to complete
    action_ptr->getNextIntent();

}
TEST(MoveTacticTest, robot_far_from_destination)
{
    Robot robot = Robot(0, Point(0,0), Vector(0,0), Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));

    PatrolTactic tactic = PatrolTactic(std::vector<Point>({Point(3,3), Point(0,1), Point(0,2)}), 1.0, Angle::zero(), 1.0);

    tactic.updateRobot(robot);

    std::shared_ptr<Action> action_ptr = tactic.getNextAction();

    ASSERT_NE(nullptr, action_ptr);

    std::shared_ptr<MoveAction> move_action = std::dynamic_pointer_cast<MoveAction>(action_ptr);

    PrintMoveActions(move_action);

    //complete action
    RobotState newRobotState = RobotState(Point(3, 3), Vector(0, 1), Angle::zero(),
                                     AngularVelocity::zero(),
                                     Timestamp::fromSeconds(3));

    simulateActionToCompletion(newRobotState, action_ptr, tactic);

    action_ptr = tactic.getNextAction();

    move_action = std::dynamic_pointer_cast<MoveAction>(action_ptr);

    PrintMoveActions(move_action);

//    std::shared_ptr<Action> action_ptr2 = tactic.getNextAction();
//    std::shared_ptr<MoveAction> move_action2 = std::dynamic_pointer_cast<MoveAction>(action_ptr2);
//    PrintMoveActions(move_action2);

}

