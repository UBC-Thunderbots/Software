#include "software/ai/hl/stp/tactic/move_tactic.h"

#include <gtest/gtest.h>

#include "software/ai/hl/stp/action/move_action.h"
#include "software/test_util/test_util.h"

TEST(MoveTacticTest, robot_far_from_destination)
{
    Robot robot = Robot(0, Point(), Vector(), Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));

    MoveTactic tactic = MoveTactic(false);
    tactic.updateRobot(robot);
    tactic.updateControlParams(Point(1, 0), Angle::quarter(), 1.0);
    auto action_ptr = tactic.getNextAction();

    // Check an action was returned (the pointer is not null)
    EXPECT_TRUE(action_ptr);

    auto move_action = std::dynamic_pointer_cast<MoveAction>(action_ptr);
    ASSERT_NE(move_action, nullptr);
    ASSERT_TRUE(move_action->getRobot().has_value());
    EXPECT_EQ(0, move_action->getRobot()->id());
    EXPECT_EQ(Point(1, 0), move_action->getDestination());
    EXPECT_EQ(Angle::quarter(), move_action->getFinalOrientation());
    EXPECT_EQ(1.0, move_action->getFinalSpeed());
}

TEST(MoveTacticTest, robot_at_destination)
{
    World world = ::TestUtil::createBlankTestingWorld();
    Robot robot = Robot(0, Point(), Vector(0, 0), Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));

    MoveTactic tactic = MoveTactic(false);

    EXPECT_FALSE(tactic.done());
    tactic.updateControlParams(Point(0, 0), Angle::zero(), 0.0);
    // We call the Action twice. The first time the Tactic is starting up so it's not
    // done. In all future calls, the action will be done
    EXPECT_TRUE(tactic.get(robot, world));
    EXPECT_FALSE(tactic.done());
    EXPECT_TRUE(tactic.get(robot, world));

    EXPECT_TRUE(tactic.done());
}

TEST(MoveTacticTest, test_calculate_robot_cost)
{
    World world = ::TestUtil::createBlankTestingWorld();

    Robot robot = Robot(0, Point(), Vector(), Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));

    MoveTactic tactic = MoveTactic(false);
    tactic.updateWorldParams(world);
    tactic.updateControlParams(Point(3, -4), Angle::zero(), 0.0);

    EXPECT_EQ(5 / world.field().totalXLength(), tactic.calculateRobotCost(robot, world));
}

TEST(MoveFSMTest, test_transitions)
{
    World world = ::TestUtil::createBlankTestingWorld();
    Robot robot = ::TestUtil::createRobotAtPos(Point(-2, -3));
    MoveFSM::ControlParams control_params{.destination       = Point(2, 3),
                                          .final_orientation = Angle::half(),
                                          .final_speed       = 0.0};

    boost::sml::sm<MoveFSM> fsm;
    EXPECT_TRUE(fsm.is(boost::sml::state<MoveFSM::idle_state>));
    fsm.process_event(MoveFSM::Update{
        .control_params = control_params,
        .common         = TacticUpdate{.robot      = robot,
                               .world      = world,
                               .set_intent = [](std::unique_ptr<Intent>) {}}});
    // robot far from destination
    EXPECT_TRUE(fsm.is(boost::sml::state<MoveFSM::move_state>));
    robot = ::TestUtil::createRobotAtPos(Point(2, 2));
    fsm.process_event(MoveFSM::Update{
        .control_params = control_params,
        .common         = TacticUpdate{.robot      = robot,
                               .world      = world,
                               .set_intent = [](std::unique_ptr<Intent>) {}}});
    // robot close to destination
    EXPECT_TRUE(fsm.is(boost::sml::state<MoveFSM::move_state>));
    robot.updateState(
        RobotState(Point(2, 3), Vector(), Angle::half(), AngularVelocity::zero()),
        Timestamp::fromSeconds(0));
    fsm.process_event(MoveFSM::Update{
        .control_params = control_params,
        .common         = TacticUpdate{.robot      = robot,
                               .world      = world,
                               .set_intent = [](std::unique_ptr<Intent>) {}}});
    // robot at destination and facing the right way
    EXPECT_TRUE(fsm.is(boost::sml::X));
}
