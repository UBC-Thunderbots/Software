#include "software/ai/hl/stp/tactic/move_fsm.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

TEST(MoveFSMTest, test_transitions)
{
    World world = ::TestUtil::createBlankTestingWorld();
    Robot robot = ::TestUtil::createRobotAtPos(Point(-2, -3));
    MoveFSM::ControlParams control_params{.destination       = Point(2, 3),
                                          .final_orientation = Angle::half(),
                                          .final_speed       = 0.0};

    BaseFSM<MoveFSM> fsm;
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
