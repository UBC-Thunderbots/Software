#include "software/ai/hl/stp/tactic/stop/stop_fsm.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

TEST(StopFSMTest, test_transitions)
{
    World world = ::TestUtil::createBlankTestingWorld();
    Robot robot(0,
                RobotState(Point(1, -3), Vector(2.1, 3.1), Angle::half(),
                           AngularVelocity::zero()),
                Timestamp::fromSeconds(123));

    BaseFSM<StopFSM> fsm;
    EXPECT_TRUE(fsm.is(boost::sml::state<StopFSM::idle_state>));
    fsm.process_event(
        StopFSM::Update(StopFSM::ControlParams{.coast = false},
                        TacticUpdate(robot, world, [](std::unique_ptr<Intent>) {})));
    // robot is still moving
    EXPECT_TRUE(fsm.is(boost::sml::state<StopFSM::stop_state>));
    robot = Robot(0,
                  RobotState(Point(1, -3), Vector(1.1, 2.1), Angle::half(),
                             AngularVelocity::zero()),
                  Timestamp::fromSeconds(123));
    fsm.process_event(
        StopFSM::Update(StopFSM::ControlParams{.coast = false},
                        TacticUpdate(robot, world, [](std::unique_ptr<Intent>) {})));
    // robot is still moving
    EXPECT_TRUE(fsm.is(boost::sml::state<StopFSM::stop_state>));
    robot = TestUtil::createRobotAtPos(Point(1, -3));
    fsm.process_event(
        StopFSM::Update(StopFSM::ControlParams{.coast = false},
                        TacticUpdate(robot, world, [](std::unique_ptr<Intent>) {})));
    // robot stopped
    EXPECT_TRUE(fsm.is(boost::sml::X));
}
