#include "software/ai/hl/stp/tactic/halt/halt_fsm.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

TEST(HaltFSMTest, test_transitions)
{
    World world = ::TestUtil::createBlankTestingWorld();
    Robot robot(0,
                RobotState(Point(1, -3), Vector(2.1, 3.1), Angle::half(),
                           AngularVelocity::zero()),
                Timestamp::fromSeconds(123));

<<<<<<< HEAD:src/software/ai/hl/stp/tactic/halt/halt_fsm_test.cpp
    FSM<HaltFSM> fsm{HaltFSM()};
    EXPECT_TRUE(fsm.is(boost::sml::state<HaltFSM::StopState>));
    fsm.process_event(HaltFSM::Update(
        {}, TacticUpdate(
                robot, world, [](std::unique_ptr<TbotsProto::Primitive>) {},
                TEST_UTIL_CREATE_MOTION_CONTROL_NO_DEST)));
=======
    FSM<StopFSM> fsm{StopFSM()};
    EXPECT_TRUE(fsm.is(boost::sml::state<StopFSM::StopState>));
    fsm.process_event(StopFSM::Update(
        {}, TacticUpdate(robot, world, [](std::shared_ptr<Primitive>) {})));
>>>>>>> upstream/master:src/software/ai/hl/stp/tactic/stop/stop_fsm_test.cpp
    // robot is still moving
    EXPECT_TRUE(fsm.is(boost::sml::state<HaltFSM::StopState>));
    robot = Robot(0,
                  RobotState(Point(1, -3), Vector(1.1, 2.1), Angle::half(),
                             AngularVelocity::zero()),
                  Timestamp::fromSeconds(123));
<<<<<<< HEAD:src/software/ai/hl/stp/tactic/halt/halt_fsm_test.cpp
    fsm.process_event(HaltFSM::Update(
        {}, TacticUpdate(
                robot, world, [](std::unique_ptr<TbotsProto::Primitive>) {},
                TEST_UTIL_CREATE_MOTION_CONTROL_NO_DEST)));
=======
    fsm.process_event(StopFSM::Update(
        {}, TacticUpdate(robot, world, [](std::shared_ptr<Primitive>) {})));
>>>>>>> upstream/master:src/software/ai/hl/stp/tactic/stop/stop_fsm_test.cpp
    // robot is still moving
    EXPECT_TRUE(fsm.is(boost::sml::state<HaltFSM::StopState>));
    robot = TestUtil::createRobotAtPos(Point(1, -3));
<<<<<<< HEAD:src/software/ai/hl/stp/tactic/halt/halt_fsm_test.cpp
    fsm.process_event(HaltFSM::Update(
        {}, TacticUpdate(
                robot, world, [](std::unique_ptr<TbotsProto::Primitive>) {},
                TEST_UTIL_CREATE_MOTION_CONTROL_NO_DEST)));
=======
    fsm.process_event(StopFSM::Update(
        {}, TacticUpdate(robot, world, [](std::shared_ptr<Primitive>) {})));
>>>>>>> upstream/master:src/software/ai/hl/stp/tactic/stop/stop_fsm_test.cpp
    // robot stopped
    EXPECT_TRUE(fsm.is(boost::sml::X));
}
