#include <gtest/gtest.h>

#include "software/ai/hl/stp/skill/stop/stop_skill_fsm.h"
#include "software/test_util/test_util.h"

TEST(StopSkillFSMTest, test_transitions)
{
    std::shared_ptr<Strategy> strategy =
        std::make_shared<Strategy>(TbotsProto::AiConfig());
    std::shared_ptr<World> world = ::TestUtil::createBlankTestingWorld();
    Robot robot(0,
                RobotState(Point(1, -3), Vector(2.1, 3.1), Angle::half(),
                           AngularVelocity::zero()),
                Timestamp::fromSeconds(123));

    FSM<StopSkillFSM> fsm{StopSkillFSM()};
    EXPECT_TRUE(fsm.is(boost::sml::state<StopSkillFSM::StopState>));
    fsm.process_event(StopSkillFSM::Update(
        {}, SkillUpdate(robot, world, strategy, [](std::shared_ptr<Primitive>) {})));
    // robot is still moving
    EXPECT_TRUE(fsm.is(boost::sml::state<StopSkillFSM::StopState>));
    robot = Robot(0,
                  RobotState(Point(1, -3), Vector(1.1, 2.1), Angle::half(),
                             AngularVelocity::zero()),
                  Timestamp::fromSeconds(123));
    fsm.process_event(StopSkillFSM::Update(
        {}, SkillUpdate(robot, world, strategy, [](std::shared_ptr<Primitive>) {})));
    // robot is still moving
    EXPECT_TRUE(fsm.is(boost::sml::state<StopSkillFSM::StopState>));
    robot = TestUtil::createRobotAtPos(Point(1, -3));
    fsm.process_event(StopSkillFSM::Update(
        {}, SkillUpdate(robot, world, strategy, [](std::shared_ptr<Primitive>) {})));
    // robot stopped
    EXPECT_TRUE(fsm.is(boost::sml::X));
}
