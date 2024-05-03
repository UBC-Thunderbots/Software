#include "software/ai/hl/stp/skill/pivot_kick/pivot_kick_skill_fsm.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

TEST(PivotKickSkillFSMTest, test_transitions)
{
    std::shared_ptr<World> world = ::TestUtil::createBlankTestingWorld();
    Robot robot                  = ::TestUtil::createRobotAtPos(Point(-2, -3));
    PivotKickSkillFSM::ControlParams control_params{
        .kick_origin       = Point(-2, 1.5),
        .kick_direction    = Angle::threeQuarter(),
        .auto_chip_or_kick = {AutoChipOrKickMode::AUTOKICK, 1.2}};

    std::shared_ptr<Strategy> strategy =
        std::make_shared<Strategy>(TbotsProto::AiConfig());

    FSM<PivotKickSkillFSM> fsm{DribbleSkillFSM()};

    EXPECT_TRUE(fsm.is(boost::sml::state<PivotKickSkillFSM::StartState>));

    fsm.process_event(PivotKickSkillFSM::Update(
        control_params,
        SkillUpdate(robot, world, strategy, [](std::shared_ptr<Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<DribbleSkillFSM>));

    // Robot now has the ball at the right location and is pointing in the right direction
    robot.updateState(RobotState(Point(-2, 1.55), Vector(), Angle::threeQuarter(),
                                 AngularVelocity::zero()),
                      Timestamp::fromSeconds(123));
    ::TestUtil::setBallPosition(world, Point(-2, 1.5), Timestamp::fromSeconds(123));
    EXPECT_TRUE(robot.isNearDribbler(world->ball().position()));

    // it takes two ticks for the fsm to realize that it's in the kick state
    fsm.process_event(PivotKickSkillFSM::Update(
        control_params,
        SkillUpdate(robot, world, strategy, [](std::shared_ptr<Primitive>) {})));
    fsm.process_event(PivotKickSkillFSM::Update(
        control_params,
        SkillUpdate(robot, world, strategy, [](std::shared_ptr<Primitive>) {})));
    // Transition to KickState
    EXPECT_TRUE(fsm.is(boost::sml::state<PivotKickSkillFSM::KickState>));

    // Ball is now kicked
    robot = ::TestUtil::createRobotAtPos(Point(-2, 1.8));
    ::TestUtil::setBallVelocity(world, Vector(0, -2.1), Timestamp::fromSeconds(123));
    EXPECT_TRUE(world->ball().hasBallBeenKicked(Angle::threeQuarter()));

    // Tactic is done
    fsm.process_event(PivotKickSkillFSM::Update(
        control_params,
        SkillUpdate(robot, world, strategy, [](std::shared_ptr<Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::X));
}
