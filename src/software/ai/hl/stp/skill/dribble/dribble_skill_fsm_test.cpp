#include "software/ai/hl/stp/skill/dribble/dribble_skill_fsm.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

TEST(DribbleSkillFSMTest, test_transitions)
{
    Robot robot = ::TestUtil::createRobotAtPos(Point(-2, -3));
    World world = ::TestUtil::createBlankTestingWorld();
    world =
        ::TestUtil::setBallPosition(world, Point(0.5, 0), Timestamp::fromSeconds(123));
    world =
        ::TestUtil::setBallVelocity(world, Vector(0, -1), Timestamp::fromSeconds(123));

    std::shared_ptr<Strategy> strategy =
        std::make_shared<Strategy>(TbotsProto::AiConfig());

    FSM<DribbleSkillFSM> fsm{DribbleSkillFSM()};

    // Start in Dribble
    EXPECT_TRUE(fsm.is(boost::sml::state<DribbleSkillFSM::GetPossession>));

    // Stay in Dribble since ball not in possession yet
    fsm.process_event(DribbleSkillFSM::Update(
        {std::nullopt, std::nullopt, false},
        SkillUpdate(robot, world, strategy, [](std::shared_ptr<Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<DribbleSkillFSM::GetPossession>));

    // Robot at ball point, so it has possession, so transition to dribble state
    robot = ::TestUtil::createRobotAtPos(Point(0.5, 0));
    fsm.process_event(DribbleSkillFSM::Update(
        {std::nullopt, std::nullopt, false},
        SkillUpdate(robot, world, strategy, [](std::shared_ptr<Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<DribbleSkillFSM::Dribble>));

    // No dribble destination set, so tactic is done
    fsm.process_event(DribbleSkillFSM::Update(
        {std::nullopt, std::nullopt, false},
        SkillUpdate(robot, world, strategy, [](std::shared_ptr<Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::X));

    // Set dribble destination, so tactic should be undone
    fsm.process_event(DribbleSkillFSM::Update(
        {Point(1, -1), std::nullopt, false},
        SkillUpdate(robot, world, strategy, [](std::shared_ptr<Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<DribbleSkillFSM::Dribble>));

    // Move ball to destination, but not the robot, so we should try to regain possession
    world = ::TestUtil::setBallPosition(world, Point(1, -1), Timestamp::fromSeconds(124));
    fsm.process_event(DribbleSkillFSM::Update(
        {Point(1, -1), std::nullopt, false},
        SkillUpdate(robot, world, strategy, [](std::shared_ptr<Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<DribbleSkillFSM::GetPossession>));

    // Move robot to where the ball is, so we now have possession and ball is at the
    // destination, but we go to the dribble state before being done
    robot = ::TestUtil::createRobotAtPos(Point(1, -1));
    fsm.process_event(DribbleSkillFSM::Update(
        {Point(1, -1), std::nullopt, false},
        SkillUpdate(robot, world, strategy, [](std::shared_ptr<Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<DribbleSkillFSM::Dribble>));

    // Finally FSM is done again
    fsm.process_event(DribbleSkillFSM::Update(
        {Point(1, -1), std::nullopt, false},
        SkillUpdate(robot, world, strategy, [](std::shared_ptr<Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::X));
}
