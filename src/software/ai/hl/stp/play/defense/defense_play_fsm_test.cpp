#include "software/ai/hl/stp/play/defense/defense_play_fsm.h"

#include <gtest/gtest.h>

#include "proto/parameters.pb.h"
#include "software/test_util/equal_within_tolerance.h"
#include "software/test_util/test_util.h"

TEST(DefensePlayFSMTest, test_transitions)
{
    std::shared_ptr<World> world = ::TestUtil::createBlankTestingWorld();

    FSM<DefensePlayFSM> fsm(DefensePlayFSM{std::make_shared<TbotsProto::AiConfig>()});
    EXPECT_TRUE(fsm.is(boost::sml::state<DefensePlayFSM::DefenseState>));

    // Place enemy robots behind the ball implying there is a large attack
    ::TestUtil::setEnemyRobotPositions(world, {Point(0, 0), Point(-1, 0), Point(-2, 0)},
                                       Timestamp::fromSeconds(0));

    fsm.process_event(DefensePlayFSM::Update(
        DefensePlayFSM::ControlParams{
            .max_allowed_speed_mode = TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT},
        PlayUpdate(
            world, 3, [](PriorityTacticVector new_tactics) {}, InterPlayCommunication{},
            [](InterPlayCommunication comm) {})));

    EXPECT_TRUE(fsm.is(boost::sml::state<DefensePlayFSM::DefenseState>));


    // Place more friendly robots behind the ball implying this is a safe time
    // to aggressively defense
    ::TestUtil::setFriendlyRobotPositions(
        world, {Point(-1, 0), Point(-3, 0), Point(-2, 0)}, Timestamp::fromSeconds(0));

    fsm.process_event(DefensePlayFSM::Update(
        DefensePlayFSM::ControlParams{
            .max_allowed_speed_mode = TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT},
        PlayUpdate(
            world, 3, [](PriorityTacticVector new_tactics) {}, InterPlayCommunication{},
            [](InterPlayCommunication comm) {})));

    EXPECT_TRUE(fsm.is(boost::sml::state<DefensePlayFSM::AggressiveDefenseState>));
}

TEST(DefensePlayFSMTest, changing_threats_caps_and_budgets)
{
    for (unsigned int cap : {0u, 1u, 2u})
    {
        auto config = std::make_shared<TbotsProto::AiConfig>();
        config->mutable_defense_play_config()
            ->mutable_defender_assignment_config()
            ->set_max_num_crease_defenders(cap);
        DefensePlayFSM defense(config);
        auto world = TestUtil::createBlankTestingWorld();
        PriorityTacticVector published;
        bool called = false;
        auto update = [&](unsigned int budget)
        {
            called = false;
            DefensePlayFSM::Update event(
                {TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT},
                PlayUpdate(
                    world, budget,
                    [&](PriorityTacticVector tactics)
                    {
                        published = tactics;
                        called    = true;
                    },
                    InterPlayCommunication{}, [](InterPlayCommunication) {}));
            defense.shadowAndBlockShots(event);
            EXPECT_TRUE(called);
            size_t total = 0, crease = 0;
            for (const auto& priority : published)
                for (const auto& tactic : priority)
                {
                    ++total;
                    if (std::dynamic_pointer_cast<CreaseDefenderTactic>(tactic))
                        ++crease;
                }
            EXPECT_LE(total, budget);
            EXPECT_LE(crease, cap);
            return total;
        };
        EXPECT_EQ(update(6), 0);
        TestUtil::setEnemyRobotPositions(world, {Point(-1, 0)}, Timestamp());
        EXPECT_EQ(update(6), cap);
        EXPECT_EQ(update(0), 0);
        EXPECT_LE(update(6), cap);
        TestUtil::setEnemyRobotPositions(world, {Point(-1, 0), Point(0, 2), Point(1, -2)},
                                         Timestamp());
        EXPECT_GT(update(6), 0);
        world = TestUtil::createBlankTestingWorld();
        EXPECT_EQ(update(6), 0);
        TestUtil::setEnemyRobotPositions(world, {Point(-1, 0)}, Timestamp());
        EXPECT_EQ(update(1), cap > 0 ? 1 : 0);
    }
}

TEST(DefensePlayFSMTest, unchanged_shadowing_update_preserves_tactic_state)
{
    DefensePlayFSM defense(std::make_shared<TbotsProto::AiConfig>());
    auto world = TestUtil::createBlankTestingWorld();
    TestUtil::setEnemyRobotPositions(world, {Point(-1, 0), Point(0, 2), Point(1, -2)},
                                     Timestamp());
    PriorityTacticVector published;
    DefensePlayFSM::Update event(
        {TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT},
        PlayUpdate(
            world, 6, [&](PriorityTacticVector tactics) { published = tactics; },
            InterPlayCommunication{}, [](InterPlayCommunication) {}));
    defense.shadowAndBlockShots(event);
    const auto previous = published;
    ASSERT_FALSE(previous[1].empty());
    ASSERT_FALSE(previous[2].empty());
    defense.shadowAndBlockShots(event);
    EXPECT_EQ(published, previous);
}

TEST(DefensePlayFSMTest, primary_threat_hold_delays_a_change_of_primary_threat)
{
    for (unsigned int hold : {0u, 10u})
    {
        auto config = std::make_shared<TbotsProto::AiConfig>();
        config->mutable_defense_play_config()
            ->mutable_defender_assignment_config()
            ->set_primary_threat_hold_updates(hold);
        DefensePlayFSM defense(config);
        auto world = TestUtil::createBlankTestingWorld();
        // Enemy robot 0 is nearest the ball
        TestUtil::setEnemyRobotPositions(world, {Point(-1, 0), Point(2, 0)},
                                         Timestamp());
        DefensePlayFSM::Update first({TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT},
                                     PlayUpdate(
                                         world, 6, [](PriorityTacticVector) {},
                                         InterPlayCommunication{},
                                         [](InterPlayCommunication) {}));
        EXPECT_EQ(defense.getStableEnemyThreats(first).front().robot.id(), 0);

        // Enemy robot 1 is now nearest the ball. The first primary threat change is not
        // held because the previous primary had been primary long enough
        TestUtil::setEnemyRobotPositions(world, {Point(-2, 0), Point(1, 0)},
                                         Timestamp());
        DefensePlayFSM::Update second({TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT},
                                      PlayUpdate(
                                          world, 6, [](PriorityTacticVector) {},
                                          InterPlayCommunication{},
                                          [](InterPlayCommunication) {}));
        EXPECT_EQ(defense.getStableEnemyThreats(second).front().robot.id(), 1);

        // Robot 0 is nearest again, but a change right after the last one is held
        DefensePlayFSM::Update third({TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT},
                                     PlayUpdate(
                                         world, 6, [](PriorityTacticVector) {},
                                         InterPlayCommunication{},
                                         [](InterPlayCommunication) {}));
        TestUtil::setEnemyRobotPositions(world, {Point(-1, 0), Point(2, 0)},
                                         Timestamp());
        int updates_until_change = 0;
        while (updates_until_change < 50 &&
               defense.getStableEnemyThreats(third).front().robot.id() == 1)
        {
            updates_until_change++;
        }
        EXPECT_EQ(updates_until_change, static_cast<int>(hold)) << "hold " << hold;
    }
}
