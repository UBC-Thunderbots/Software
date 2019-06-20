#include <gtest/gtest.h>

#include <exception>

#include "ai/hl/stp/stp.h"
#include "ai/world/world.h"
#include "test/test_util/test_util.h"


class STPRefboxGameStatePlaySelectionTest
    : public ::testing::Test,
      public ::testing::WithParamInterface<RefboxGameState>
{
   protected:
    void SetUp() override
    {
        stp                  = STP(0);
        world.mutableField() = ::Test::TestUtil::createSSLDivBField();

        Robot robot_0(0, Point(-1.1, 1), Point(), Angle::zero(), AngularVelocity::zero(),
                      Timestamp::fromSeconds(0));
        Robot robot_1(1, Point(2, 0.81), Point(), Angle::zero(), AngularVelocity::zero(),
                      Timestamp::fromSeconds(0));
        Robot robot_2(2, Point(0, 5.0), Point(), Angle::zero(), AngularVelocity::zero(),
                      Timestamp::fromSeconds(0));
        world.mutableFriendlyTeam().updateRobots({robot_0, robot_1, robot_2});

        Robot enemy_robot_0(0, Point(1.1, 1), Point(), Angle::zero(),
                            AngularVelocity::zero(), Timestamp::fromSeconds(0));
        Robot enemy_robot_1(1, Point(-2, 0.81), Point(), Angle::zero(),
                            AngularVelocity::zero(), Timestamp::fromSeconds(0));
        Robot enemy_robot_2(2, Point(0, -5.0), Point(), Angle::zero(),
                            AngularVelocity::zero(), Timestamp::fromSeconds(0));
        world.mutableEnemyTeam().updateRobots(
            {enemy_robot_0, enemy_robot_1, enemy_robot_2});
    }
    STP stp;
    World world;
};

TEST_P(STPRefboxGameStatePlaySelectionTest,
       test_play_selection_for_all_refbox_game_states)
{
    world.mutableGameState().updateRefboxGameState(GetParam());

    try
    {
        auto play_ptr = stp.calculateNewPlay(world);
    }
    catch (...)
    {
        FAIL() << "No play for game state " << GetParam();
    }
}

auto all_refbox_game_states = ::Test::TestUtil::getAllRefboxGameStates();

// TODO: uncomment this when plays are completed
// INSTANTIATE_TEST_CASE_P(AllRefboxGameStates, STPRefboxGameStatePlaySelectionTest,
//                        ::testing::ValuesIn(all_refbox_game_states.begin(),
//                        all_refbox_game_states.end()));

int main(int argc, char **argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}