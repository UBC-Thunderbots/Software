#include <gtest/gtest.h>

#include "software/ai/ai.h"
#include "software/test_util/test_util.h"
#include "software/world/world.h"

class GameStatePlaySelectionTest : public ::testing::Test,
                                   public ::testing::WithParamInterface<RefboxGameState>
{
   public:
   protected:
    void SetUp() override
    {
        world.mutableField() = ::Test::TestUtil::createSSLDivBField();

        Robot robot_0(0, Point(-1.1, 1), Vector(), Angle::zero(), AngularVelocity::zero(),
                      Timestamp::fromSeconds(0));
        Robot robot_1(1, Point(2, 0.81), Vector(), Angle::zero(), AngularVelocity::zero(),
                      Timestamp::fromSeconds(0));
        Robot robot_2(2, Point(0, 5.0), Vector(), Angle::zero(), AngularVelocity::zero(),
                      Timestamp::fromSeconds(0));
        world.mutableFriendlyTeam().updateRobots({robot_0, robot_1, robot_2});

        Robot enemy_robot_0(0, Point(1.1, 1), Vector(), Angle::zero(),
                            AngularVelocity::zero(), Timestamp::fromSeconds(0));
        Robot enemy_robot_1(1, Point(-2, 0.81), Vector(), Angle::zero(),
                            AngularVelocity::zero(), Timestamp::fromSeconds(0));
        Robot enemy_robot_2(2, Point(0, -5.0), Vector(), Angle::zero(),
                            AngularVelocity::zero(), Timestamp::fromSeconds(0));
        world.mutableEnemyTeam().updateRobots(
            {enemy_robot_0, enemy_robot_1, enemy_robot_2});
    }

    World world;
    AI ai;
};

// TEST_P(GameStatePlaySelectionTest, test_play_selection_for_refbox_game_states)
//{
//    world.mutableGameState().updateRefboxGameState(GetParam());
//    ai.getPrimitives(world);
//    // assert that the play name is not "None"
//    ASSERT_NE(ai.getPlayInfo().play_name, AI::NO_PLAY_NAME);
//}
//
// auto all_refbox_game_states = ::Test::TestUtil::getAllRefboxGameStates();
//
// INSTANTIATE_TEST_CASE_P(AllRefboxGameStates, GameStatePlaySelectionTest,
//                        ::testing::ValuesIn(all_refbox_game_states.begin(),
//                                            all_refbox_game_states.end()));
