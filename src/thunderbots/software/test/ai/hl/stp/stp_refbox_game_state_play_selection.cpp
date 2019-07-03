#include <gtest/gtest.h>

#include <exception>

#include "ai/hl/stp/play/example_play.h"
#include "ai/hl/stp/play/halt_play.h"
#include "ai/hl/stp/play/play_factory.h"
#include "ai/hl/stp/stp.h"
#include "ai/world/world.h"
#include "test/test_util/test_util.h"


struct PlaySelectionTestParams
{
    std::string name;
    std::vector<Point> friendly_positions;
    std::vector<Point> enemy_positions;
    Point ball_position;
    Vector ball_velocity;
    RefboxGameState first_game_state;
    RefboxGameState second_game_state;
};


class STPRefboxGameStatePlaySelectionTestWithPositions
    : public ::testing::Test,
      public ::testing::WithParamInterface<PlaySelectionTestParams>
{
   public:
    STPRefboxGameStatePlaySelectionTestWithPositions()
        : stp([]() { return std::make_unique<HaltPlay>(); }, 0)
    {
    }

   protected:
    void SetUp() override
    {
        world.mutableField()     = ::Test::TestUtil::createSSLDivBField();
        world.mutableGameState() = GameState();
    }

    STP stp;
    World world;
};

TEST_P(STPRefboxGameStatePlaySelectionTestWithPositions,
       test_play_selection_for_states_and_positions)
{
    // set up the friendly team
    ::Test::TestUtil::setFriendlyRobotPositions(world, GetParam().friendly_positions,
                                                Timestamp());
    ::Test::TestUtil::setEnemyRobotPositions(world, GetParam().enemy_positions,
                                             Timestamp());
    world.mutableBall() =
        Ball(GetParam().ball_position, GetParam().ball_velocity, Timestamp());

    // to set restart reason, etc. properly
    world.mutableGameState().updateRefboxGameState(GetParam().first_game_state,
                                                   world.ball());
    world.mutableGameState().updateRefboxGameState(GetParam().second_game_state,
                                                   world.ball());
    std::unique_ptr<Play> play;
    try
    {
        play = stp.calculateNewPlay(world);
    }
    catch (const std::runtime_error& e)
    {
        FAIL() << "No play for test case: " + GetParam().name;
    }

    // make sure we're not getting Example Play or Halt Play for non-halt situations
    if (dynamic_cast<ExamplePlay*>(play.get()) != nullptr ||
        (dynamic_cast<HaltPlay*>(play.get()) != nullptr && !world.gameState().isHalted()))
    {
        // we got example play, fail
        FAIL() << "Incorrect play " << play->getName()
               << " for test case: " + GetParam().name;
    }

    std::cout << "Play " << play->getName()
              << " selected for test case: " << GetParam().name << std::endl;
}

std::vector<PlaySelectionTestParams> test_params = {
    {.name               = "Our Kickoff",
     .friendly_positions = {{-4.0, 0}, {-0.25, 0}, {-2.0, 1.0}, {-2.0, -1.0}},
     .enemy_positions    = {{4.0, 0}, {0.5, 0}, {2.0, 1.0}, {2.0, -1.0}},
     .ball_position      = Point(0, 0),
     .ball_velocity      = Vector(),
     .first_game_state   = RefboxGameState::PREPARE_KICKOFF_US,
     .second_game_state  = RefboxGameState::PREPARE_KICKOFF_US},
    {.name               = "Their Kickoff",
     .friendly_positions = {{-4.0, 0}, {-0.5, 0}, {-2.0, 1.0}, {-2.0, -1.0}},
     .enemy_positions    = {{4.0, 0}, {0.25, 0}, {2.0, 1.0}, {2.0, -1.0}},
     .ball_position      = Point(0, 0),
     .ball_velocity      = Vector(),
     .first_game_state   = RefboxGameState::PREPARE_KICKOFF_THEM,
     .second_game_state  = RefboxGameState::PREPARE_KICKOFF_THEM},
    {.name               = "Normal play after our kickoff",
     .friendly_positions = {{-4.0, 0}, {-0.25, 0}, {-2.0, 1.0}, {-2.0, -1.0}},
     .enemy_positions    = {{4.0, 0}, {0.5, 0}, {2.0, 1.0}, {2.0, -1.0}},
     .ball_position      = Point(0, 0),
     .ball_velocity      = Vector(1, 0),
     .first_game_state   = RefboxGameState::PREPARE_KICKOFF_US,
     .second_game_state  = RefboxGameState::NORMAL_START},
    {.name               = "Normal play after their kickoff",
     .friendly_positions = {{-4.0, 0}, {-0.5, 0}, {-2.0, 1.0}, {-2.0, -1.0}},
     .enemy_positions    = {{4.0, 0}, {0.25, 0}, {2.0, 1.0}, {2.0, -1.0}},
     .ball_position      = Point(0, 0),
     .ball_velocity      = Vector(-1, 0),
     .first_game_state   = RefboxGameState::PREPARE_KICKOFF_THEM,
     .second_game_state  = RefboxGameState::NORMAL_START},
    {.name               = "Friendly indirect free setup on friendly field side",
     .friendly_positions = {{-4.0, 0}, {-2.0, 3.0}, {-2.0, 0.0}, {-2.0, -1.0}},
     .enemy_positions    = {{4.0, 0}, {0.25, 0}, {2.0, 1.0}, {2.0, -1.0}},
     .ball_position      = Point(-2.0, 2.8),
     .ball_velocity      = Vector(),
     .first_game_state   = RefboxGameState::INDIRECT_FREE_US,
     .second_game_state  = RefboxGameState::INDIRECT_FREE_US},
    {.name               = "Friendly indirect free setup on enemy field side",
     .friendly_positions = {{-4.0, 0}, {2.0, -3.0}, {-2.0, 0.0}, {-2.0, -1.0}},
     .enemy_positions    = {{4.0, 0}, {0.25, 0}, {2.0, 1.0}, {2.0, -1.0}},
     .ball_position      = Point(2.0, -2.8),
     .ball_velocity      = Vector(),
     .first_game_state   = RefboxGameState::INDIRECT_FREE_US,
     .second_game_state  = RefboxGameState::INDIRECT_FREE_US},
    // enemy indirect free on both sides of the field
    {.name               = "Enemy indirect free setup on friendly field side",
     .friendly_positions = {{-4.0, 0}, {0.25, 0}, {-2.0, 0.0}, {-2.0, -1.0}},
     .enemy_positions    = {{4.0, 0}, {-2.0, 3.0}, {2.0, 1.0}, {2.0, -1.0}},
     .ball_position      = Point(-2.0, 2.8),
     .ball_velocity      = Vector(),
     .first_game_state   = RefboxGameState::INDIRECT_FREE_THEM,
     .second_game_state  = RefboxGameState::INDIRECT_FREE_THEM},
    {.name               = "Enemy indirect free setup on enemy field side",
     .friendly_positions = {{-4.0, 0}, {0.25, 0}, {-2.0, 0.0}, {-2.0, -1.0}},
     .enemy_positions    = {{4.0, 0}, {2.0, -3.0}, {2.0, 1.0}, {2.0, -1.0}},
     .ball_position      = Point(2.0, -2.8),
     .ball_velocity      = Vector(),
     .first_game_state   = RefboxGameState::INDIRECT_FREE_THEM,
     .second_game_state  = RefboxGameState::INDIRECT_FREE_THEM},
    {.name               = "Friendly direct free setup on enemy field side (corner kick)",
     .friendly_positions = {{-4.0, 0}, {4.5, -3.0}, {-2.0, 0.0}, {-2.0, -1.0}},
     .enemy_positions    = {{4.0, 0}, {2.0, -3.0}, {2.0, 1.0}, {2.0, -1.0}},
     .ball_position      = Point(4.3, -2.8),
     .ball_velocity      = Vector(),
     .first_game_state   = RefboxGameState::DIRECT_FREE_US,
     .second_game_state  = RefboxGameState::DIRECT_FREE_US},
    {.name = "Friendly direct free setup on friendly field side (goal kick)",
     .friendly_positions = {{-4.0, 0}, {-3.5, 3.0}, {-2.0, 0.0}, {-2.0, -1.0}},
     .enemy_positions    = {{4.0, 0}, {2.0, -3.0}, {2.0, 1.0}, {2.0, -1.0}},
     .ball_position      = Point(-3.5, 2.8),
     .ball_velocity      = Vector(),
     .first_game_state   = RefboxGameState::DIRECT_FREE_US,
     .second_game_state  = RefboxGameState::DIRECT_FREE_US},
    {.name               = "Enemy direct free setup on enemy field side (goal kick)",
     .friendly_positions = {{-4.0, 0}, {2.0, -3.0}, {-2.0, 0.0}, {-2.0, -1.0}},
     .enemy_positions    = {{4.0, 0}, {3.5, -3.0}, {2.0, 1.0}, {2.0, -1.0}},
     .ball_position      = Point(3.5, -2.8),
     .ball_velocity      = Vector(),
     .first_game_state   = RefboxGameState::DIRECT_FREE_THEM,
     .second_game_state  = RefboxGameState::DIRECT_FREE_THEM},
    {.name               = "Enemy direct free setup on friendly field side (corner kick)",
     .friendly_positions = {{-4.0, 0}, {2.0, -3.0}, {-2.0, 0.0}, {-2.0, -1.0}},
     .enemy_positions    = {{4.0, 0}, {-4.5, 3.0}, {2.0, 1.0}, {2.0, -1.0}},
     .ball_position      = Point(-4.3, 2.8),
     .ball_velocity      = Vector(),
     .first_game_state   = RefboxGameState::DIRECT_FREE_THEM,
     .second_game_state  = RefboxGameState::DIRECT_FREE_THEM},
    {.name               = "Friendly penalty kick",
     .friendly_positions = {{-4.0, 0}, {4.5 - 2.0, 0.0}, {0.3, 2.5}, {0.3, -2.5}},
     .enemy_positions    = {{4.5, 0}, {0, 2.5}, {0, 0}, {0, -2.5}},
     .ball_position      = Point(4.5 - 1.8, 0),
     .ball_velocity      = Vector(),
     .first_game_state   = RefboxGameState::PREPARE_PENALTY_US,
     .second_game_state  = RefboxGameState::NORMAL_START},
    {.name               = "Enemy penalty kick",
     .friendly_positions = {{-4.5, 0}, {0, 0.0}, {0, 2.5}, {0, -2.5}},
     .enemy_positions    = {{4.5, 0}, {-(4.5 - 2.0), 2.5}, {-0.3, 0}, {-0.3, -2.5}},
     .ball_position      = Point(-(4.5 - 1.8), 0),
     .ball_velocity      = Vector(),
     .first_game_state   = RefboxGameState::PREPARE_PENALTY_THEM,
     .second_game_state  = RefboxGameState::NORMAL_START},
    {.name               = "Friendly penalty kick post",
     .friendly_positions = {{-4.0, 0}, {4.5 - 2.0, 0.0}, {0.3, 2.5}, {0.3, -2.5}},
     .enemy_positions    = {{4.5, 0}, {0, 2.5}, {0, 0}, {0, -2.5}},
     .ball_position      = Point(4.5 - 1.0, 0),
     .ball_velocity      = Vector(1, 0),
     .first_game_state   = RefboxGameState::PREPARE_PENALTY_US,
     .second_game_state  = RefboxGameState::NORMAL_START},
    {.name               = "Enemy penalty kick post",
     .friendly_positions = {{-4.5, 0}, {0, 0.0}, {0, 2.5}, {0, -2.5}},
     .enemy_positions    = {{4.5, 0}, {-(4.5 - 2.0), 2.5}, {-0.3, 0}, {-0.3, -2.5}},
     .ball_position      = Point(-(4.5 - 1.0), 0),
     .ball_velocity      = Vector(-1, 0),
     .first_game_state   = RefboxGameState::PREPARE_PENALTY_THEM,
     .second_game_state  = RefboxGameState::NORMAL_START},
    {.name               = "Loose ball on their side",
     .friendly_positions = {{-4.0, 0}, {-0.5, 0}, {-2.0, 1.0}, {-2.0, -1.0}},
     .enemy_positions    = {{4.0, 0}, {0.25, 0}, {2.0, 1.0}, {2.0, 0.0}},
     .ball_position      = Point(2.0, -2.0),
     .ball_velocity      = Vector(0, -1.0),
     .first_game_state   = RefboxGameState::PREPARE_KICKOFF_THEM,
     .second_game_state  = RefboxGameState::NORMAL_START},
    {.name               = "Loose ball on our side",
     .friendly_positions = {{-4.0, 0}, {-0.5, 0}, {-2.0, 1.0}, {-2.0, -0.0}},
     .enemy_positions    = {{4.0, 0}, {0.25, 0}, {2.0, 1.0}, {2.0, 0.0}},
     .ball_position      = Point(-2.0, -2.0),
     .ball_velocity      = Vector(0, -1.0),
     .first_game_state   = RefboxGameState::PREPARE_KICKOFF_THEM,
     .second_game_state  = RefboxGameState::NORMAL_START},
    {.name               = "Their ball on our side",
     .friendly_positions = {{-4.0, 0}, {-0.5, 0}, {-2.0, 1.0}, {-2.0, -0.0}},
     .enemy_positions    = {{4.0, 0}, {-3.0, 0}, {2.0, 1.0}, {2.0, 0.0}},
     .ball_position      = Point(-3.3, 0.0),
     .ball_velocity      = Vector(-1, 0.0),
     .first_game_state   = RefboxGameState::PREPARE_KICKOFF_THEM,
     .second_game_state  = RefboxGameState::NORMAL_START},
    {.name               = "Our ball on their side",
     .friendly_positions = {{-4.0, 0}, {3.0, 0}, {-2.0, 1.0}, {-2.0, -0.0}},
     .enemy_positions    = {{4.0, 0}, {0.0, 0}, {2.0, 1.0}, {2.0, 0.0}},
     .ball_position      = Point(3.3, 0.0),
     .ball_velocity      = Vector(1, 0),
     .first_game_state   = RefboxGameState::PREPARE_KICKOFF_THEM,
     .second_game_state  = RefboxGameState::NORMAL_START}};

INSTANTIATE_TEST_CASE_P(TestPositions, STPRefboxGameStatePlaySelectionTestWithPositions,
                        ::testing::ValuesIn(test_params.begin(), test_params.end()));

int main(int argc, char** argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}