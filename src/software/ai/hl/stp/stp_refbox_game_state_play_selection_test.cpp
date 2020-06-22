#include <gtest/gtest.h>

#include <exception>

#include "software/ai/hl/stp/play/halt_play.h"
#include "software/ai/hl/stp/stp.h"
#include "software/test_util/test_util.h"
#include "software/world/world.h"


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
        : stp([]() { return std::make_unique<HaltPlay>(); },
              std::make_shared<const AIControlConfig>(), 0)
    {
    }

   protected:
    STP stp;
    World world = ::TestUtil::createBlankTestingWorld();
};

TEST_P(STPRefboxGameStatePlaySelectionTestWithPositions,
       test_play_selection_for_states_and_positions)
{
    // set up the friendly team
    ::TestUtil::setFriendlyRobotPositions(world, GetParam().friendly_positions,
                                          Timestamp());
    ::TestUtil::setEnemyRobotPositions(world, GetParam().enemy_positions, Timestamp());
    world.updateBallStateWithTimestamp(
        TimestampedBallState(GetParam().ball_position, Vector(), Timestamp()));

    // to set restart reason, etc. properly
    world.mutableGameState().updateRefboxGameState(GetParam().first_game_state);
    world.mutableGameState().updateBall(world.ball());
    world.mutableGameState().updateRefboxGameState(GetParam().second_game_state);
    world.mutableGameState().updateBall(world.ball());
    std::unique_ptr<Play> play;
    try
    {
        play = stp.calculateNewPlay(world);
    }
    catch (const std::runtime_error& e)
    {
        FAIL() << "No play for test case: " + GetParam().name;
    }
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
    {.name               = "Friendly direct free setup on friendly field side",
     .friendly_positions = {{-4.0, 0}, {-4.5, -3.0}, {-2.0, 0.0}, {-2.0, -1.0}},
     .enemy_positions    = {{4.0, 0}, {2.0, -3.0}, {2.0, 1.0}, {2.0, -1.0}},
     .ball_position      = Point(-4.3, -2.8),
     .ball_velocity      = Vector(),
     .first_game_state   = RefboxGameState::DIRECT_FREE_US,
     .second_game_state  = RefboxGameState::DIRECT_FREE_US},
    {.name               = "Friendly direct free setup on enemy field side",
     .friendly_positions = {{-4.0, 0}, {4.5, 3.0}, {-2.0, 0.0}, {-2.0, -1.0}},
     .enemy_positions    = {{4.0, 0}, {2.0, -3.0}, {2.0, 1.0}, {2.0, -1.0}},
     .ball_position      = Point(4.3, 2.8),
     .ball_velocity      = Vector(),
     .first_game_state   = RefboxGameState::DIRECT_FREE_US,
     .second_game_state  = RefboxGameState::DIRECT_FREE_US},
    {.name               = "Enemy direct free setup on friendly field side",
     .friendly_positions = {{-4.0, 0}, {2.0, -3.0}, {-2.0, 0.0}, {-2.0, -1.0}},
     .enemy_positions    = {{4.0, 0}, {-4.5, -3.0}, {2.0, 1.0}, {2.0, -1.0}},
     .ball_position      = Point(-4.3, -2.8),
     .ball_velocity      = Vector(),
     .first_game_state   = RefboxGameState::DIRECT_FREE_THEM,
     .second_game_state  = RefboxGameState::DIRECT_FREE_THEM},
    {.name               = "Enemy direct free setup on enemy field side",
     .friendly_positions = {{-4.0, 0}, {2.0, -3.0}, {-2.0, 0.0}, {-2.0, -1.0}},
     .enemy_positions    = {{4.0, 0}, {4.5, 3.0}, {2.0, 1.0}, {2.0, -1.0}},
     .ball_position      = Point(4.3, 2.8),
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
     .second_game_state  = RefboxGameState::NORMAL_START}};

// TODO (Issue #1330): Reenable these tests
// INSTANTIATE_TEST_CASE_P(TestPositions,
// STPRefboxGameStatePlaySelectionTestWithPositions,
//                        ::testing::ValuesIn(test_params.begin(), test_params.end()));

class STPRefboxGameStatePlaySelectionTest
    : public ::testing::Test,
      public ::testing::WithParamInterface<RefboxGameState>
{
   public:
    STPRefboxGameStatePlaySelectionTest()
        : stp([]() { return nullptr; }, std::make_shared<const AIControlConfig>(), 0)
    {
    }

   protected:
    void SetUp() override
    {
        auto default_play_constructor = []() -> std::unique_ptr<Play> {
            return std::make_unique<HaltPlay>();
        };
        // Give an explicit seed to STP so that our tests are deterministic
        stp = STP(default_play_constructor, 0);

        Robot robot_0(0, Point(-1.1, 1), Vector(), Angle::zero(), AngularVelocity::zero(),
                      Timestamp::fromSeconds(0));
        Robot robot_1(1, Point(2, 0.81), Vector(), Angle::zero(), AngularVelocity::zero(),
                      Timestamp::fromSeconds(0));
        Robot robot_2(2, Point(0, 5.0), Vector(), Angle::zero(), AngularVelocity::zero(),
                      Timestamp::fromSeconds(0));
        world.updateFriendlyTeamState(Team({robot_0, robot_1, robot_2}));

        Robot enemy_robot_0(0, Point(1.1, 1), Vector(), Angle::zero(),
                            AngularVelocity::zero(), Timestamp::fromSeconds(0));
        Robot enemy_robot_1(1, Point(-2, 0.81), Vector(), Angle::zero(),
                            AngularVelocity::zero(), Timestamp::fromSeconds(0));
        Robot enemy_robot_2(2, Point(0, -5.0), Vector(), Angle::zero(),
                            AngularVelocity::zero(), Timestamp::fromSeconds(0));
        world.updateEnemyTeamState(Team({enemy_robot_0, enemy_robot_1, enemy_robot_2}));
    }

    STP stp;
    World world = ::TestUtil::createBlankTestingWorld();
};

TEST_P(STPRefboxGameStatePlaySelectionTest,
       test_play_selection_for_all_refbox_game_states)
{
    // TODO (Issue #1330): replace the ball with real parameterized values
    world.mutableGameState().updateRefboxGameState(GetParam());
    world.mutableGameState().updateBall(
        Ball(Point(), Vector(), Timestamp::fromSeconds(0)));

    try
    {
        auto play_ptr = stp.calculateNewPlay(world);
    }
    catch (...)
    {
        FAIL() << "No play for game state " << GetParam();
    }
}

auto all_refbox_game_states = ::TestUtil::getAllRefboxGameStates();

// TODO (Issue #1330): Reenable these tests
// INSTANTIATE_TEST_CASE_P(AllRefboxGameStates, STPRefboxGameStatePlaySelectionTest,
//                        ::testing::ValuesIn(all_refbox_game_states.begin(),
//                                            all_refbox_game_states.end()));
