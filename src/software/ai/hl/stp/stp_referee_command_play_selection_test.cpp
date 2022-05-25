#include <gtest/gtest.h>

#include <exception>

#include "proto/parameters.pb.h"
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
    RefereeCommand first_game_state;
    RefereeCommand second_game_state;
};

class STPRefereeCommandPlaySelectionTestWithPositions
    : public ::testing::Test,
      public ::testing::WithParamInterface<PlaySelectionTestParams>
{
   public:
    STPRefereeCommandPlaySelectionTestWithPositions() : stp(ai_config) {}

   protected:
    STP stp;
    TbotsProto::AiConfig ai_config;
    World world = ::TestUtil::createBlankTestingWorld();
};

std::vector<PlaySelectionTestParams> test_params = {
    {.name               = "Our Kickoff",
     .friendly_positions = {{-4.0, 0}, {-0.25, 0}, {-2.0, 1.0}, {-2.0, -1.0}},
     .enemy_positions    = {{4.0, 0}, {0.5, 0}, {2.0, 1.0}, {2.0, -1.0}},
     .ball_position      = Point(0, 0),
     .ball_velocity      = Vector(),
     .first_game_state   = RefereeCommand::PREPARE_KICKOFF_US,
     .second_game_state  = RefereeCommand::PREPARE_KICKOFF_US},
    {.name               = "Their Kickoff",
     .friendly_positions = {{-4.0, 0}, {-0.5, 0}, {-2.0, 1.0}, {-2.0, -1.0}},
     .enemy_positions    = {{4.0, 0}, {0.25, 0}, {2.0, 1.0}, {2.0, -1.0}},
     .ball_position      = Point(0, 0),
     .ball_velocity      = Vector(),
     .first_game_state   = RefereeCommand::PREPARE_KICKOFF_THEM,
     .second_game_state  = RefereeCommand::PREPARE_KICKOFF_THEM},
    {.name               = "Normal play after our kickoff",
     .friendly_positions = {{-4.0, 0}, {-0.25, 0}, {-2.0, 1.0}, {-2.0, -1.0}},
     .enemy_positions    = {{4.0, 0}, {0.5, 0}, {2.0, 1.0}, {2.0, -1.0}},
     .ball_position      = Point(0, 0),
     .ball_velocity      = Vector(1, 0),
     .first_game_state   = RefereeCommand::PREPARE_KICKOFF_US,
     .second_game_state  = RefereeCommand::NORMAL_START},
    {.name               = "Normal play after their kickoff",
     .friendly_positions = {{-4.0, 0}, {-0.5, 0}, {-2.0, 1.0}, {-2.0, -1.0}},
     .enemy_positions    = {{4.0, 0}, {0.25, 0}, {2.0, 1.0}, {2.0, -1.0}},
     .ball_position      = Point(0, 0),
     .ball_velocity      = Vector(-1, 0),
     .first_game_state   = RefereeCommand::PREPARE_KICKOFF_THEM,
     .second_game_state  = RefereeCommand::NORMAL_START},
    {.name               = "Friendly indirect free setup on friendly field side",
     .friendly_positions = {{-4.0, 0}, {-2.0, 3.0}, {-2.0, 0.0}, {-2.0, -1.0}},
     .enemy_positions    = {{4.0, 0}, {0.25, 0}, {2.0, 1.0}, {2.0, -1.0}},
     .ball_position      = Point(-2.0, 2.8),
     .ball_velocity      = Vector(),
     .first_game_state   = RefereeCommand::INDIRECT_FREE_US,
     .second_game_state  = RefereeCommand::INDIRECT_FREE_US},
    {.name               = "Friendly indirect free setup on enemy field side",
     .friendly_positions = {{-4.0, 0}, {2.0, -3.0}, {-2.0, 0.0}, {-2.0, -1.0}},
     .enemy_positions    = {{4.0, 0}, {0.25, 0}, {2.0, 1.0}, {2.0, -1.0}},
     .ball_position      = Point(2.0, -2.8),
     .ball_velocity      = Vector(),
     .first_game_state   = RefereeCommand::INDIRECT_FREE_US,
     .second_game_state  = RefereeCommand::INDIRECT_FREE_US},
    // enemy indirect free on both sides of the field
    {.name               = "Enemy indirect free setup on friendly field side",
     .friendly_positions = {{-4.0, 0}, {0.25, 0}, {-2.0, 0.0}, {-2.0, -1.0}},
     .enemy_positions    = {{4.0, 0}, {-2.0, 3.0}, {2.0, 1.0}, {2.0, -1.0}},
     .ball_position      = Point(-2.0, 2.8),
     .ball_velocity      = Vector(),
     .first_game_state   = RefereeCommand::INDIRECT_FREE_THEM,
     .second_game_state  = RefereeCommand::INDIRECT_FREE_THEM},
    {.name               = "Enemy indirect free setup on enemy field side",
     .friendly_positions = {{-4.0, 0}, {0.25, 0}, {-2.0, 0.0}, {-2.0, -1.0}},
     .enemy_positions    = {{4.0, 0}, {2.0, -3.0}, {2.0, 1.0}, {2.0, -1.0}},
     .ball_position      = Point(2.0, -2.8),
     .ball_velocity      = Vector(),
     .first_game_state   = RefereeCommand::INDIRECT_FREE_THEM,
     .second_game_state  = RefereeCommand::INDIRECT_FREE_THEM},
    {.name               = "Friendly direct free setup on friendly field side",
     .friendly_positions = {{-4.0, 0}, {-4.5, -3.0}, {-2.0, 0.0}, {-2.0, -1.0}},
     .enemy_positions    = {{4.0, 0}, {2.0, -3.0}, {2.0, 1.0}, {2.0, -1.0}},
     .ball_position      = Point(-4.3, -2.8),
     .ball_velocity      = Vector(),
     .first_game_state   = RefereeCommand::DIRECT_FREE_US,
     .second_game_state  = RefereeCommand::DIRECT_FREE_US},
    {.name               = "Friendly direct free setup on enemy field side",
     .friendly_positions = {{-4.0, 0}, {4.5, 3.0}, {-2.0, 0.0}, {-2.0, -1.0}},
     .enemy_positions    = {{4.0, 0}, {2.0, -3.0}, {2.0, 1.0}, {2.0, -1.0}},
     .ball_position      = Point(4.3, 2.8),
     .ball_velocity      = Vector(),
     .first_game_state   = RefereeCommand::DIRECT_FREE_US,
     .second_game_state  = RefereeCommand::DIRECT_FREE_US},
    {.name               = "Enemy direct free setup on friendly field side",
     .friendly_positions = {{-4.0, 0}, {2.0, -3.0}, {-2.0, 0.0}, {-2.0, -1.0}},
     .enemy_positions    = {{4.0, 0}, {-4.5, -3.0}, {2.0, 1.0}, {2.0, -1.0}},
     .ball_position      = Point(-4.3, -2.8),
     .ball_velocity      = Vector(),
     .first_game_state   = RefereeCommand::DIRECT_FREE_THEM,
     .second_game_state  = RefereeCommand::DIRECT_FREE_THEM},
    {.name               = "Enemy direct free setup on enemy field side",
     .friendly_positions = {{-4.0, 0}, {2.0, -3.0}, {-2.0, 0.0}, {-2.0, -1.0}},
     .enemy_positions    = {{4.0, 0}, {4.5, 3.0}, {2.0, 1.0}, {2.0, -1.0}},
     .ball_position      = Point(4.3, 2.8),
     .ball_velocity      = Vector(),
     .first_game_state   = RefereeCommand::DIRECT_FREE_THEM,
     .second_game_state  = RefereeCommand::DIRECT_FREE_THEM},
    {.name               = "Friendly penalty kick",
     .friendly_positions = {{-4.0, 0}, {4.5 - 2.0, 0.0}, {0.3, 2.5}, {0.3, -2.5}},
     .enemy_positions    = {{4.5, 0}, {0, 2.5}, {0, 0}, {0, -2.5}},
     .ball_position      = Point(4.5 - 1.8, 0),
     .ball_velocity      = Vector(),
     .first_game_state   = RefereeCommand::PREPARE_PENALTY_US,
     .second_game_state  = RefereeCommand::NORMAL_START},
    {.name               = "Enemy penalty kick",
     .friendly_positions = {{-4.5, 0}, {0, 0.0}, {0, 2.5}, {0, -2.5}},
     .enemy_positions    = {{4.5, 0}, {-(4.5 - 2.0), 2.5}, {-0.3, 0}, {-0.3, -2.5}},
     .ball_position      = Point(-(4.5 - 1.8), 0),
     .ball_velocity      = Vector(),
     .first_game_state   = RefereeCommand::PREPARE_PENALTY_THEM,
     .second_game_state  = RefereeCommand::NORMAL_START},
    {.name               = "Friendly penalty kick post",
     .friendly_positions = {{-4.0, 0}, {4.5 - 2.0, 0.0}, {0.3, 2.5}, {0.3, -2.5}},
     .enemy_positions    = {{4.5, 0}, {0, 2.5}, {0, 0}, {0, -2.5}},
     .ball_position      = Point(4.5 - 1.0, 0),
     .ball_velocity      = Vector(1, 0),
     .first_game_state   = RefereeCommand::PREPARE_PENALTY_US,
     .second_game_state  = RefereeCommand::NORMAL_START},
    {.name               = "Enemy penalty kick post",
     .friendly_positions = {{-4.5, 0}, {0, 0.0}, {0, 2.5}, {0, -2.5}},
     .enemy_positions    = {{4.5, 0}, {-(4.5 - 2.0), 2.5}, {-0.3, 0}, {-0.3, -2.5}},
     .ball_position      = Point(-(4.5 - 1.0), 0),
     .ball_velocity      = Vector(-1, 0),
     .first_game_state   = RefereeCommand::PREPARE_PENALTY_THEM,
     .second_game_state  = RefereeCommand::NORMAL_START}};

INSTANTIATE_TEST_CASE_P(TestPositions, STPRefereeCommandPlaySelectionTestWithPositions,
                        ::testing::ValuesIn(test_params.begin(), test_params.end()));

class STPRefereeCommandPlaySelectionTest
    : public ::testing::Test,
      public ::testing::WithParamInterface<RefereeCommand>
{
   public:
    STPRefereeCommandPlaySelectionTest() : stp(ai_config) {}

   protected:
    void SetUp() override
    {
        // Give an explicit seed to STP so that our tests are deterministic
        stp = STP(ai_config);

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

    TbotsProto::AiConfig ai_config;
    STP stp;
    World world = ::TestUtil::createBlankTestingWorld();
};

// TODO (Issue #1999): Include `BALL_PLACEMENT_THEM` when ball placement states have plays
// NORMAL_START is omitted since there is no preceding PREPARE state
// GOAL_US and GOAL_THEM are omitted since selecting a play is not applicable
INSTANTIATE_TEST_CASE_P(
    AllRefboxGameStates, STPRefereeCommandPlaySelectionTest,
    ::testing::Values(RefereeCommand::HALT, RefereeCommand::STOP,
                      RefereeCommand::FORCE_START, RefereeCommand::PREPARE_KICKOFF_US,
                      RefereeCommand::PREPARE_KICKOFF_THEM,
                      RefereeCommand::PREPARE_PENALTY_US,
                      RefereeCommand::PREPARE_PENALTY_THEM,
                      RefereeCommand::DIRECT_FREE_US, RefereeCommand::DIRECT_FREE_THEM,
                      RefereeCommand::INDIRECT_FREE_US,
                      RefereeCommand::INDIRECT_FREE_THEM, RefereeCommand::TIMEOUT_US,
                      RefereeCommand::TIMEOUT_THEM, RefereeCommand::BALL_PLACEMENT_US));
