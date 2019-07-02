#include <gtest/gtest.h>

#include <exception>

#include "ai/hl/stp/play/halt_play.h"
#include "ai/hl/stp/stp.h"
#include "ai/world/world.h"
#include "test/test_util/test_util.h"


struct PlaySelectionTestParams {
    std::vector<Point> friendly_positions;
    std::vector<Point> enemy_positions;
    Point ball_position;
    RefboxGameState game_state;
};


Team createTeamFromPoints(const std::vector<Point>& points) {
    std::vector<Robot> team_robots;
    team_robots.reserve(points.size());
    unsigned id = 0;
    std::transform(points.begin(), points.end(),
                   std::back_inserter(team_robots), [&id](const Point& point) {
                return Robot(id++, point, Point(), Angle(), AngularVelocity(), Timestamp());
            });
}

class STPRefboxGameStatePlaySelectionTestWithPositions :
        public ::testing::Test,
        public ::testing::WithParamInterface<PlaySelectionTestParams> {
public:
    STPRefboxGameStatePlaySelectionTestWithPositions() : stp([]() {return std::make_unique<HaltPlay>();}, 0) {}
protected:
    void SetUp() override {
        world.mutableField() = ::Test::TestUtil::createSSLDivBField();
    }

    STP stp;
    World world;
};

TEST_P(STPRefboxGameStatePlaySelectionTestWithPositions, test_play_selection_for_states_and_positions) {
    // set up the friendly team
    world.mutableFriendlyTeam() = createTeamFromPoints(GetParam().friendly_positions);
    world.mutableEnemyTeam() = createTeamFromPoints(GetParam().enemy_positions);
    world.mutableBall() = Ball(GetParam().ball_position, Vector(), Timestamp());
    world.mutableGameState().updateRefboxGameState(GetParam().game_state);
    try {
        stp.calculateNewPlay(world);
    } catch (const std::runtime_error& e) {
        FAIL() << "No play for RefboxGameState " << GetParam().game_state << " and ball pos "
               << GetParam().ball_position;
    }
}

std::vector<PlaySelectionTestParams> test_params = {
        {
            .friendly_positions = {},
            .enemy_positions = {},
            .ball_position = Point(),
            .game_state = RefboxGameState::HALT
        }
};

class STPRefboxGameStatePlaySelectionTest
        : public ::testing::Test,
          public ::testing::WithParamInterface<RefboxGameState> {
public:
    STPRefboxGameStatePlaySelectionTest() : stp([]() { return nullptr; }) {}

protected:
    void SetUp() override {
        auto default_play_constructor = []() -> std::unique_ptr<Play> {
            return std::make_unique<HaltPlay>();
        };
        // Give an explicit seed to STP so that our tests are deterministic
        stp = STP(default_play_constructor, 0);
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
       test_play_selection_for_all_refbox_game_states) {
    world.mutableGameState().updateRefboxGameState(GetParam());

    try {
        auto play_ptr = stp.calculateNewPlay(world);
    }
    catch (...) {
        FAIL() << "No play for game state " << GetParam();
    }
}

auto all_refbox_game_states = ::Test::TestUtil::getAllRefboxGameStates();

INSTANTIATE_TEST_CASE_P(AllRefboxGameStates, STPRefboxGameStatePlaySelectionTest,
                        ::testing::ValuesIn(all_refbox_game_states.begin(),
                                            all_refbox_game_states.end()));

int main(int argc, char **argv) {
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}