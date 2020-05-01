#include "software/sensor_fusion/refbox_data.h"

#include <gtest/gtest.h>

TEST(RefboxDataTest, test_get_name_of_refbox_gamestate)
{
    for (int i = 0; i < static_cast<int>(RefboxGameState::REFBOX_GAME_STATE_COUNT); i++)
    {
        try
        {
            RefboxGameState state = static_cast<RefboxGameState>(i);
            toString(state);
        }
        catch (std::invalid_argument &)
        {
            ADD_FAILURE() << "Unable to get a name for refbox gamestate " << i
                          << std::endl;
        }
        catch (std::exception &)
        {
            ADD_FAILURE() << "Unexpected exception thrown while trying to get the refbox"
                          << " gamestate name for " << i << std::endl;
        }
    }
}


TEST(RefboxDataTest, test_getters_of_refbox_data)
{
    Timestamp packet_timestamp                 = Timestamp::fromSeconds(1.0);
    Timestamp game_state_timestamp             = Timestamp::fromSeconds(1.0);
    int game_state_counter                     = 1;
    Point designated_position                  = Point(1.0, 2.0);
    bool blue_team_on_positive_half            = true;
    Duration current_game_state_time_remaining = Duration::fromMilliseconds(1000);

    std::string friendly_team_name;
    int friendly_team_score;
    int friendly_team_red_cards;
    std::vector<int> friendly_team_yellow_card_times;
    int friendly_team_yellow_cards;
    int friendly_team_timeouts;
    int friendly_team_timeout_time;
    int friendly_team_goalkeeper;
    int friendly_team_foul_counter;
    int friendly_team_ball_placement_failures;
    bool friendly_team_can_place_ball;
    int friendly_team_max_allowed_bots;
    TeamInfo friendly_team_info(
        friendly_team_name, friendly_team_score, friendly_team_red_cards,
        friendly_team_yellow_card_times, friendly_team_yellow_cards,
        friendly_team_timeouts, friendly_team_timeout_time, friendly_team_goalkeeper,
        friendly_team_foul_counter, friendly_team_ball_placement_failures,
        friendly_team_can_place_ball, friendly_team_max_allowed_bots);

    EXPECT_EQ(friendly_team_name, friendly_team_info.getName());
    EXPECT_EQ(friendly_team_score, friendly_team_info.getScore());
    EXPECT_EQ(friendly_team_red_cards, friendly_team_info.getRedCards());
    EXPECT_EQ(friendly_team_yellow_card_times, friendly_team_info.getYellowCardTimes());
    EXPECT_EQ(friendly_team_yellow_cards, friendly_team_info.getYellowCards());
    EXPECT_EQ(friendly_team_timeouts, friendly_team_info.getTimeouts());
    EXPECT_EQ(friendly_team_timeout_time, friendly_team_info.getTimeoutTime());
    EXPECT_EQ(friendly_team_goalkeeper, friendly_team_info.getGoalkeeper());
    EXPECT_EQ(friendly_team_foul_counter, friendly_team_info.getFoulCounter());
    EXPECT_EQ(friendly_team_ball_placement_failures,
              friendly_team_info.getBallPlacementFailures());
    EXPECT_EQ(friendly_team_can_place_ball, friendly_team_info.getCanPlaceBall());
    EXPECT_EQ(friendly_team_max_allowed_bots, friendly_team_info.getMaxAllowedBots());

    std::string enemy_team_name;
    int enemy_team_score;
    int enemy_team_red_cards;
    std::vector<int> enemy_team_yellow_card_times;
    int enemy_team_yellow_cards;
    int enemy_team_timeouts;
    int enemy_team_timeout_time;
    int enemy_team_goalkeeper;
    int enemy_team_foul_counter;
    int enemy_team_ball_placement_failures;
    bool enemy_team_can_place_ball;
    int enemy_team_max_allowed_bots;
    TeamInfo enemy_team_info(enemy_team_name, enemy_team_score, enemy_team_red_cards,
                             enemy_team_yellow_card_times, enemy_team_yellow_cards,
                             enemy_team_timeouts, enemy_team_timeout_time,
                             enemy_team_goalkeeper, enemy_team_foul_counter,
                             enemy_team_ball_placement_failures,
                             enemy_team_can_place_ball, enemy_team_max_allowed_bots);

    EXPECT_EQ(enemy_team_name, enemy_team_info.getName());
    EXPECT_EQ(enemy_team_score, enemy_team_info.getScore());
    EXPECT_EQ(enemy_team_red_cards, enemy_team_info.getRedCards());
    EXPECT_EQ(enemy_team_yellow_card_times, enemy_team_info.getYellowCardTimes());
    EXPECT_EQ(enemy_team_yellow_cards, enemy_team_info.getYellowCards());
    EXPECT_EQ(enemy_team_timeouts, enemy_team_info.getTimeouts());
    EXPECT_EQ(enemy_team_timeout_time, enemy_team_info.getTimeoutTime());
    EXPECT_EQ(enemy_team_goalkeeper, enemy_team_info.getGoalkeeper());
    EXPECT_EQ(enemy_team_foul_counter, enemy_team_info.getFoulCounter());
    EXPECT_EQ(enemy_team_ball_placement_failures,
              enemy_team_info.getBallPlacementFailures());
    EXPECT_EQ(enemy_team_can_place_ball, enemy_team_info.getCanPlaceBall());
    EXPECT_EQ(enemy_team_max_allowed_bots, enemy_team_info.getMaxAllowedBots());

    RefboxGameState game_state      = RefboxGameState::STOP;
    RefboxGameState next_game_state = RefboxGameState::NORMAL_START;
    RefboxStage stage               = RefboxStage::PENALTY_SHOOTOUT;
    std::vector<GameEvent> game_events;
    std::vector<ProposedGameEvent> proposed_game_events;

    RefboxData refbox_data(packet_timestamp, game_state_timestamp, game_state_counter,
                           designated_position, blue_team_on_positive_half,
                           current_game_state_time_remaining, friendly_team_info,
                           enemy_team_info, game_state, next_game_state, stage,
                           game_events, proposed_game_events);

    EXPECT_EQ(packet_timestamp, refbox_data.getPacketTimestamp());
    EXPECT_EQ(game_state_timestamp, refbox_data.getGameStateTimestamp());
    EXPECT_EQ(game_state_counter, refbox_data.getGameStateCounter());
    EXPECT_EQ(designated_position, refbox_data.getDesignatedPosition());
    EXPECT_EQ(blue_team_on_positive_half, refbox_data.getBlueTeamOnPositiveHalf());
    EXPECT_EQ(current_game_state_time_remaining,
              refbox_data.getCurrentGameStateTimeRemaining());
    EXPECT_EQ(friendly_team_info, refbox_data.getFriendlyTeamInfo());
    EXPECT_EQ(enemy_team_info, refbox_data.getEnemyTeamInfo());
    EXPECT_EQ(game_state, refbox_data.getGameState());
    EXPECT_EQ(next_game_state, refbox_data.getNextGameState());
    EXPECT_EQ(stage, refbox_data.getStage());
}
