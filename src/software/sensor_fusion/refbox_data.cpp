#include "software/sensor_fusion/refbox_data.h"

#include <magic_enum/magic_enum.hpp>

std::string name(const RefboxGameState& state)
{
    std::string find_name_result = static_cast<std::string>(magic_enum::enum_name(state));
    return find_name_result;
}

std::ostream& operator<<(std::ostream& os, const RefboxGameState& state)
{
    os << name(state);
    return os;
}

std::string name(const RefboxStage& stage)
{
    std::string find_name_result = static_cast<std::string>(magic_enum::enum_name(stage));
    return find_name_result;
}

std::ostream& operator<<(std::ostream& os, const RefboxStage& stage)
{
    os << name(stage);
    return os;
}

TeamInfo::TeamInfo() {}
TeamInfo::TeamInfo(std::string name, int score, int red_cards,
                   std::vector<int> yellow_card_times, int yellow_cards, int timeouts,
                   int timeout_time, int goalkeeper, int foul_counter,
                   int ball_placement_failures, bool can_place_ball, int max_allowed_bots)
    : name(name),
      score(score),
      red_cards(red_cards),
      yellow_card_times(yellow_card_times),
      yellow_cards(yellow_cards),
      timeouts(timeouts),
      timeout_time(timeout_time),
      goalkeeper(goalkeeper),
      foul_counter(foul_counter),
      ball_placement_failures(ball_placement_failures),
      can_place_ball(can_place_ball),
      max_allowed_bots(max_allowed_bots)
{
}

std::string TeamInfo::getName(void) const
{
    return name;
}

int TeamInfo::getScore(void) const
{
    return score;
}

int TeamInfo::getRedCards(void) const
{
    return red_cards;
}

std::vector<int> TeamInfo::getYellowCardTimes(void) const
{
    return yellow_card_times;
}

int TeamInfo::getYellowCards(void) const
{
    return yellow_cards;
}

int TeamInfo::getTimeouts(void) const
{
    return timeouts;
}

int TeamInfo::getTimeoutTime(void) const
{
    return timeout_time;
}

int TeamInfo::getGoalkeeper(void) const
{
    return goalkeeper;
}

int TeamInfo::getFoulCounter(void) const
{
    return foul_counter;
}

int TeamInfo::getBallPlacementFailures(void) const
{
    return ball_placement_failures;
}

bool TeamInfo::getCanPlaceBall(void) const
{
    return can_place_ball;
}

int TeamInfo::getMaxAllowedBots(void) const
{
    return max_allowed_bots;
}
