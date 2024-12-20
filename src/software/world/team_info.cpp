#include "software/world/team_info.h"

TeamInfo::TeamInfo()
    : name_(""), score_(0), red_cards_(0), yellow_cards_(0), fouls_count_(0)
{
}

TeamInfo::TeamInfo(const TbotsProto::GameState::TeamInfo& team_info_proto)
    : name_(team_info_proto.name()),
      score_(team_info_proto.score()),
      red_cards_(team_info_proto.red_cards()),
      yellow_cards_(team_info_proto.yellow_cards()),
      fouls_count_(team_info_proto.fouls_count())
{
    std::transform(team_info_proto.yellow_card_times().begin(),
                   team_info_proto.yellow_card_times().end(),
                   std::back_inserter(yellow_card_times_),
                   [](unsigned int microseconds) {
                       return Duration::fromMicroseconds(microseconds);
                   });
}

const std::string& TeamInfo::getName() const
{
    return name_;
}

unsigned int TeamInfo::getScore() const
{
    return score_;
}

unsigned int TeamInfo::getRedCards() const
{
    return red_cards_;
}

const std::vector<Duration>& TeamInfo::getYellowCardTimes() const
{
    return yellow_card_times_;
}

unsigned int TeamInfo::getYellowCards() const
{
    return yellow_cards_;
}

unsigned int TeamInfo::getFoulsCount() const
{
    return fouls_count_;
}

bool TeamInfo::operator==(const TeamInfo& other) const
{
    return name_ == other.name_ && score_ == other.score_ &&
           red_cards_ == other.red_cards_ &&
           yellow_card_times_ == other.yellow_card_times_ &&
           yellow_cards_ == other.yellow_cards_ && fouls_count_ == other.fouls_count_;
}
