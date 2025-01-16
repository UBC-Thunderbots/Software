#pragma once

#include <string>
#include <vector>

#include "proto/game_state.pb.h"
#include "software/time/duration.h"

/**
 * Stores game controller information about a team, such as the team name, score,
 * red cards, yellow cards, fouls, etc.
 */
class TeamInfo
{
   public:
    /**
     * Constructs a TeamInfo with default field values.
     */
    TeamInfo();

    /**
     * Constructs a TeamInfo object from a TeamInfo protobuf message.
     *
     * @param team_info_proto the TeamInfo protobuf message containing
     * the data to initialize this object.
     */
    explicit TeamInfo(const TbotsProto::GameState::TeamInfo& team_info_proto);

    /**
     * Gets the name of the team.
     *
     * @return the name of the team
     */
    const std::string& getName() const;

    /**
     * Gets the number of goals scored by the team during normal play and overtime.
     *
     * @return the team's score
     */
    unsigned int getScore() const;

    /**
     * Gets the number of red cards issued to the team since the beginning of the game.
     *
     * @return the number of red cards issued to the team
     */
    unsigned int getRedCards() const;

    /**
     * Gets the total number of yellow cards issued to the team since the beginning of
     * the game.
     *
     * @return the number of yellow cards issued to the team
     */
    unsigned int getYellowCards() const;

    /**
     * Gets the amount of time left on each active yellow card issued to the team.
     * If the team currently has no yellow cards, the returned vector has no elements.
     *
     * @return a vector of durations representing the time remaining on each
     * active yellow card
     */
    const std::vector<Duration>& getYellowCardTimes() const;

    /**
     * Gets the total number of countable fouls that could lead to yellow cards.
     *
     * @return the total number of fouls committed by the team
     */
    unsigned int getFoulsCount() const;

    /**
     * Equality operator for comparing two TeamInfo objects.
     *
     * @param other the TeamInfo object to compare against.
     * @return true if all fields of this TeamInfo are equal to those of other,
     * false otherwise
     */
    bool operator==(const TeamInfo& other) const;

   private:
    std::string name_;
    unsigned int score_;
    unsigned int red_cards_;
    unsigned int yellow_cards_;
    std::vector<Duration> yellow_card_times_;
    unsigned int fouls_count_;
};
