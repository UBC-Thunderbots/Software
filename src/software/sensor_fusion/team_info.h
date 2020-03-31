#pragma once

#include <string>
#include <vector>

class TeamInfo
{
   public:
    /**
     * Default constructor
     */
    TeamInfo();

    /**
     * Constructor that initializes all fields
     *
     * @param name name
     * @param score score
     * @param red_cards red_cards
     * @param yellow_card_times yellow_card_times
     * @param yellow_cards yellow_cards
     * @param timeouts timeouts
     * @param timeout_time timeout_time
     * @param goalkeeper goalkeeper
     * @param foul_counter foul_counter
     * @param ball_placement_failures ball_placement_failures
     * @param can_place_ball can_place_ball
     * @param max_allowed_bots max_allowed_bots
     */
    TeamInfo(std::string name, int score, int red_cards,
             std::vector<int> yellow_card_times, int yellow_cards, int timeouts,
             int timeout_time, int goalkeeper, int foul_counter,
             int ball_placement_failures, bool can_place_ball, int max_allowed_bots);

    /**
     * Return name
     *
     * @return name
     */
    std::string getName(void) const;

    /**
     * Return score
     *
     * @return score
     */
    int getScore(void) const;

    /**
     * Return number of red cards
     *
     * @return number of red cards
     */
    int getRedCards(void) const;

    /**
     * Return yellow card times
     *
     * @return yellow card times
     */
    std::vector<int> getYellowCardTimes(void) const;

    /**
     * Return number of red cards
     *
     * @return number of red cards
     */
    int getYellowCards(void) const;

    /**
     * Return number of timeouts
     *
     * @return number of timeouts
     */
    int getTimeouts(void) const;

    /**
     * Return timeout time
     *
     * @return timeout time
     */
    int getTimeoutTime(void) const;

    /**
     * Return goalkeeper
     *
     * @return goalkeeper
     */
    int getGoalkeeper(void) const;

    /**
     * Return foul counter
     *
     * @return foul counter
     */
    int getFoulCounter(void) const;

    /**
     * Return ball placement failures
     *
     * @return ball placement failures
     */
    int getBallPlacementFailures(void) const;

    /**
     * Return can place ball
     *
     * @return can place ball
     */
    bool getCanPlaceBall(void) const;

    /**
     * Return max allowed bots
     *
     * @return max allowed bots
     */
    int getMaxAllowedBots(void) const;

    bool operator==(const TeamInfo &other) const;

   private:
    // The team's name (empty string if operator has not typed anything).
    std::string name;
    // The number of goals scored by the team during normal play and overtime.
    int score;
    // The number of red cards issued to the team since the beginning of the game.
    int red_cards;
    // The amount of time (in microseconds) left on each yellow card issued to the team.
    // If no yellow cards are issued, this array has no elements.
    // Otherwise, times are ordered from smallest to largest.
    std::vector<int> yellow_card_times;
    // The total number of yellow cards ever issued to the team.
    int yellow_cards;
    // The number of timeouts this team can still call.
    // If in a timeout right now, that timeout is excluded.
    int timeouts;
    // The number of microseconds of timeout this team can use.
    int timeout_time;
    // The pattern number of this team's goalkeeper.
    int goalkeeper;
    // The total number of countable fouls that act towards yellow cards
    int foul_counter;
    // The number of consecutive ball placement failures of this team
    int ball_placement_failures;
    // Indicate if the team is able and allowed to place the ball
    bool can_place_ball;
    // The maximum number of bots allowed on the field based on division and cards
    int max_allowed_bots;
};
