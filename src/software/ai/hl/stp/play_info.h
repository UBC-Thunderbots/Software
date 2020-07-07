#pragma once

#include <string>
#include <unordered_set>
#include <vector>

/**
 * A description of the play.  Contains the refbox game state, the name of the current
 * play being run by AI, and the tactic assignments for all the robots on the field.
 */
class PlayInfo
{
   public:
    /**
     * Constructs a PlayInfo object.
     */
    explicit PlayInfo();

    /**
     * Constructs a PlayInfo object with given arguments for fields
     */
    explicit PlayInfo(std::string refbox_game_state_name, std::string play_name,
                      std::unordered_set<std::string> robots_tactic_assignment);

    /**
     * Get refbox game state name
     *
     * @return refbox game state name
     */
    std::string getRefboxGameStateName() const;

    /**
     * Get play name
     *
     * @return play_name field
     */
    std::string getPlayName() const;

    /**
     * Get unordered set containing tactic assignments for robots
     *
     * @return robot_tactic_assignment field
     */
    std::unordered_set<std::string> getRobotTacticAssignment() const;

    /**
     * Adds new_assignment to robot_tactic_assignment
     * @param new_assignment
     */
    void addRobotTacticAssignment(std::string new_assignment);

    /**
     * Equals operator for PlayInfo objects
     *
     * @param other, PlayInfo object to be compared to
     * @return True if all fields are equal for both objects, false otherwise
     */
    bool operator==(const PlayInfo &other) const;

   private:
    std::string refbox_game_state_name;
    std::string play_name;
    std::unordered_set<std::string> robot_tactic_assignment;
};
