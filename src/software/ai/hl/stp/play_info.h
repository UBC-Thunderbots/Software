#pragma once

#include <string>
#include <unordered_set>
#include <vector>

/**
 * A description of the play.  Contains the type of play being
 * run, its name, and the tactic assignments for all the robots
 * on the field.
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
    explicit PlayInfo(std::string play_type, std::string play_name,
                      std::unordered_set<std::string> robots_tactic_assignment);

    /**
     * Get play type
     *
     * @return play_type field
     */
    std::string getPlayType() const;

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
    std::string play_type;
    std::string play_name;
    std::unordered_set<std::string> robot_tactic_assignment;
};
