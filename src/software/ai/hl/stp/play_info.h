#pragma once

#include <string>
#include <vector>

class PlayInfo
{
   public:
    /**
     * Constructs a PlayInfo object.
     */
    explicit PlayInfo();

    /**
     * @return play_type field
     */
    std::string getPlayType() const;

    /**
     * @return play_name field
     */
    std::string getPlayName() const;

    /**
     * @return robot_tactic_assignment field
     */
    std::vector<std::string> getRobotTacticAssignment() const;

    /**
     * Sets the play_type field to new_play_type
     * @param new_play_type, the new play type to be set to
     */
    void setPlayType(std::string new_play_type);

    /**
     * Sets the play_name field to new_play_name
     * @param new_play_name, the new play name to be set to
     */
    void setPlayName(std::string new_play_name);

    /**
     * Sets the robot_tactic_assignment field to rta
     * @param rta, the new vector containing robot assignments
     */
    void setRobotTacticAssignment(std::vector<std::string> rta);

    /**
     * Adds new_assignment to robot_tactic_assignment
     * @param new_assignment
     */
    void addAssignment(std::string new_assignment);

    bool operator==(const PlayInfo& other) const;

   private:
    std::string play_type;
    std::string play_name;
    std::vector<std::string> robot_tactic_assignment;
};
