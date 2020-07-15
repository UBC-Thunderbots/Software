#include "software/ai/hl/stp/play_info.h"

PlayInfo::PlayInfo()
    : refbox_game_state_name(""), play_name(""), robot_tactic_assignment({})
{
}

PlayInfo::PlayInfo(std::string refbox_game_state_name, std::string play_name,
                   std::unordered_set<std::string> robot_tactic_assignment)
    : refbox_game_state_name(refbox_game_state_name),
      play_name(play_name),
      robot_tactic_assignment(robot_tactic_assignment)
{
}

std::string PlayInfo::getRefboxGameStateName() const
{
    return refbox_game_state_name;
}

std::string PlayInfo::getPlayName() const
{
    return play_name;
}

std::unordered_set<std::string> PlayInfo::getRobotTacticAssignment() const
{
    return robot_tactic_assignment;
}

void PlayInfo::addRobotTacticAssignment(std::string new_assignment)
{
    this->robot_tactic_assignment.emplace(new_assignment);
}

bool PlayInfo::operator==(const PlayInfo &other) const
{
    return this->refbox_game_state_name == other.refbox_game_state_name &&
           this->play_name == other.play_name &&
           this->robot_tactic_assignment == other.robot_tactic_assignment;
}
