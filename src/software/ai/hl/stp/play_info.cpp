#include "software/ai/hl/stp/play_info.h"

PlayInfo::PlayInfo() : play_type(""), play_name(""), robot_tactic_assignment({}) {}

PlayInfo::PlayInfo(std::string play_type, std::string play_name,
                   std::unordered_set<std::string> robot_tactic_assignment)
    : play_type(play_type),
      play_name(play_name),
      robot_tactic_assignment(robot_tactic_assignment)
{
}

std::string PlayInfo::getPlayType() const
{
    return play_type;
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
    return this->play_type == other.play_type && this->play_name == other.play_name &&
           this->robot_tactic_assignment == other.robot_tactic_assignment;
}
