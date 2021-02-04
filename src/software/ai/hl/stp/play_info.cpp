#include "software/ai/hl/stp/play_info.h"

PlayInfo::PlayInfo(std::string referee_command_name, std::string play_name,
                   std::vector<std::string> robot_tactic_assignment)
    : referee_command_name(referee_command_name),
      play_name(play_name),
      robot_tactic_assignment(robot_tactic_assignment)
{
}

std::string PlayInfo::getRefereeCommandName() const
{
    return referee_command_name;
}

std::string PlayInfo::getPlayName() const
{
    return play_name;
}

std::vector<std::string> PlayInfo::getRobotTacticAssignment() const
{
    return robot_tactic_assignment;
}

void PlayInfo::addRobotTacticAssignment(std::string new_assignment)
{
    this->robot_tactic_assignment.push_back(new_assignment);
}

bool PlayInfo::operator==(const PlayInfo &other) const
{
    return this->referee_command_name == other.referee_command_name &&
           this->play_name == other.play_name &&
           this->robot_tactic_assignment == other.robot_tactic_assignment;
}
