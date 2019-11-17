#include "software/ai/hl/stp/play_info.h"

PlayInfo::PlayInfo() : play_type(""), play_name(""), robot_tactic_assignment({}) {}

std::string PlayInfo::getPlayType() const
{
    return play_type;
}

std::string PlayInfo::getPlayName() const
{
    return play_name;
}

std::vector<std::string> PlayInfo::getRobotTacticAssignment() const
{
    return robot_tactic_assignment;
}

void PlayInfo::setPlayType(std::string new_play_type)
{
    this->play_type = new_play_type;
}

void PlayInfo::setPlayName(std::string new_play_name)
{
    this->play_name = new_play_name;
}

void PlayInfo::setRobotsTacticAssignment(std::vector<std::string> rta)
{
    this->robot_tactic_assignment = rta;
}

void PlayInfo::addAssignment(std::string new_assignment)
{
    this->robot_tactic_assignment.emplace_back(new_assignment);
}

bool PlayInfo::operator==(const PlayInfo &other) const
{
    return this->play_type == other.play_type && this->play_name == other.play_name &&
           this->robot_tactic_assignment == other.robot_tactic_assignment;
}
