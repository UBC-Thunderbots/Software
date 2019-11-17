#include "software/ai/hl/stp/play_info.h"

PlayInfo::PlayInfo() : play_type(""), play_name(""), robot_tactic_assignment({}) {}

bool PlayInfo::operator==(const PlayInfo &other) const
{
    return play_type == other.play_type && play_name == other.play_name &&
           robot_tactic_assignment == other.robot_tactic_assignment;
}