#pragma once

#include <string>
#include <vector>

struct PlayInfo
{
    std::string play_type;
    std::string play_name;
    std::vector<std::string> robot_tactic_assignment;

    PlayInfo()
    {
        play_name               = "";
        play_type               = "";
        robot_tactic_assignment = {};
    }

    bool operator==(const PlayInfo& other) const
    {
        return play_type == other.play_type && play_name == other.play_name &&
               robot_tactic_assignment == other.robot_tactic_assignment;
    }
};
