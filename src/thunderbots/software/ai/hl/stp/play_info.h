#pragma once

#include <string>

struct PlayInfo
{
    std::string play_type;
    std::string play_name;
    std::vector<std::string> tactics;

    PlayInfo()
    {
        play_name = "";
        play_type = "";
        tactics   = {};
    }
};
