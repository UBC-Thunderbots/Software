#pragma once

#include <string>
#include <vector>

class PlayInfo
{
   public:
    explicit PlayInfo();
    bool operator==(const PlayInfo& other) const;

   private:
    std::string play_type;
    std::string play_name;
    std::vector<std::string> robot_tactic_assignment;
};
