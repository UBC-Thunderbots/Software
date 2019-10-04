#include "software/ai/move_rule/move_rule.h"

#include <map>
#include <string>

std::ostream& operator<<(std::ostream& os, const MoveRule& rule)
{
    static const std::map<MoveRule, std::string> move_rule_names = {
        {MoveRule::ENEMY_ROBOTS_COLLISION, "ENEMY_ROBOTS_COLLISION"},
        {MoveRule::FRIENDLY_DEFENSE_AREA, "FRIENDLY_DEFENSE_AREA"},
        {MoveRule::ENEMY_DEFENSE_AREA, "ENEMY_DEFENSE_AREA"},
        {MoveRule::INFLATED_ENEMY_DEFENSE_AREA, "INFLATED_ENEMY_DEFENSE_AREA"},
        {MoveRule::CENTER_CIRCLE, "CENTER_CIRCLE"},
        {MoveRule::HALF_METER_AROUND_BALL, "HALF_METER_AROUND_BALL"},
        {MoveRule::ENEMY_HALF, "ENEMY_HALF"},
        {MoveRule::FRIENDLY_HALF, "FRIENDLY_HALF"}};
    
    auto iter = move_rule_names.find(rule);
    if (iter != move_rule_names.end())
    {
        os << iter->second;
    }
    else
    {
        os << "No String Representation For MoveRule: " << static_cast<int>(rule);
    }
    return os;
}

