#include "ai/intent/avoid_area.h"

#include <map>
#include <string>

static const std::map<AvoidArea, std::string> avoid_area_names = {
    {AvoidArea::FRIENDLY_DEFENSE_AREA, "FRIENDLY_DEFENSE_AREA"},
    {AvoidArea::ENEMY_DEFENSE_AREA, "ENEMY_DEFENSE_AREA"},
    {AvoidArea::INFLATED_ENEMY_DEFENSE_AREA, "INFLATED_ENEMY_DEFENSE_AREA"},
    {AvoidArea::CENTER_CIRCLE, "CENTER_CIRCLE"},
    {AvoidArea::HALF_METER_AROUND_BALL, "HALF_METER_AROUND_BALL"},
    {AvoidArea::ENEMY_HALF, "ENEMY_HALF"},
    {AvoidArea::FRIENDLY_HALF, "FRIENDLY_HALF"}};

std::ostream& operator<<(std::ostream& os, const AvoidArea& area)
{
    auto iter = avoid_area_names.find(area);
    if (iter != avoid_area_names.end())
    {
        os << iter->second;
    }
    else
    {
        os << "No String Representation For AvoidArea: " << static_cast<int>(area);
    }
    return os;
}
