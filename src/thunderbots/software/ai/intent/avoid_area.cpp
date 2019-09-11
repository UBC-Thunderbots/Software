#include "ai/intent/avoid_area.h"

#include <map>
#include <string>

static const std::map<AvoidArea, std::string> avoid_area_names = {
    {AvoidArea::ENEMY_ROBOTS, "ENEMY_ROBOTS"},
    {AvoidArea::FRIENDLY_DEFENSE_AREA, "FRIENDLY_DEFENSE_AREA"},
    {AvoidArea::ENEMY_DEFENSE_AREA, "ENEMY_DEFENSE_AREA"},
    {AvoidArea::INFLATED_ENEMY_DEFENSE_AREA, "INFLATED_ENEMY_DEFENSE_AREA"},
    {AvoidArea::CENTER_CIRCLE, "CENTER_CIRCLE"},
    {AvoidArea::HALF_METER_AROUND_BALL, "HALF_METER_AROUND_BALL"},
    {AvoidArea::BALL, "BALL"},
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

avoid_area_mask_t getAvoidAreasFromGameState(const GameState& game_state)
{
    avoid_area_mask_t areas_to_avoid = 0;

    // default avoid areas regardless of game state
    areas_to_avoid.set((uint32_t)AvoidArea::FRIENDLY_DEFENSE_AREA, true);
    areas_to_avoid.set((uint32_t)AvoidArea::ENEMY_ROBOTS, true);

    if (game_state.stayAwayFromBall())
    {
        areas_to_avoid.set((uint32_t)AvoidArea::HALF_METER_AROUND_BALL, true);
    }

    if (game_state.isPenalty())
    {
        if (game_state.isOurPenalty())
        {
            areas_to_avoid.set((uint32_t)AvoidArea::ENEMY_HALF, true);
        }
        else
        {
            // Is their penalty
            areas_to_avoid.set((uint32_t)AvoidArea::FRIENDLY_HALF, true);
        }
    }
    else if (game_state.isKickoff())
    {
        areas_to_avoid.set((uint32_t)AvoidArea::HALF_METER_AROUND_BALL, true);
        areas_to_avoid.set((uint32_t)AvoidArea::CENTER_CIRCLE, true);
        areas_to_avoid.set((uint32_t)AvoidArea::ENEMY_HALF, true);
    }
    else
    {
        if (game_state.stayAwayFromBall() || game_state.isOurKickoff())
        {
            areas_to_avoid.set((uint32_t)AvoidArea::HALF_METER_AROUND_BALL, true);
        }

        if (game_state.isOurPenalty())
        {
            areas_to_avoid.set((uint32_t)AvoidArea::ENEMY_DEFENSE_AREA, true);
        }
        else
        {
            areas_to_avoid.set((uint32_t)AvoidArea::INFLATED_ENEMY_DEFENSE_AREA, true);
        }
    }

    return areas_to_avoid;
}
